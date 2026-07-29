#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rolling_bonxai/common.hpp"
#include "rolling_bonxai/task_queue.hpp"
#include "rolling_bonxai/file_storage_backend.hpp"
#include "bonxai_map/occupancy_map.hpp"

namespace RollingBonxai
{
/**
 * @brief Configuration for AsyncChunkManager
 */
struct AsyncChunkManagerConfig
{
    bool enable_io{true};
    size_t num_load_threads{3};     ///< Number of dedicated load worker threads
    size_t num_save_threads{1};     ///< Number of dedicated save worker threads
    
    /// Timeout for graceful shutdown (waiting for saves to drain)
    std::chrono::seconds shutdown_timeout{30};
};

/**
 * @brief Statistics for async I/O operations
 */
struct AsyncIOStats
{
    std::atomic<uint64_t> loads_requested{0};
    std::atomic<uint64_t> loads_completed{0};
    std::atomic<uint64_t> loads_failed{0};
    std::atomic<uint64_t> saves_requested{0};
    std::atomic<uint64_t> saves_completed{0};
    std::atomic<uint64_t> saves_failed{0};
};

/**
 * @brief Snapshot of async I/O statistics (non-atomic, safe to copy)
 */
struct AsyncIOStatsSnapshot
{
    uint64_t loads_requested{0};
    uint64_t loads_completed{0};
    uint64_t loads_failed{0};
    uint64_t saves_requested{0};
    uint64_t saves_completed{0};
    uint64_t saves_failed{0};
    size_t pending_loads{0};
    size_t pending_saves{0};
};

/**
 * @class AsyncChunkManager
 * @brief Manages asynchronous loading and saving of occupancy map chunks
 * 
 * Provides non-blocking chunk I/O using dedicated worker threads.
 * Uses separate thread pools for loads and saves so that saves never block loads.
 * 
 * Load operations use a priority queue (higher priority loads processed first).
 * Save operations use a FIFO queue.
 * 
 * Duplicate load prevention:
 * - Layer 1 (caller): RollingBonsai checks active_chunks_ and pending_loads_ before calling
 * - Layer 2 (this class): loadAsync() rejects requests for coords already in pending_loads_
 * 
 * Ownership model for saves:
 * - The caller must std::move the VoxelGrid out of the OccupancyMap before calling saveAsync().
 *   This transfers ownership of the grid data to the save pipeline, avoiding shared state 
 *   between the main thread and save workers.
 * - After moving the grid out, the source OccupancyMap is in an invalid state and must be
 *   removed from chunk storage by the caller.
 * 
 * @thread_safety
 * - loadAsync() and saveAsync() are safe to call from the main thread.
 * - Worker threads access the FileStorageBackend concurrently. Since each operation
 *   targets a different file (keyed by ChunkCoord), this is safe for FileStorageBackend.
 * - Internal queues and pending_loads_ map are protected by their own mutexes.
 * 
 * @example
 * @code
 * auto backend = std::make_unique<FileStorageBackend>("/var/chunks");
 * backend->initStorageBackend();
 * 
 * AsyncChunkManagerConfig config;
 * AsyncChunkManager manager(std::move(backend), config);
 * 
 * // Async load
 * auto future = manager.loadAsync(coord, 0.8f, grid_factory);
 * if (future.has_value()) {
 *     // Store future, poll later with future->wait_for(0s)
 * }
 * 
 * // Async save (caller has moved the VoxelGrid out of the OccupancyMap)
 * auto save_future = manager.saveAsync(coord, std::move(voxel_grid));
 * @endcode
 */
class AsyncChunkManager
{
public:
    /// Factory function type for creating new OccupancyMaps when a chunk has no data on disk
    using OccupancyMapFactory = std::function<std::unique_ptr<Bonxai::OccupancyMap>()>;
    using OccupancyOptionsFactory = std::function<Bonxai::Occupancy::OccupancyOptions()>;

    /**
     * @brief Construct and start worker threads
     * 
     * @param backend Owning pointer to FileStorageBackend (must be initialized)
     * @param config  Thread pool and shutdown configuration
     * 
     * @pre backend->isBackendInit() == true
     * @post Worker threads are running and ready to process requests
     */
    AsyncChunkManager(std::unique_ptr<FileStorageBackend> backend,
                      const AsyncChunkManagerConfig& config = AsyncChunkManagerConfig{});

    /**
     * @brief Destructor - performs graceful shutdown
     *
     * Drains queued and in-flight saves (bounded by config.shutdown_timeout),
     * then shuts down both queues and joins all worker threads. Queued loads
     * are abandoned, which is harmless — an unfinished load has no data to lose.
     */
    ~AsyncChunkManager();

    // Non-copyable, non-movable
    AsyncChunkManager(const AsyncChunkManager&) = delete;
    AsyncChunkManager& operator=(const AsyncChunkManager&) = delete;
    AsyncChunkManager(AsyncChunkManager&&) = delete;
    AsyncChunkManager& operator=(AsyncChunkManager&&) = delete;

    /**
     * @brief Queue an asynchronous chunk load
     * 
     * Returns immediately. The actual disk I/O happens on a load worker thread.
     * 
     * @param coord     Chunk coordinate to load
     * @param priority  Load priority (higher = loaded sooner). Range [0.0, 1.0].
     * @param factory   Factory function to create a new OccupancyMap if chunk
     *                  does not exist on disk. The factory must produce a valid
     *                  OccupancyMap with the correct resolution.
     * 
     * @return std::optional containing a future to the loaded OccupancyMap.
     *         Returns std::nullopt if:
     *         - The coord is already in pending_loads_ (duplicate, Layer 2 rejection)
     *         - The manager has been shut down
     * 
     * @note The returned OccupancyMap is constructed from the deserialized VoxelGrid.
     *       If the chunk does not exist on disk, the factory is used to create a new one.
     */
    [[nodiscard]] std::optional<std::future<std::unique_ptr<Bonxai::OccupancyMap>>>
    loadAsync(const ChunkCoord& coord, float priority, 
              OccupancyMapFactory map_fac,OccupancyOptionsFactory opt_fac);

   /**
     * @brief Check if a chunk is currently being loaded
     * 
     * @param coord Chunk coordinate to check
     * @return true if coord is in the pending_loads_ map
     */
    [[nodiscard]] bool isLoading(const ChunkCoord& coord) const;

    /**
     * @brief Queue an asynchronous chunk save
     * 
     * Returns immediately. The actual disk I/O happens on a save worker thread.
     * 
     * @param coord Chunk coordinate
     * @param grid  VoxelGrid to save (ownership transferred via move).
     *              The caller must have extracted this from the OccupancyMap
     *              via std::move on getGrid(). The OccupancyMap is invalid after this.
     * 
     * @return Future that resolves to true on success, false on failure.
     *         Returns a ready future with false if the manager has been shut down.
     * 
     * @note The caller is responsible for:
     *       1. Checking isDirty() before calling (don't save clean chunks)
     *       2. Moving the VoxelGrid out of the OccupancyMap
     *       3. Removing the (now invalid) OccupancyMap from chunk storage
     */
    [[nodiscard]] std::future<bool> 
    saveAsync(const ChunkCoord& coord,
              Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>&& grid);

    /**
     * @brief Wait for all pending save operations to complete
     * 
     * Blocks until the save queue is empty and all in-flight saves have finished.
     * Uses the shutdown_timeout from config. If timeout expires, returns false.
     * 
     * @return true if all saves completed, false if timed out
     * 
     * @note Call this before destruction if you need to guarantee data persistence.
     *       The destructor does NOT call this automatically.
     */
    [[nodiscard]] bool waitForPendingSaves();

    /**
     * @brief Get a snapshot of current I/O statistics
     * 
     * @return AsyncIOStatsSnapshot (non-atomic copy, safe to read)
     */
    [[nodiscard]] AsyncIOStatsSnapshot getStats() const;

    /**
     * @brief Get number of pending load requests (queued + in-flight)
     */
    [[nodiscard]] size_t pendingLoadCount() const;

    /**
     * @brief Get number of pending save requests (queued + in-flight)
     */
    [[nodiscard]] size_t pendingSaveCount() const;

private:

    /**
     * @brief Load request queued for worker threads
     * 
     * Ordering: higher priority first, then FIFO (earlier enqueue_time first).
     * operator< is defined for std::priority_queue (max-heap), so it returns true
     * when `this` has LOWER priority than `other` (lower priority = sorted earlier
     * in the underlying container = popped last).
     * Strictly an internal type
     */
    struct LoadRequest
    {
        ChunkCoord coord;
        float priority;
        OccupancyMapFactory map_factory;
        OccupancyOptionsFactory options_factory;
        std::shared_ptr<std::promise<std::unique_ptr<Bonxai::OccupancyMap>>> promise;
        std::chrono::steady_clock::time_point enqueue_time;

        /// Ordering for max-heap: lower priority (or later time) = "less than"
        bool operator<(const LoadRequest& other) const
        {
            if (priority != other.priority) {
                return priority < other.priority;  // lower priority → less than → popped last
            }
            return enqueue_time > other.enqueue_time;  // later time → less than → popped last
        }
    };

    /**
     * @brief Save request queued for worker threads (FIFO, no priority)
     * Strictly an internal type
     */
    struct SaveRequest
    {
        ChunkCoord coord;
        std::unique_ptr<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>> griduptr;  // Owns the moved grid
        std::shared_ptr<std::promise<bool>> promise;

    };

    /**
     * @brief Load worker loop - runs on each load worker thread
     * 
     * Blocks on load_queue_.pop() until a request arrives or shutdown.
     * Loads from backend, deserializes into OccupancyMap, fulfills promise.
     */
    void loadWorkerLoop();

    /**
     * @brief Save worker loop - runs on each save worker thread
     * 
     * Blocks on save_queue_.pop() until a request arrives or shutdown.
     * Serializes grid, saves to backend, fulfills promise.
     */
    void saveWorkerLoop();

    /**
     * @brief Remove a coord from the pending_loads_ map
     * 
     * Called by load workers after fulfilling a load promise.
     * 
     * @param coord Chunk coordinate to remove
     */
    void removePendingLoad(const ChunkCoord& coord);


    /// Configuration
    AsyncChunkManagerConfig config_;

    /// Storage backend (owned)
    std::unique_ptr<FileStorageBackend> backend_;

    /// Load queue (priority: higher priority loads first)
    ThreadSafePriorityQueue<LoadRequest> load_queue_;

    /// Save queue (FIFO)
    ThreadSafeQueue<SaveRequest> save_queue_;

    /// Pending loads tracker for duplicate prevention (Layer 2)
    std::unordered_map<
        ChunkCoord,
        std::shared_ptr<std::promise<std::unique_ptr<Bonxai::OccupancyMap>>>,
        ChunkCoordHash
    > pending_loads_;
    mutable std::mutex pending_loads_mutex_;

    /// Load Worker threads
    std::vector<std::thread> load_workers_;

    /// Save Worker threads
    std::vector<std::thread> save_workers_;

    /// Shutdown flag
    std::atomic<bool> stop_{false};

    /// In-flight save counter (for waitForPendingSaves)
    std::atomic<size_t> inflight_saves_{0};
    std::mutex saves_done_mutex_;
    std::condition_variable saves_done_cv_;

    /// Statistics
    AsyncIOStats stats_;
};

}