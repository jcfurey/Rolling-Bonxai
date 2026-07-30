#include "rolling_bonxai/async_chunk_manager.hpp"

#include <sstream>
#include <iostream>

namespace RollingBonxai
{
AsyncChunkManager::AsyncChunkManager(
    std::unique_ptr<FileStorageBackend> backend,
    const AsyncChunkManagerConfig& config)
    : config_(config),
      backend_(std::move(backend)),
      stop_(false) 
{
    // Spawn load workers, reserve space first
    load_workers_.reserve(config_.num_load_threads);
    for (size_t i = 0; i < config_.num_load_threads; ++i) {
        // build the threads in place
        load_workers_.emplace_back(&AsyncChunkManager::loadWorkerLoop, this);
    }

    // Spawn save workers
    save_workers_.reserve(config_.num_save_threads);
    for (size_t i = 0; i < config_.num_save_threads; ++i) {
        // build the threads in place
        save_workers_.emplace_back(&AsyncChunkManager::saveWorkerLoop, this);
    }
}

AsyncChunkManager::~AsyncChunkManager()
{
    // Drain queued saves BEFORE anything is torn down. Setting stop_ or shutting
    // the save queue down first would discard every dirty chunk still waiting to
    // be written, silently losing map data on every clean exit.
    if (!waitForPendingSaves()) {
        std::cerr << "[AsyncChunkManager] Destructor: pending saves did not drain "
                     "within the shutdown timeout; some chunks may be lost"
                  << std::endl;
    }

    // Signal all workers to stop
    stop_.store(true);

    // Shutdown both queues — this unblocks any workers waiting on pop()
    load_queue_.shutdown();
    save_queue_.shutdown();

    // Join load workers
    for (auto& worker : load_workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    // Join save workers
    for (auto& worker : save_workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

std::optional<std::future<std::unique_ptr<Bonxai::OccupancyMap>>>
AsyncChunkManager::loadAsync(const ChunkCoord& coord, float priority, 
                             OccupancyMapFactory map_fac,OccupancyOptionsFactory opt_fac)
{
    // Reject if shut down
    if (stop_.load()) {
        return std::nullopt;
    }

    // Layer 2 duplicate prevention: check pending_loads_
    std::lock_guard<std::mutex> lock(pending_loads_mutex_);

    if (pending_loads_.find(coord) != pending_loads_.end()) {
        // Already loading this coord — reject duplicate
        std::ostringstream oss;
        oss << "[AsyncChunkManager] Duplicate load request rejected for coord ("
            << coord.x << ", " << coord.y << ", " << coord.z << ")";
        std::cerr << oss.str() << std::endl;
        return std::nullopt;
    }

    // Create promise and extract the future from the promise. We are now linked
    auto promise = std::make_shared<std::promise<std::unique_ptr<Bonxai::OccupancyMap>>>();
    // Return the future to the caller. The caller can check the future to see if there is valid values
    auto future = promise->get_future();

    // Register in pending_loads_, guaranteed to be unique because we already check for duplicate requests
    pending_loads_[coord] = promise;

    // Build and enqueue load request
    LoadRequest request;
    request.coord = coord;
    request.priority = priority;
    request.map_factory = std::move(map_fac);
    request.options_factory = std::move(opt_fac);
    request.promise = promise;
    request.enqueue_time = std::chrono::steady_clock::now();

    // Update the stats
    stats_.loads_requested.fetch_add(1, std::memory_order_relaxed);

    try {
        load_queue_.push(std::move(request));
    } catch (const std::runtime_error&) {
        // Queue was shut down between our check and push — clean up
        pending_loads_.erase(coord);
        return std::nullopt;
    }
    // Caller will place the future "somewhere"
    return future;
}

bool AsyncChunkManager::isLoading(const ChunkCoord& coord) const
{
    // lock the pending_load_ and check to see if this coord is still loading
    std::lock_guard<std::mutex> lock(pending_loads_mutex_);
    return pending_loads_.find(coord) != pending_loads_.end();
}

std::future<bool> 
AsyncChunkManager::saveAsync(const ChunkCoord& coord,
                             Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>&& grid)
{
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();

    // If shut down, return immediate failure
    if (stop_.load()) {
        promise->set_value(false);
        return future;
    }

    // add to the inflight saves
    stats_.saves_requested.fetch_add(1, std::memory_order_relaxed);

    // Increment in-flight counter before enqueue so waitForPendingSaves
    // never sees a transient state where the queue is empty but work is pending
    inflight_saves_.fetch_add(1, std::memory_order_acq_rel);

    // The request packet
    SaveRequest request;
    request.coord = coord;
    request.griduptr = std::make_unique<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>>(std::move(grid));
    request.promise = promise;

    try {
        save_queue_.push(std::move(request));
    } catch (const std::runtime_error&) {
        // Queue was shut down — decrement counter, report failure
        inflight_saves_.fetch_sub(1, std::memory_order_acq_rel);
        promise->set_value(false);
    }

    return future;
}

bool AsyncChunkManager::waitForPendingSaves()
{
    // Nothing to drain, and no worker to notify us — return before waiting so a
    // manager with num_save_threads == 0 (or one that never saved) exits at once.
    if (save_queue_.empty() && inflight_saves_.load(std::memory_order_acquire) == 0) {
        return true;
    }

    std::unique_lock<std::mutex> lock(saves_done_mutex_);

    // wait until the save queue is empty and inflight saves are all done. or timeout first
    bool completed = saves_done_cv_.wait_for(
        lock,
        config_.shutdown_timeout,
        [this]() {
            return save_queue_.empty() && inflight_saves_.load(std::memory_order_acquire) == 0;
        }
    );

    if (!completed) {
        std::cerr << "[AsyncChunkManager] waitForPendingSaves timed out after "
                  << config_.shutdown_timeout.count() << "s. "
                  << "Pending saves: " << inflight_saves_.load() 
                  << ", Queue size: " << save_queue_.size() << std::endl;
    }

    return completed;
}

AsyncIOStatsSnapshot AsyncChunkManager::getStats() const
{
    AsyncIOStatsSnapshot snapshot;
    snapshot.loads_requested = stats_.loads_requested.load(std::memory_order_relaxed);
    snapshot.loads_completed = stats_.loads_completed.load(std::memory_order_relaxed);
    snapshot.loads_failed = stats_.loads_failed.load(std::memory_order_relaxed);
    snapshot.saves_requested = stats_.saves_requested.load(std::memory_order_relaxed);
    snapshot.saves_completed = stats_.saves_completed.load(std::memory_order_relaxed);
    snapshot.saves_failed = stats_.saves_failed.load(std::memory_order_relaxed);
    snapshot.pending_loads = pendingLoadCount();
    snapshot.pending_saves = pendingSaveCount();
    return snapshot;
}

size_t AsyncChunkManager::pendingLoadCount() const
{
    std::lock_guard<std::mutex> lock(pending_loads_mutex_);
    return pending_loads_.size();
}

size_t AsyncChunkManager::pendingSaveCount() const
{
    return inflight_saves_.load(std::memory_order_acquire);
}

// ============================================================================
// Worker Loops
// ============================================================================

void AsyncChunkManager::loadWorkerLoop()
{
    while (!stop_.load(std::memory_order_acquire)) {
        // Blocking pop — returns nullopt on shutdown
        auto request_opt = load_queue_.pop();
        if (!request_opt.has_value()) { /*Request will be nullopt if queues are shutdown AND queue is empty*/
            break;  // Queue shut down
        }

        LoadRequest& request = request_opt.value();

        // Every path out of this block must leave the promise satisfied exactly
        // once and must not let an exception escape: an uncaught throw on a
        // worker thread terminates the process, and a second set_value() on an
        // already-satisfied promise throws std::future_error.
        try {
            std::unique_ptr<Bonxai::VoxelGrid<Bonxai::Occupancy::CellOcc>> grid_ptr{nullptr};
            if (config_.enable_io) {
                grid_ptr = backend_->load(request.coord);
            }

            // result to store
            std::unique_ptr<Bonxai::OccupancyMap> result;
            if (grid_ptr) {
                // Chunk existed on disk — construct OccupancyMap from deserialized VoxelGrid
                auto options = request.options_factory();
                result = std::make_unique<Bonxai::OccupancyMap>(
                    options, std::move(*grid_ptr)
                );
            }
            else {
                result = request.map_factory();
            }

            // Fulfill promise
            request.promise->set_value(std::move(result));
            stats_.loads_completed.fetch_add(1, std::memory_order_relaxed);
        }
        catch (const std::exception& e) {
            // Propagate the failure to the caller through the promise so that
            // pollPendingLoads() reports it instead of blocking forever.
            try {
                request.promise->set_exception(std::current_exception());
            } catch (...) {/*Promise already satisfied (shouldn't happen, but be defensive)*/}
            stats_.loads_failed.fetch_add(1, std::memory_order_relaxed);

            std::ostringstream oss;
            oss << "[AsyncChunkManager] Load failed for coord ("
                << request.coord.x << ", " << request.coord.y << ", " << request.coord.z
                << "): " << e.what();
            std::cerr << oss.str() << std::endl;
        }

        // Remove from pending_loads_ regardless of success/failure
        removePendingLoad(request.coord);
    }
}

void AsyncChunkManager::saveWorkerLoop()
{
    while (!stop_.load(std::memory_order_acquire)) {
        // Blocking pop — returns nullopt on shutdown
        auto request_opt = save_queue_.pop();
        if (!request_opt.has_value()) {
            break;  // Queue shut down
        }

        SaveRequest& request = request_opt.value();

        try {
            
            bool success = config_.enable_io ? backend_->save(request.coord, *request.griduptr) : true;
            request.promise->set_value(success);

            if (success) {
                stats_.saves_completed.fetch_add(1, std::memory_order_relaxed);
            } else {
                stats_.saves_failed.fetch_add(1, std::memory_order_relaxed);

                std::ostringstream oss;
                oss << "[AsyncChunkManager] Save returned false for coord ("
                    << request.coord.x << ", " << request.coord.y << ", " << request.coord.z << ")";
                std::cerr << oss.str() << std::endl;
            }

        } catch (const std::exception& e) {
            try {
                request.promise->set_value(false);
            } catch (...) {
                // Promise already satisfied
            }
            stats_.saves_failed.fetch_add(1, std::memory_order_relaxed);

            std::ostringstream oss;
            oss << "[AsyncChunkManager] Save threw exception for coord ("
                << request.coord.x << ", " << request.coord.y << ", " << request.coord.z
                << "): " << e.what();
            std::cerr << oss.str() << std::endl;
        }

        // Decrement in-flight counter and notify waitForPendingSaves.
        // The decrement has to happen under saves_done_mutex_: waitForPendingSaves
        // evaluates its predicate while holding that lock, so decrementing outside
        // it lets the last save land between the predicate check and the wait,
        // costing a full shutdown_timeout of dead time on every clean exit.
        {
            std::lock_guard<std::mutex> lock(saves_done_mutex_);
            inflight_saves_.fetch_sub(1, std::memory_order_acq_rel);
        }
        saves_done_cv_.notify_all();
    }
}

void AsyncChunkManager::removePendingLoad(const ChunkCoord& coord)
{
    std::lock_guard<std::mutex> lock(pending_loads_mutex_);
    pending_loads_.erase(coord);
}

}