#pragma once
#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <atomic>
#include <string>
#include <sstream>
#include "bonxai_map/occupancy_map.hpp"

namespace RollingBonxai
{
/**
 * @brief Type alias for linear velocity in 3D space
 * 
 * Uses Eigen::Vector3d for efficient linear algebra operations.
 * Intended for future velocity-based features (e.g., predictive preloading).
 */
using LinearVelocity3D = Eigen::Vector3d;
using Position3D = Eigen::Vector3d;

struct MapParams
{
  double resolution{0.0};
  double occupancy_min_z{0.0};
  double occupancy_max_z{0.0};
  double occupancy_threshold{0.50};

  double sensor_max_range{0.0};
  double sensor_hit{0.0};
  double sensor_miss{0.0};
  double sensor_min{0.0};
  double sensor_max{0.0};
};

/**
 * @brief Chunk coordinate in grid space using center-origin convention
 * 
 * Represents a chunk's integer coordinate where chunk (0,0,0) is centered at
 * world position (0,0,0). Each chunk coordinate maps to a cubic region of
 * world space with size determined by the ChunkCoordinateSystem.
 */
struct ChunkCoord
{
    int32_t x;  /// X-axis chunk coordinate
    int32_t y;  /// Y-axis chunk coordinate
    int32_t z;  /// Z-axis chunk coordinate

    /**
     * @brief Default constructor initializing all coordinates to (0,0,0)
     */
    ChunkCoord();

    /**
     * @brief Parameterized constructor
     * @param x_ X-axis chunk coordinate
     * @param y_ Y-axis chunk coordinate
     * @param z_ Z-axis chunk coordinate
     */
    ChunkCoord(int32_t x_, int32_t y_, int32_t z_);

    /**
     * @brief Equality comparison operator
     * @param other Chunk coordinate to compare against
     * @return true if all coordinates are equal
     */
    bool operator==(const ChunkCoord& other) const;

    /**
     * @brief Inequality comparison operator
     * @param other Chunk coordinate to compare against
     * @return true if any coordinate differs
     */
    bool operator!=(const ChunkCoord& other) const;

    /**
     * @brief Addition operator for coordinate offsets
     * @param offset Offset to add
     * @return New ChunkCoord with offset applied
     */
    ChunkCoord operator+(const ChunkCoord& offset) const;

    /**
     * @brief Subtraction operator for coordinate offsets
     * @param offset Offset to subtract
     * @return New ChunkCoord with offset applied
     */
    ChunkCoord operator-(const ChunkCoord& offset) const;

    /**
     * @brief Compound addition assignment operator
     * @param offset Offset to add in-place
     * @return Reference to this after modification
     */
    ChunkCoord& operator+=(const ChunkCoord& offset);

    /**
     * @brief Compound subtraction assignment operator
     * @param offset Offset to subtract in-place
     * @return Reference to this after modification
     */
    ChunkCoord& operator-=(const ChunkCoord& offset);
}; // ChunkCoord


/**
 * @brief Hash function for ChunkCoord using FNV-1a algorithm
 * 
 * Implements the Fowler-Noll-Vo 1a (FNV-1a) 64-bit hash for efficient
 * use in std::unordered_map and std::unordered_set.
 * 
 * @note Algorithm: hash = (hash XOR byte) × prime, for each coordinate
 * @see https://en.wikipedia.org/wiki/Fowler-Noll-Vo_hash_function
 */
struct ChunkCoordHash 
{
    /**
     * @brief Compute FNV-1a hash of chunk coordinate
     * @param c Chunk coordinate to hash
     * @return 64-bit hash value
     * @complexity O(1)
     */
    std::size_t operator()(const ChunkCoord& c) const noexcept {
        std::size_t h = 14695981039346656037ull; // FNV-1a 64-bit offset

        //XOR first, then prime for each byte

        h ^= static_cast<std::uint32_t>(c.x);
        h *= 1099511628211ull; // FNV-1a 64-bit prime

        h ^= static_cast<std::uint32_t>(c.y);
        h *= 1099511628211ull;

        h ^= static_cast<std::uint32_t>(c.z);
        h *= 1099511628211ull;

        return h;
    }
};

/**
 * @brief Timestamp metadata stored alongside chunk data
 * 
 * Timestamps are stored in nanoseconds since epoch (int64_t) for precision.
 * This enables temporal policies and analytics in future versions.
 */
struct ChunkTimestamp 
{
    int64_t creation_time_ns;      /// When chunk first created
    int64_t last_modified_ns;      /// When chunk last saved
    int64_t last_accessed_ns;      /// When chunk last loaded
    uint64_t access_count;         /// How many times accessed
    
    ChunkTimestamp();
    
    // Conversion helpers
    std::chrono::system_clock::time_point getCreationTime() const;
    std::chrono::system_clock::time_point getLastModified() const;
    std::chrono::system_clock::time_point getLastAccessed() const;
    
    // Age calculations
    std::chrono::seconds getAge() const;
    std::chrono::seconds getTimeSinceModified() const;
    std::chrono::seconds getTimeSinceAccessed() const;

    //Get access count
    uint64_t getAccessCount() const;
};

/**
 * @brief Metadata stored 
 * 
 * Timestamps are stored in nanoseconds since epoch (int64_t) for precision.
 * This enables temporal policies and analytics in future versions.
 */
struct ChunkMetadata {
    int64_t creation_time{0};
    int64_t last_access_time{0};
    uint64_t access_count{0};
    bool is_dirty {false};
};


/**
 * @brief ManagedChunk is a wrapper around an OccupancyMap (aka a chunk) to handle
 * the lifetimes of the chunk deterministically
 * 
 * It also provides utilities to track the "dirtiness" of the chunk
 */

class ManagedChunk 
{
    using MapUPtr = std::unique_ptr<Bonxai::OccupancyMap>;
public:
    /**
     * @brief
     */
    explicit ManagedChunk(MapUPtr map, const ChunkCoord& chunk_coordinate, bool dirty = false);

    /**
     * @brief
     */
    ~ManagedChunk() = default;

    /**
     * @brief
     */
    ManagedChunk(ManagedChunk&& other) noexcept;
    
    /**
     * @brief
     */
    ManagedChunk& operator=(ManagedChunk&& other) noexcept;
    
    /**
     * @brief
     */
    ManagedChunk(const ManagedChunk&) = delete;

    /**
     * @brief
     */
    ManagedChunk& operator=(const ManagedChunk&) = delete;

    /**
     * @brief Readonly const access to underlying map
     */
    [[nodiscard]] const Bonxai::OccupancyMap* getConstMap() const;

    /**
     * @brief Mutable access to underlying map
     */
    [[nodiscard]] Bonxai::OccupancyMap* getMutableMap();

    /**
     * @brief Get the chunk coord
     */
    [[nodiscard]] ChunkCoord getChunkCoord() const;

    /**
     * @brief Get the chunk coord as a str
     */
    [[nodiscard]] std::string getChunkCoordStr() const;

    /**
     * @brief Check if it is dirty
     */
    [[nodiscard]] bool isDirty() const;

    /**
     * @brief Mark as dirty
     */
    void markDirty();

    /**
     * @brief
     */
    void markClean();

    /**
     * @brief Transfer ownership to another unique pointer
     * This will leave the unique pointer as a nullptr_t
     */
    [[nodiscard]] MapUPtr transferMapOwnership();

    /**
     * @brief Check if the map is valid, basically just a check if the map is a nullptr
     */
    [[nodiscard]] bool isMapValid() const;
    
private:
    
    /// @brief An owning unique pointer to the chunk
    MapUPtr map_; 

    /// @brief The chunk coordinate corresponding to this chunk
    ChunkCoord chunk_coordinate_;

    /// @brief Threadsafe dirty tracking, true means it has been updated before since its last load
    std::atomic<bool> is_dirty_;
};




} // namespace Rolling Bonxai