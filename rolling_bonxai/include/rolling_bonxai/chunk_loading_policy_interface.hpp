#pragma once

#include "rolling_bonxai/common.hpp"
#include "rolling_bonxai/coordinate_system.hpp"

#include <chrono>
#include <string>
#include <unordered_map>

namespace RollingBonxai
{

/**
 * @brief Context provided to chunk loading policies for decision-making
 * 
 * Contains read-only information about the current state of the system,
 * including robot position, current chunk, time, and chunk metadata.
 * Policies use this context to determine which chunks to load/evict.
 */
struct PolicyContext 
{
    /// Current ego position in world coordinates (meters)
    Eigen::Vector3d robot_position;
    
    /// Current chunk coordinate the robot occupies
    ChunkCoord current_chunk;
    
    /// Current system time (for temporal policies)
    std::chrono::steady_clock::time_point current_time;
    
    /// Reference to chunk metadata map (read-only from policy perspective)
    /// Maps ChunkCoord -> ChunkMetadata for all known chunks
    const std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash>& metadata;
    
    /**
     * @brief Default constructor
     * @param robot_pos Robot position in world frame
     * @param current Current chunk coordinate
     * @param time Current timestamp
     * @param meta Reference to metadata map
     */
    PolicyContext(const Eigen::Vector3d& robot_pos,
                  const ChunkCoord& current,
                  const std::chrono::steady_clock::time_point& time,
                  const std::unordered_map<ChunkCoord, ChunkMetadata, ChunkCoordHash>& meta)
        : robot_position(robot_pos),
          current_chunk(current),
          current_time(time),
          metadata(meta)
    {}
    
    /**
     * @brief Get metadata for a specific chunk
     * @param coord Chunk coordinate to query
     * @return Pointer to metadata if found, nullptr otherwise
     * 
     * @note Helper method for convenient metadata access
     */
    [[nodiscard]] const ChunkMetadata* getMetadata(const ChunkCoord& coord) const {
        auto it = metadata.find(coord);
        return (it != metadata.end()) ? &it->second : nullptr;
    }
}; // struct PolicyContext


/**
 * @class ChunkLoadingPolicy
 * @brief Abstract interface for chunk management policies
 * 
 * Defines the contract for policies that determine which chunks should be
 * loaded into memory and which should be evicted. Different robot types and
 * use cases require different strategies.
 * 
 * @performance
 * - shouldLoad/shouldEvict: Target < 1Âµs per call
 * - getPriority: Target < 100ns per call
 * - All methods must be const (no side effects)
 * 
 * @thread_safety
 * - All methods are const and should be thread-safe for concurrent reads
 * - No mutable state allowed in policy implementations
 * 
 * @note Policies are evaluated frequently (every update cycle), so performance
 *       is critical. Avoid expensive computations or allocations.
 * 
 * @example
 * @code
 * class MyPolicy : public ChunkLoadingPolicy {
 * public:
 *     bool shouldLoad(const ChunkCoord& coord, const PolicyContext& ctx) const override {
 *         // Custom logic here
 *         return true;
 *     }
 *     // ... implement other methods
 * };
 * @endcode
 */
class ChunkLoadingPolicy 
{
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~ChunkLoadingPolicy() = default;
    
    /**
     * @brief Determine if a chunk should be loaded into memory
     * 
     * Called to decide whether a chunk should be loaded based on current
     * system state. This is the primary method for chunk selection.
     * 
     * @param coord Chunk coordinate to evaluate
     * @param context Current system context (robot position, time, metadata)
     * @return true if chunk should be loaded, false otherwise
     * 
     * @note Must be const (no side effects)
     * @note Called frequently - optimize for performance
     */
    [[nodiscard]] virtual bool shouldLoad(const ChunkCoord& coord,
                                          const PolicyContext& context) const = 0;
    
    /**
     * @brief Determine if a chunk should be evicted from memory
     * 
     * Called to decide whether a currently-loaded chunk should be evicted.
     * Typically the inverse of shouldLoad, but can implement custom logic
     * (e.g., hysteresis, grace periods).
     * 
     * @param coord Chunk coordinate to evaluate
     * @param context Current system context
     * @return true if chunk should be evicted, false to keep in memory
     * 
     * @note Must be const (no side effects)
     * @note Default implementation: return !shouldLoad(coord, context)
     */
    [[nodiscard]] virtual bool shouldEvict(const ChunkCoord& coord,
                                           const PolicyContext& context) const = 0;
    
    /**
     * @brief Get loading priority for a chunk
     * 
     * Used to order the loading queue when multiple chunks need to be loaded.
     * Higher priority chunks are loaded first.
     * 
     * @param coord Chunk coordinate to evaluate
     * @param context Current system context
     * @return Priority value in range [0.0, 1.0]
     *         - 0.0 = lowest priority (load last)
     *         - 1.0 = highest priority (load first)
     * 
     * @note Must be const (no side effects)
     * @note Relative ordering matters more than absolute values
     * @note For chunks that shouldn't load, return 0.0
     */
    [[nodiscard]] virtual double getPriority(const ChunkCoord& coord,
                                             const PolicyContext& context) const = 0;

    /**
     * @brief Chebyshev radius (in chunks) that must be enumerated to find every
     *        chunk this policy could possibly want loaded
     *
     * The orchestrator walks the cube of neighbours within this radius and asks
     * shouldLoad() about each one. A policy that can accept a chunk outside the
     * returned radius would simply never see it offered, so implementations must
     * return an upper bound over every axis, not just the horizontal one.
     *
     * @return Radius in chunks (>= 0)
     */
    [[nodiscard]] virtual int getSearchRadius() const = 0;

    /**
     * @brief Get human-readable name of the policy
     * 
     * Used for logging, debugging, and user interfaces. Should be descriptive
     * and include key parameters.
     * 
     * @return Policy name (e.g., "NeighborhoodPolicy(radius=1)")
     * 
     * @note Default implementation returns base class name
     */
    [[nodiscard]] virtual std::string getName() const {
        return "ChunkLoadingPolicy";
    }
    
    /**
     * @brief Get policy description for documentation
     * 
     * Provides a detailed description of what the policy does and when
     * to use it. Optional, defaults to empty string.
     * 
     * @return Human-readable description
     */
    [[nodiscard]] virtual std::string getDescription() const {
        return "";
    }
    
}; // class ChunkLoadingPolicy

} // namespace RollingBonxai