#pragma once

#include "rolling_bonxai/chunk_loading_policy_interface.hpp"

namespace RollingBonxai
{

/**
 * @class NeighborhoodPolicy
 * @brief Load all chunks within Chebyshev distance (max absolute difference) from current chunk
 * 
 * This is the default and most common policy. It loads all chunks within a cubic
 * neighborhood around the robot's current chunk.
 * 
 * @details
 * - Uses Chebyshev distance (L∞ metric): max(|dx|, |dy|, |dz|)
 * - Radius of 1 loads 26 neighbors (3³ - 1)
 * - Radius of 2 loads 124 neighbors (5³ - 1)
 * - Priority inversely proportional to distance (closer = higher priority)
 * 
 * @performance
 * - shouldLoad: O(1), ~10ns (3 subtractions + 3 abs + 1 max + 1 comparison)
 * - getPriority: O(1), ~15ns (same as shouldLoad + 1 division)
 * 
 * @use_cases
 * - General-purpose robotics
 * - Aerial robots needing full 3D awareness
 * - Fast-moving robots (increase radius)
 * 
 * @example
 * @code
 * auto policy = std::make_unique<NeighborhoodPolicy>(1);  // radius=1, 26 neighbors
 * rolling_bonsai.setPolicy(std::move(policy));
 * @endcode
 */
class NeighborhoodPolicy : public ChunkLoadingPolicy
{
public:
    /**
     * @brief Construct neighborhood policy with specified radius
     * @param radius Chebyshev distance radius (in chunks)
     * @throws std::invalid_argument if radius < 0
     * 
     * @note Typical values: 1 (26 chunks), 2 (124 chunks), 3 (342 chunks)
     */
    explicit NeighborhoodPolicy(int radius = 1);
    
    [[nodiscard]] bool shouldLoad(const ChunkCoord& coord,
                                  const PolicyContext& context) const override;
    
    [[nodiscard]] bool shouldEvict(const ChunkCoord& coord,
                                   const PolicyContext& context) const override;
    
    [[nodiscard]] double getPriority(const ChunkCoord& coord,
                                     const PolicyContext& context) const override;
    
    [[nodiscard]] std::string getName() const override;
    
    [[nodiscard]] std::string getDescription() const override;

    [[nodiscard]] int getSearchRadius() const override;

    /**
     * @brief Get configured radius
     * @return Radius in chunks
     */
    [[nodiscard]] int getRadius() const;

private:
    int radius_;  ///< Chebyshev distance radius (in chunks)

}; // class NeighborhoodPolicy


/**
 * @class PlanarNeighborhoodPolicy
 * @brief Neighborhood policy with vertical (Z-axis) restriction for ground robots
 * 
 * Extends NeighborhoodPolicy with a vertical range constraint. Useful for ground
 * robots that don't need to load chunks far above or below ground level.
 * 
 * @details
 * - Applies neighborhood radius on X and Y axes
 * - Restricts Z to range [z_min, z_max]
 * - User can also set the range to be relative to the ego's current position/chunk as opposed to a global frame
 *
 * - Under use_relative = false regime, the absolute z value will be compared with z_min. So, z_min and z_max
 * - refer to actual chunk coordinates. For example, [z_min=3,z_max=10] would state that we only load chunks
 * - between z coordinates 3 and 10 inclusive (3,4,5,6,7,8,9,10). Similarly, [z_min=-5,z_max=8] would mean we keep
 * - chunks with z coordinates from -5 all the way to +8. Therefore, negative values have semantic/spatial meaning
 * 
 * - Under the use_relative = true regime, the relative z value will be compared to z_min. As per the previous example,
 * - [z_min=3,z_max=10] means that we will load all coordinates 3 chunks below and 10 chunks above the current coordinate
 * - [z_min=0,z_max=5] means that we will load no chunk below the ego and 5 chunks above the agent. To drive the point home,
 * - [z_min=-5,z_max=20] means that while we load 20 chunks above the ego, there is no spatial meaning of loading -5 chunks.
 * - Therefore, under the use_relative regime, we CANNOT have any negative values as z_min already captures the idea of "below"
 * 
 * - Saves memory by not loading aerial/underground chunks
 * - The user can also select the use_relative flag to load chunks at a relative distance
 * - Example: robot at z=1, z_range=[0,2] loads ground±1 layer only
 * 
 * @performance
 * - shouldLoad: O(1), ~20ns (neighborhood check + 2 comparisons)
 * - getPriority: O(1), ~20ns
 * 
 * @memory_savings
 * - Radius=1, z_range=3 layers: 9 chunks vs 27 chunks (67% reduction)
 * - Radius=2, z_range=3 layers: 45 chunks vs 125 chunks (64% reduction)
 * 
 * @use_cases
 * - Ground robots (wheeled, tracked)
 * - Indoor navigation (flat environments)
 * - Warehouse robots (known floor levels)
 * 
 * @example
 * @code
 * // Load only ground level ± 1 layer (z=[0,2])
 * auto policy = std::make_unique<PlanarNeighborhoodPolicy>(1, 0, 2,false);
 * rolling_bonsai.setPolicy(std::move(policy));
 * @endcode
 */
class PlanarNeighborhoodPolicy : public ChunkLoadingPolicy
{
public:
    /**
     * @brief Construct planar neighborhood policy
     * 
     * @param radius Chebyshev distance radius on X-Y plane (in chunks)
     * @param z_min Minimum Z coordinate (chunk units, inclusive)
     * @param z_max Maximum Z coordinate (chunk units, inclusive)
     * @param use_relative If set to true, z_min and z_max become relative values to the robot's position
     * @throws std::invalid_argument if radius < 0 or z_min > z_max
     * 
     * @note Z coordinates are absolute chunk coordinates if use_relative is set to false, otherwise its relative
     */
    PlanarNeighborhoodPolicy(int radius, int z_min, int z_max, bool use_relative);
    
    [[nodiscard]] bool shouldLoad(const ChunkCoord& coord,
                                  const PolicyContext& context) const override;
    
    [[nodiscard]] bool shouldEvict(const ChunkCoord& coord,
                                   const PolicyContext& context) const override;
    
    [[nodiscard]] double getPriority(const ChunkCoord& coord,
                                     const PolicyContext& context) const override;
    
    [[nodiscard]] std::string getName() const override;
    
    [[nodiscard]] std::string getDescription() const override;

    /**
     * @brief Enumeration radius
     *
     * The vertical extent is independent of the horizontal radius, so the cube
     * that must be walked is bounded by the largest of the three.
     */
    [[nodiscard]] int getSearchRadius() const override;

    /**
     * @brief Get configured parameters
     */
    [[nodiscard]] int getRadius() const;
    [[nodiscard]] int getZMin() const;
    [[nodiscard]] int getZMax() const;
    [[nodiscard]] bool isRelative() const;
    
private:
    int radius_;  ///< Horizontal neighborhood radius
    int z_min_;   ///< Minimum Z coordinate (inclusive)
    int z_max_;   ///< Maximum Z coordinate (inclusive)
    bool use_relative_;
    
}; // class PlanarNeighborhoodPolicy


/**
 * @class TemporalNeighborhoodPolicy
 * @brief Neighborhood policy with age-based filtering for dynamic environments
 * 
 * Extends NeighborhoodPolicy by excluding chunks older than a specified age.
 * Useful in dynamic environments where old map data becomes stale.
 * 
 * @details
 * - Loads chunks within neighborhood radius
 * - Excludes chunks created more than max_age ago
 * - New chunks (no metadata) are always loaded
 * - Priority boosted for newer chunks
 * 
 * @use_cases
 * - Outdoor robots (changing terrain, weather)
 * - Dynamic environments (construction sites, warehouses)
 * - Long-running missions (multi-day operations)
 * - Environments with moving obstacles
 * 
 * @example
 * @code
 * // Load chunks within radius=1, but only if created within last hour
 * auto policy = std::make_unique<TemporalNeighborhoodPolicy>(
 *     1,                          // radius
 *     std::chrono::hours(1)       // max_age
 * );
 * rolling_bonxai.setPolicy(std::move(policy));
 * @endcode
 */
class TemporalNeighborhoodPolicy : public ChunkLoadingPolicy
{
public:
    /**
     * @brief Construct temporal neighborhood policy
     * 
     * @param radius Chebyshev distance radius (in chunks)
     * @param max_age Maximum chunk age (time since creation)
     * @throws std::invalid_argument if radius < 0
     * 
     * @note Chunks with no metadata (newly explored) are always loaded
     */
    TemporalNeighborhoodPolicy(int radius, double age_weightage, std::chrono::seconds max_age);
    
    [[nodiscard]] bool shouldLoad(const ChunkCoord& coord,
                                  const PolicyContext& context) const override;
    
    [[nodiscard]] bool shouldEvict(const ChunkCoord& coord,
                                   const PolicyContext& context) const override;
    
    [[nodiscard]] double getPriority(const ChunkCoord& coord,
                                     const PolicyContext& context) const override;
    
    [[nodiscard]] std::string getName() const override;
    
    [[nodiscard]] std::string getDescription() const override;

    [[nodiscard]] int getSearchRadius() const override;

    /**
     * @brief Get configured parameters
     */
    [[nodiscard]] int getRadius() const;
    [[nodiscard]] std::chrono::seconds getMaxAge() const;
    [[nodiscard]] double getAgeWeightage() const;
    
private:
    int radius_;                    ///< Chebyshev distance radius
    std::chrono::seconds max_age_;  ///< Maximum chunk age
    double age_weightage_;
    
}; // class TemporalNeighborhoodPolicy

} // namespace RollingBonxai