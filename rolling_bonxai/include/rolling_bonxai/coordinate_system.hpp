#pragma once
#include "bonxai_core/bonxai.hpp"
#include "bonxai_map/occupancy_map.hpp"
#include "rolling_bonxai/common.hpp"
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace RollingBonxai
{
/**
 * @brief Center-origin chunk coordinate system for spatial indexing
 * 
 * Manages conversions between world positions (meters) and chunk coordinates
 * using a center-origin convention. Chunk (0,0,0) is centered at world origin,
 * with boundaries positioned away from common spawn points to prevent thrashing
 * from localization noise.
 * 
 * Key properties:
 * - Robot at world origin (0,0,0) is at CENTER of chunk (0,0,0)
 * - Distance to nearest boundary: chunk_size/2 (e.g., 5m for 10m chunks)
 * - Half-open intervals [min, max) for consistent boundary behavior
 * - Round-half-away-from-zero for symmetric coordinate mapping
 */
class ChunkCoordinateSystem
{
    using Vector3D = Eigen::Vector3d;
public:

    /**
     * @brief Classification of neighbor relationship between chunks
     * 
     * Based on number of axes that differ by ±1:
     * - SOURCE: Same chunk (0 axes differ)
     * - FACE: Adjacent along one axis (1 axis differs)
     * - EDGE: Adjacent along two axes (2 axes differ)
     * - CORNER: Adjacent along three axes (3 axes differ)
     * - INDETERMINATE: Non-adjacent or invalid
     */
    enum class NeighbourType : uint8_t {
        SOURCE = 0,        /// Same chunk (delta = 0,0,0)
        FACE = 1,          /// Face neighbor (6 total, e.g., ±1,0,0)
        EDGE = 2,          /// Edge neighbor (12 total, e.g., ±1,±1,0)
        CORNER = 3,        /// Corner neighbor (8 total, e.g., ±1,±1,±1)
        INDETERMINATE = 4  /// Not an immediate neighbor (|delta| > 1)
    };

    /**
     * @brief Enumeration of spatial axes
     * 
     * Used for per-axis boundary queries and distance calculations.
     */
    enum class AxisType : uint8_t {
        X = 0,             /// X-axis (index 0)
        Y = 1,             /// Y-axis (index 1)
        Z = 2,             /// Z-axis (index 2)
        INDETERMINATE = 4  /// Invalid/unspecified axis
    };

    /**
     * @brief Convert NeighbourType enum to string representation
     * @param type Neighbor type to convert
     * @return String name of the type
     */
    static inline std::string reflectNeighbourType(const NeighbourType& type) {
        // build a mapping
        std::array<std::string,5> mapping = {"SOURCE","FACE","EDGE","CORNER","INDETERMINATE"};

        return mapping[static_cast<uint8_t>(type)];
    }

    /**
     * @brief Convert AxisType enum to string representation
     * @param type Axis type to convert
     * @return String name of the axis
     */
    static inline std::string reflectAxisType(const AxisType& type) {
        // create the mapping
        std::array<std::string,4> mapping = {"X","Y","Z","INDETERMINATE"};
        
        return mapping[static_cast<uint8_t>(type)];
    }

    /**
     * @brief Construct coordinate system with specified chunk size
     * @param chunk_size Size of each chunk cube in meters (must be > 0)
     * @throws std::invalid_argument if chunk_size <= 0 (implementation-dependent)
     */
    explicit ChunkCoordinateSystem(double voxel_resolution,double chunk_size);

    /**
     * @brief Convert arbitrary position type to chunk coordinate
     * 
     * Template overload accepting any type convertible via Bonxai::ConvertPoint
     * (e.g., pcl::PointXYZ, Eigen::Vector3d, custom types with x,y,z fields).
     * 
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] ChunkCoord positionToChunkCoordinate(const PositionCoordinateT& position_coordinate) const {
        // Convert it to a point we can all agree upon
        Vector3D point_to_use = Bonxai::ConvertPoint<Vector3D>(position_coordinate);
        return positionToChunkCoordinate(point_to_use.x(),point_to_use.y(),point_to_use.z());
    }

    /**
     * @brief Convert world position to chunk coordinate
     * 
     * Uses center-origin convention with round-half-away-from-zero for symmetry.
     * Position (0,0,0) maps to chunk (0,0,0).
     * 
     * @param x World X coordinate (meters)
     * @param y World Y coordinate (meters)
     * @param z World Z coordinate (meters)
     * @return Chunk coordinate containing the position
     * 
     */
    [[nodiscard]] ChunkCoord positionToChunkCoordinate(double x, double y, double z) const;

    /**
     * @brief Convert arbitrary position type to chunk coordinate
     * 
     * Template overload accepting any type convertible via Bonxai::ConvertPoint
     * (e.g., pcl::PointXYZ, Eigen::Vector3d, custom types with x,y,z fields).
     * 
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] Bonxai::CoordT positionToVoxelCoordinate(const PositionCoordinateT& position_coordinate) const {
        // Convert it to a point we can all agree upon
        Bonxai::Point3D point_to_use = Bonxai::ConvertPoint<Bonxai::Point3D>(position_coordinate);
        return positionToVoxelCoordinate(point_to_use);
    }

    /**
     * @brief Convert world position to voxel coordinate
     * 
     * Uses center-origin convention with round-half-away-from-zero for symmetry.
     * Position (0,0,0) maps to chunk (0,0,0).
     * 
     * @param x World X coordinate (meters)
     * @param y World Y coordinate (meters)
     * @param z World Z coordinate (meters)
     * @return Voxel containing the position
     * 
     */
    [[nodiscard]] Bonxai::CoordT positionToVoxelCoordinate(const Bonxai::Point3D& bonxai_3d_pt) const;

    /**
     * @brief Convert world position to voxel coordinate
     * 
     * Uses center-origin convention with round-half-away-from-zero for symmetry.
     * Position (0,0,0) maps to chunk (0,0,0).
     * 
     * @param x World X coordinate (meters)
     * @param y World Y coordinate (meters)
     * @param z World Z coordinate (meters)
     * @return Voxel containing the position
     * 
     */
    [[nodiscard]] Bonxai::CoordT positionToVoxelCoordinate(double x, double y, double z) const;

    /**
     * @brief Convert chunk coordinate to world position (center)
     * 
     * Returns the center position of the specified chunk in world coordinates.
     * 
     * @param chunk_coord Chunk coordinate
     * @return 3D position at chunk center (meters)
     */
    [[nodiscard]] Vector3D chunkToPositionCoordinate(const ChunkCoord& chunk_coord) const;

    /**
     * @brief Convert chunk coordinate to world position (center)
     * 
     * Returns the center position of the specified chunk in world coordinates.
     * 
     * @param chunk_coord Chunk coordinate
     * @return 3D position at chunk center (meters)
     */
    [[nodiscard]] Vector3D voxelToPositionCoordinate(const Bonxai::CoordT& voxel_coord) const;

    /**
     * @brief Get minimum boundary (corner) of chunk as 3D vector
     * 
     * Returns the lower-left-back corner of the chunk's spatial extent.
     * 
     * @param chunk_coord Chunk coordinate
     * @return Minimum corner position (meters)
     * @see chunkMaxBounds() for opposite corner
     */
    [[nodiscard]] Vector3D chunkMinBounds(const ChunkCoord& chunk_coord) const;

    /**
     * @brief Get minimum boundary along specific axis
     * 
     * @param chunk_coord Chunk coordinate
     * @param axis Axis to query (X, Y, or Z)
     * @return Minimum coordinate value along specified axis (meters)
     * 
     * @warning If axis is INDETERMINATE, returns X-axis value as default
     */
    [[nodiscard]] double chunkMinBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const;

    /**
     * @brief Convert chunk world-space bounds to voxel-space integer bounds
     * 
     * Used for the ray-walk boundary-check optimization. Returns the
     * min (inclusive) and max (exclusive, matching half-open convention)
     * voxel coordinates for a given chunk.
     * 
     * @param chunk Chunk coordinate
     * @return Pair of {voxel_min, voxel_max} as CoordT
     */
    [[nodiscard]] std::pair<Bonxai::CoordT, Bonxai::CoordT> worldBoundsToVoxelBounds(const ChunkCoord& chunk) const;

    /**
     * @brief Get maximum boundary (corner) of chunk as 3D vector
     * 
     * Returns the upper-right-front corner of the chunk's spatial extent.
     * Note: Uses half-open interval, so max boundary is exclusive.
     * 
     * @param chunk_coord Chunk coordinate
     * @return Maximum corner position (meters)
     * 
     * @note For chunk centered at (cx,cy,cz): max = center + (size/2, size/2, size/2)
     * @note Boundary convention: [min, max) - max is exclusive
     * @see chunkMinBounds() for opposite corner
     */
    [[nodiscard]] Vector3D chunkMaxBounds(const ChunkCoord& chunk_coord) const;

    /**
     * @brief Get maximum boundary along specific axis
     * 
     * @param chunk_coord Chunk coordinate
     * @param axis Axis to query (X, Y, or Z)
     * @return Maximum coordinate value along specified axis (meters)
     * 
     * @note If axis is INDETERMINATE, returns X-axis value as default
     * @note Boundary convention: [min, max) - max is exclusive
     */
    [[nodiscard]] double chunkMaxBounds(const ChunkCoord& chunk_coord, const AxisType &axis) const;

    /**
     * @brief Get configured chunk size
     * @return Chunk size in meters
     */
    [[nodiscard]] double getChunkSize() const;

    /**
     * @brief Check if position is inside chunk (template version)
     * 
     * @tparam PositionCoordinateT Type with x,y,z fields or conversion support
     * @param position Position to test
     * @param chunk_coord Chunk to test against
     * @return true if position is within chunk boundaries
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] bool isPositionInChunk(const PositionCoordinateT& position, const ChunkCoord& chunk_coord) const {
        Vector3D point_to_use = Bonxai::ConvertPoint<Vector3D>(position);
        return isPositionInChunk(point_to_use,chunk_coord);
    }

    /**
     * @brief Check if 3D position is inside chunk
     * 
     * Uses half-open interval [min, max) for boundaries. Position exactly on
     * max boundary belongs to next chunk.
     * 
     * @param position Position to test (meters)
     * @param chunk_coord Chunk to test against
     * @return true if position is within chunk boundaries
     */
    [[nodiscard]] bool isPositionInChunk(const Vector3D& position, const ChunkCoord& chunk_coord) const;

    /**
     * @brief Calculate distance to chunk boundary (template version)
     * 
     * @tparam PositionCoordinateT Type with x,y,z fields or conversion support
     * @param position Position to measure from
     * @param chunk_coord Chunk to measure distance to
     * @param optional_axis If specified, measure distance along this axis only
     * @return Distance to boundary in meters, or -1.0 if position not in chunk
     */
    template <typename PositionCoordinateT>
    [[nodiscard]] double distanceToBoundary(const PositionCoordinateT& position, 
                                            const ChunkCoord& chunk_coord,
                                            std::optional<AxisType> optional_axis = std::nullopt) const {
        Vector3D pos_to_use = Bonxai::ConvertPoint<Vector3D>(position);
        return distanceToBoundary(pos_to_use,chunk_coord,optional_axis);
    }

    /**
     * @brief Calculate perpendicular distance from position to nearest chunk boundary
     * 
     * Computes minimum distance to any face boundary (or specific axis if specified).
     * Returns negative sentinel if position is outside chunk.
     * 
     * @param position Position to measure from (meters)
     * @param chunk_coord Chunk to measure distance within
     * @param optional_axis If specified, measure distance along this axis only
     * @return Distance to nearest boundary (meters), or -1.0 if:
     *         - Position not in chunk
     *         - optional_axis is INDETERMINATE
     */
    [[nodiscard]] double distanceToBoundary(const Vector3D& position, 
                                            const ChunkCoord& chunk_coord,
                                            std::optional<AxisType> optional_axis = std::nullopt) const;

    /**
     * @brief Get all 6 face-adjacent neighbors of a chunk
     * 
     * Returns neighbors that differ by ±1 on exactly one axis.
     * 
     * @param chunk_coord Center chunk
     * @return Vector of 6 face neighbors (±x, ±y, ±z)
     */
    [[nodiscard]] static std::vector<ChunkCoord> getFaceNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief Get all 12 edge-adjacent neighbors of a chunk
     * 
     * Returns neighbors that differ by ±1 on exactly two axes.
     * 
     * @param chunk_coord Center chunk
     * @return Vector of 12 edge neighbors
     * 
     * @note Grouped by plane: 4 xy-plane, 4 xz-plane, 4 yz-plane
     */
    [[nodiscard]] static std::vector<ChunkCoord> getEdgeNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief Get all 8 corner-adjacent neighbors of a chunk
     * 
     * Returns neighbors that differ by ±1 on all three axes.
     * 
     * @param chunk_coord Center chunk
     * @return Vector of 8 corner neighbors (±x, ±y, ±z combinations)
     */
    [[nodiscard]] static std::vector<ChunkCoord> getCornerNeighbours(const ChunkCoord& chunk_coord);
    
    /**
     * @brief Get all 26 neighbors (face + edge + corner) of a chunk
     * 
     * Efficiently combines all neighbor types using forEachNeighbour internally
     * to minimize allocations.
     * 
     * @param chunk_coord Center chunk
     * @return Vector of all 26 neighbors (3³ - 1)
     */
    [[nodiscard]] static std::vector<ChunkCoord> getAllNeighbours(const ChunkCoord& chunk_coord);

    /**
     * @brief Classify neighbor relationship based on coordinate delta
     * 
     * Determines neighbor type by counting axes that differ by ±1.
     * 
     * @param delta Difference between two chunk coordinates (target - source)
     * @return Neighbor type classification
     */
    [[nodiscard]] static NeighbourType getNeighbourType(const ChunkCoord& delta);

    /**
     * @brief Iterate over all neighbors within radius without allocation
     * 
     * Calls the provided functor for each neighbor within Chebyshev distance
     * (max absolute coordinate difference) <= radius. The center chunk itself
     * is NOT visited.
     * 
     * @tparam Func Functor type with signature: void(const ChunkCoord&)
     * @param center Center chunk coordinate
     * @param radius Maximum Chebyshev distance (in chunks)
     * @param func Functor called for each neighbor
     * 
     * @note Neighbor count: (2×radius+1)³ - 1
     *       - radius=1: 26 neighbors (3³-1)
     *       - radius=2: 124 neighbors (5³-1)
     *       - radius=3: 342 neighbors (7³-1)
     */
    template <typename Func>
    static void forEachNeighbour(const ChunkCoord& center, int radius, Func&& func) {
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    ChunkCoord nb{center.x + dx, center.y + dy, center.z + dz};
                    func(nb); 
                }
            }
        }
    }

    /**
     * @brief Round floating-point value to nearest integer
     * 
     * Implements round-half-away-from-zero for symmetric behavior:
     * - 0.5 → 1
     * - -0.5 → -1
     * 
     * @param val Value to round
     * @return Nearest integer
     * 
     * @note This differs from std::round which uses round-half-to-even
     * @note Used internally by positionToChunkCoordinate for symmetry
     * @complexity O(1), ~1ns
     */
    [[nodiscard]] static int32_t roundToInt(double val);

private:
    double voxel_resolution_;
    double voxel_inv_resolution_;
    double chunk_size_;       ///< Size of each chunk cube (meters)
    double chunk_size_half_;  ///< Cached half chunk size (meters)
    Vector3D half_chunk_vec;  ///< Cached half-size vector for boundary calculations

}; // class ChunkCoordinateSystem

} // namespace RollingBonxai