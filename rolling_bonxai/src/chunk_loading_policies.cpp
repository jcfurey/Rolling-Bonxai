#include "rolling_bonxai/chunk_loading_policies.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace RollingBonxai
{

// ============================================================================
// NeighborhoodPolicy Implementation
// ============================================================================

NeighborhoodPolicy::NeighborhoodPolicy(int radius)
    : radius_(radius)
{
    if (radius < 0) {
        throw std::invalid_argument("NeighborhoodPolicy: radius must be non-negative");
    }
}

bool NeighborhoodPolicy::shouldLoad(const ChunkCoord& coord,
                                    const PolicyContext& context) const
{
    const int dx = std::abs(coord.x - context.current_chunk.x);
    const int dy = std::abs(coord.y - context.current_chunk.y);
    const int dz = std::abs(coord.z - context.current_chunk.z);
    
    // Chebyshev distance
    const int dist = std::max({dx, dy, dz});
    
    return dist <= radius_;
}

bool NeighborhoodPolicy::shouldEvict(const ChunkCoord& coord,
                                     const PolicyContext& context) const
{
    // Evict anything not in the load set
    return !shouldLoad(coord, context);
}

double NeighborhoodPolicy::getPriority(const ChunkCoord& coord,
                                       const PolicyContext& context) const
{
    const int dx = std::abs(coord.x - context.current_chunk.x);
    const int dy = std::abs(coord.y - context.current_chunk.y);
    const int dz = std::abs(coord.z - context.current_chunk.z);
    
    const int dist = std::max({dx, dy, dz});
    
    // Closer chunks get higher priority
    // Formula: 1.0 / (1.0 + distance)
    // dist=0: priority=1.0, dist=1: priority=0.5, dist=2: priority=0.33, etc.
    return 1.0 / (1.0 + static_cast<double>(dist));
}

std::string NeighborhoodPolicy::getName() const
{
    std::ostringstream oss;
    oss << "NeighborhoodPolicy(radius=" << radius_ << ")";
    return oss.str();
}

std::string NeighborhoodPolicy::getDescription() const
{
    std::ostringstream oss;
    oss << "Loads all chunks within Chebyshev distance " << radius_
        << " from current chunk. ";
    
    const int total_chunks = std::pow(2 * radius_ + 1, 3) - 1;
    oss << "Typical load: " << total_chunks << " chunks (3D neighborhood).";
    
    return oss.str();
}

int NeighborhoodPolicy::getRadius() const
{
    return radius_;
}

int NeighborhoodPolicy::getSearchRadius() const
{
    return radius_;
}

// ============================================================================
// PlanarNeighborhoodPolicy Implementation
// ============================================================================

PlanarNeighborhoodPolicy::PlanarNeighborhoodPolicy(int radius, int z_min, int z_max, bool use_relative)
    : radius_(radius),
      z_min_(z_min),
      z_max_(z_max),
      use_relative_(use_relative)
{
    if (radius < 0) {
        throw std::invalid_argument("PlanarNeighborhoodPolicy: radius must be non-negative");
    }

    if (!use_relative && z_min > z_max) {
        throw std::invalid_argument("PlanarNeighborhoodPolicy: z_min must be <= z_max");
    }

    // Under the relative scheme, the idea of z_max and z_min are relative quantities strictly above
    // and below the ego agent. So, no negative signs

    if (use_relative && (z_min < 0 || z_max < 0)) {
        throw std::invalid_argument("PlanarNeighborhoodPolicy: Both z_min and z_max must be positive values. Their relative magnitudes dont matter");
    }
}

bool PlanarNeighborhoodPolicy::shouldLoad(const ChunkCoord& coord,
                                          const PolicyContext& context) const
{
    // Check horizontal neighborhood
    const int abs_dx = std::abs(coord.x - context.current_chunk.x);
    const int abs_dy = std::abs(coord.y - context.current_chunk.y);
    const int abs_dz = std::abs(coord.z - context.current_chunk.z);
    const int signed_dz = coord.z - context.current_chunk.z;

    if (use_relative_) {
        // if signed_dz > 0, this means that the coord in question is above us
        if (signed_dz > 0 && abs_dz > z_max_) {
            return false;
        }
        // if signed_dz < 0, this means that the coord in question in below us
        if (signed_dz < 0 && abs_dz > z_min_) {
            return false;
        }
    }

    else {
        if (coord.z < z_min_ || coord.z > z_max_) {
            return false;
        }
    }

    const int dist = std::max({abs_dx, abs_dy, abs_dz});
    
    return dist <= radius_;
}

bool PlanarNeighborhoodPolicy::shouldEvict(const ChunkCoord& coord,
                                           const PolicyContext& context) const
{
    return !shouldLoad(coord, context);
}

double PlanarNeighborhoodPolicy::getPriority(const ChunkCoord& coord,
                                             const PolicyContext& context) const
{
    if (!shouldLoad(coord, context)) {
        return 0.0;
    }
    
    const int dx = std::abs(coord.x - context.current_chunk.x);
    const int dy = std::abs(coord.y - context.current_chunk.y);
    const int dz = std::abs(coord.z - context.current_chunk.z);
    
    const int dist = std::max({dx, dy, dz});
    
    return 1.0 / (1.0 + static_cast<double>(dist));
}

std::string PlanarNeighborhoodPolicy::getName() const
{
    std::ostringstream oss;
    oss << "PlanarNeighborhoodPolicy(radius=" << radius_
        << ", z=[" << z_min_ << "," << z_max_ << "])";
    return oss.str();
}

std::string PlanarNeighborhoodPolicy::getDescription() const
{
    std::ostringstream oss;
    oss << "Loads chunks within horizontal radius " << radius_
        << " and vertical range [" << z_min_ << "," << z_max_ << "]. ";
    
    const int z_layers = z_max_ - z_min_ + 1;
    const int xy_chunks_per_layer = std::pow(2 * radius_ + 1, 2);
    const int total_chunks = xy_chunks_per_layer * z_layers - 1;  // -1 for current chunk
    
    oss << "Typical load: " << total_chunks << " chunks ("
        << z_layers << " vertical layers).";
    
    return oss.str();
}

int PlanarNeighborhoodPolicy::getRadius() const
{
    return radius_;
}

int PlanarNeighborhoodPolicy::getSearchRadius() const
{
    // Relative mode measures z_min/z_max as offsets below/above the ego, so both
    // are vertical extents that the enumeration cube has to cover. In absolute
    // mode they are world chunk coordinates instead, and the vertical reach is
    // then bounded by the horizontal radius (shouldLoad() still applies the
    // Chebyshev test), so only the relative case can widen the cube.
    if (use_relative_) {
        return std::max({radius_, z_min_, z_max_});
    }
    return radius_;
}

int PlanarNeighborhoodPolicy::getZMin() const
{
    return z_min_;
}

int PlanarNeighborhoodPolicy::getZMax() const
{
    return z_max_;
}

bool PlanarNeighborhoodPolicy::isRelative() const {
    return use_relative_;
}

// ============================================================================
// TemporalNeighborhoodPolicy Implementation
// ============================================================================

TemporalNeighborhoodPolicy::TemporalNeighborhoodPolicy(int radius, double age_weightage, std::chrono::seconds max_age)
    : radius_(radius),
      max_age_(max_age),
      age_weightage_(age_weightage)
{
    if (radius < 0) {
        throw std::invalid_argument("TemporalNeighborhoodPolicy: radius must be non-negative");
    }

    if (age_weightage < 0 || age_weightage > 1) {
        throw std::invalid_argument("TemporalNeighborhoodPolicy: age weightage must be E [0,1]");
    }
}

bool TemporalNeighborhoodPolicy::shouldLoad(const ChunkCoord& coord,
                                            const PolicyContext& context) const
{
    // Check neighborhood first
    const int dx = std::abs(coord.x - context.current_chunk.x);
    const int dy = std::abs(coord.y - context.current_chunk.y);
    const int dz = std::abs(coord.z - context.current_chunk.z);
    
    const int dist = std::max({dx, dy, dz});
    
    if (dist > radius_) {
        return false;  // Outside neighborhood
    }
    
    // Check age (if chunk exists)
    const ChunkMetadata* meta = context.getMetadata(coord);
    if (meta == nullptr) {
        // New chunk (no metadata yet), always load
        return true;
    }
    
    // Calculate age
    const auto age = std::chrono::duration_cast<std::chrono::seconds>(
        context.current_time - 
        std::chrono::steady_clock::time_point(std::chrono::nanoseconds(meta->creation_time))
    );
    
    return age < max_age_;
}

bool TemporalNeighborhoodPolicy::shouldEvict(const ChunkCoord& coord,
                                             const PolicyContext& context) const
{
    return !shouldLoad(coord, context);
}

double TemporalNeighborhoodPolicy::getPriority(const ChunkCoord& coord,
                                               const PolicyContext& context) const
{
    if (!shouldLoad(coord, context)) {
        return 0.0;
    }
    
    // Base priority from distance
    const int dx = std::abs(coord.x - context.current_chunk.x);
    const int dy = std::abs(coord.y - context.current_chunk.y);
    const int dz = std::abs(coord.z - context.current_chunk.z);
    
    const int dist = std::max({dx, dy, dz});
    const double dist_priority = 1.0 / (1.0 + static_cast<double>(dist));
    
    // Age bonus (newer = higher priority)
    const ChunkMetadata* meta = context.getMetadata(coord);
    if (meta == nullptr) {
        // New chunk, give it full priority
        return dist_priority;
    }
    
    const auto age = std::chrono::duration_cast<std::chrono::seconds>(
        context.current_time - 
        std::chrono::steady_clock::time_point(std::chrono::nanoseconds(meta->creation_time))
    );
    
    const double age_seconds = static_cast<double>(age.count());
    const double max_age_seconds = static_cast<double>(max_age_.count());
    
    // Freshness factor: 1.0 for brand new, 0.0 for max_age
    double freshness = 1.0 - (age_seconds / max_age_seconds);
    freshness = std::max(0.0, freshness);  // Clamp to [0, 1]
    
    return (1.0 - age_weightage_) * dist_priority + age_weightage_ * freshness;
}

std::string TemporalNeighborhoodPolicy::getName() const
{
    std::ostringstream oss;
    oss << "TemporalNeighborhoodPolicy(radius=" << radius_
        << ", max_age=" << max_age_.count() << "s)";
    return oss.str();
}

std::string TemporalNeighborhoodPolicy::getDescription() const
{
    std::ostringstream oss;
    oss << "Loads chunks within radius " << radius_
        << " that are newer than " << max_age_.count() << " seconds. ";
    oss << "Filters out stale data in dynamic environments. "
        << "New chunks (no metadata) are always loaded.";
    
    return oss.str();
}

int TemporalNeighborhoodPolicy::getRadius() const
{
    return radius_;
}

int TemporalNeighborhoodPolicy::getSearchRadius() const
{
    return radius_;
}

std::chrono::seconds TemporalNeighborhoodPolicy::getMaxAge() const
{
    return max_age_;
}

double TemporalNeighborhoodPolicy::getAgeWeightage() const {
    return age_weightage_;
}

} // namespace RollingBonxai