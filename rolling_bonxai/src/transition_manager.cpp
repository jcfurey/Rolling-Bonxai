#include "rolling_bonxai/transition_manager.hpp"

#include <sstream>
#include <utility>

namespace RollingBonxai
{

void TransitionManager::reportSkip(uint64_t lifetime_count,
                                   uint64_t run_length,
                                   const std::string& what)
{
    // Rate-limit on the LIFETIME count, not the current run. A pose stream that
    // alternates valid/invalid every cycle would otherwise restart the run at 1
    // each time and log on every single one. This way the budget is at most one
    // line per kSkipLogStride skips for this cause, whatever the pattern.
    if ((lifetime_count % kSkipLogStride) != 1) {
        return;
    }
    warned_current_run_ = true;

    if (!logger_) {
        return;
    }

    std::ostringstream oss;
    oss << "[RollingBonxai]: Rolling update skipped — " << what
        << ". The rolling window is frozen at chunk ("
        << reference_chunk_coord_.x << "," << reference_chunk_coord_.y << ","
        << reference_chunk_coord_.z << "); no chunks will load or evict until "
           "valid poses resume. Consecutive skipped cycles: " << run_length
        << ", total for this cause: " << lifetime_count << ".";
    logger_->log_warn(oss.str());
}

void TransitionManager::reportRecoveryIfNeeded()
{
    const uint64_t run = nonfinite_pose_skips_ + degenerate_state_skips_;
    if (run == 0) {
        return;
    }

    // Only pair a recovery line with a run that actually announced itself,
    // otherwise the rate limit above could be undone by recovery spam.
    if (warned_current_run_ && logger_) {
        std::ostringstream oss;
        oss << "[RollingBonxai]: Rolling update recovered after " << run
            << " skipped cycle(s) (" << nonfinite_pose_skips_
            << " non-finite pose, " << degenerate_state_skips_
            << " degenerate state).";
        logger_->log_warn(oss.str());
    }

    nonfinite_pose_skips_ = 0;
    degenerate_state_skips_ = 0;
    warned_current_run_ = false;
}

uint64_t TransitionManager::getNonFinitePoseSkipCount() const
{
    return total_nonfinite_pose_skips_;
}

uint64_t TransitionManager::getDegenerateStateSkipCount() const
{
    return total_degenerate_state_skips_;
}

TransitionManager::TransitionManager(double hyst_ratio,
                                     ChunkCoordinateSystem& csys,
                                     Logger::SharedPtr logger)
    : hyst_ratio_(hyst_ratio)
    , csys_(csys)
    , logger_(std::move(logger))
{
    // Clamp hysteresis ratio to valid range [0.01, 1.0]
    // Values outside this range are either too sensitive or too large to be useful
    hyst_ratio_ = std::clamp(hyst_ratio, 0.01, 1.0);
    
    // Preload ratio for V3 predictive loading (not used in V2)
    // Set to 50% larger than hysteresis to trigger preloading before hysteresis zone
    preload_ratio_ = 1.5 * hyst_ratio_;
}

bool TransitionManager::shouldTriggerTransition(const TransitionManagerContext& context)
{
    // === REJECT NON-FINITE POSES ===
    // A NaN or infinity here makes every comparison in isPositionInChunk()
    // false, so the chunk the position was just classified into reports the
    // position as outside itself, which used to end in the UNINITIALIZED throw
    // below.
    //
    // Note on where this comes from: tf2 screens NaN translations at ingestion
    // (TF_NAN_INPUT), so a NaN cannot arrive through a TF lookup. It does NOT
    // screen infinities — an infinite translation passes straight through to
    // callers. Direct users of this library bypass tf2 entirely. Hence the
    // check covers both rather than just one.
    //
    // Hold the current state and skip the cycle; the next valid pose resumes
    // normally. Skipping is reported, because a frozen rolling window otherwise
    // looks identical to a stationary robot.
    if (!context.ego_position.allFinite()) {
        ++nonfinite_pose_skips_;
        ++total_nonfinite_pose_skips_;
        reportSkip(total_nonfinite_pose_skips_, nonfinite_pose_skips_,
                   "ego position is not finite (NaN or infinity)");
        return false;
    }

    reportRecoveryIfNeeded();

    // === INITIALIZATION (First Update) ===
    if (!initialized_) {
        initialized_ = true;
        updateEgoState(context);

        // Determine starting chunk from robot's initial position
        ChunkCoord initial_chunk = csys_.positionToChunkCoordinate(ego_position_);
        
        // Set this as both reference and previous
        setRefChunkCoord(initial_chunk);
        setPrevChunkCoord(initial_chunk);

        // Compute initial state (typically STABLE_SOURCE if spawned at chunk center)
        TransitionState initial_state = computeStateIfSameChunk(context, initial_chunk);

        // Initialize state machine
        setRefTransitionState(initial_state);
        setPrevTransitionState(TransitionState::UNINITIALIZED);

        // Return true to trigger initial chunk loading
        return true;
    }

    // === UPDATE EGO STATE ===
    updateEgoState(context);

    // Get current geometric chunk coordinate (where robot actually is)
    ChunkCoord current_chunk_coord = csys_.positionToChunkCoordinate(ego_position_);

    // === CHUNK JUMP DETECTION ===
    // If robot jumped multiple chunks (teleportation), bypass hysteresis
    if (detectChunkJump(context)) {
        // Update reference immediately and trigger I/O
        setPrevChunkCoord(reference_chunk_coord_);
        setRefChunkCoord(current_chunk_coord);
        
        setPrevTransitionState(reference_t_state_);
        setRefTransitionState(TransitionState::STABLE_SOURCE);
        
        return true;
    }

    // === STATE COMPUTATION ===
    // Determine if robot is in same chunk as reference or has moved to neighbor
    bool in_same_chunk = (current_chunk_coord == reference_chunk_coord_);

    // Compute new state based on position
    TransitionState new_state = in_same_chunk 
        ? computeStateIfSameChunk(context, current_chunk_coord)
        : computeStateIfNextChunk(context, current_chunk_coord);

    // Sanity check: should never go back to UNINITIALIZED.
    //
    // This is reached when the coordinate system reports the ego as outside the
    // very chunk it just classified the ego into. That is a geometric
    // contradiction, but a reachable one: a large finite coordinate (1e300, say,
    // from a diverging state estimator) is finite so it clears the check above
    // and passes tf2's own screening, yet positionToChunkCoordinate() cannot
    // represent the resulting chunk index in an int32, so the chunk it returns
    // does not contain the position. Verified reachable end to end.
    //
    // It used to throw, and since this runs inside the point cloud subscription
    // callback the exception escaped the executor and took the whole node down.
    // A mapping node should not die over one bad pose: hold the current state,
    // trigger no I/O, and let the next update sort itself out — but say so, so
    // a persistent inconsistency does not degrade silently.
    if (new_state == TransitionState::UNINITIALIZED) {
        ++degenerate_state_skips_;
        ++total_degenerate_state_skips_;
        std::ostringstream cause;
        cause << "coordinate system places ego (" << ego_position_.x() << ","
              << ego_position_.y() << "," << ego_position_.z()
              << ") outside chunk (" << current_chunk_coord.x << ","
              << current_chunk_coord.y << "," << current_chunk_coord.z
              << ") that it was just classified into";
        reportSkip(total_degenerate_state_skips_, degenerate_state_skips_, cause.str());
        return false;
    }

    // === SAME CHUNK CASE ===
    if (in_same_chunk) {
        // Robot still in reference chunk, just update state
        // No transition possible, no I/O needed
        setPrevTransitionState(reference_t_state_);
        setRefTransitionState(new_state);
        
        return false;
    }

    // === DIFFERENT CHUNK CASE (Robot in neighboring chunk) ===
    // Update state tracking
    setPrevTransitionState(reference_t_state_);
    setRefTransitionState(new_state);

    // Check if transition should be confirmed
    if (reference_t_state_ == TransitionState::STABLE_NEXT) {
        // Robot has penetrated far enough into next chunk
        // Confirm the transition and trigger I/O
        
        // Update chunk tracking
        setPrevChunkCoord(reference_chunk_coord_);
        setRefChunkCoord(current_chunk_coord);

        // Reset state to STABLE_SOURCE in the new chunk
        setPrevTransitionState(reference_t_state_);
        setRefTransitionState(TransitionState::STABLE_SOURCE);

        // Trigger I/O: load new neighborhood, evict old chunks
        return true;
    }

    // Still in HYSTERESIS_NEXT state - waiting for confirmation
    // No I/O triggered yet
    return false;
}

double TransitionManager::calculateBoundaryPenetration(
    const TransitionManagerContext& context,
    const ChunkCoord& reference_chunk,
    const ChunkCoord& next_chunk) const
{
    const double chunk_size = csys_.getChunkSize();
    const double half_chunk = chunk_size / 2.0;

    // Get center positions of both chunks
    Position3D source_center = csys_.chunkToPositionCoordinate(reference_chunk);

    // Determine which axes changed between chunks
    ChunkCoord diff = next_chunk - reference_chunk;

    // Track minimum penetration across all changing axes
    // For edge/corner transitions, robot must exceed threshold on ALL axes
    double min_penetration = std::numeric_limits<double>::max();

    // Reference to robot position for clarity
    const auto& robot_pos = context.ego_position;

    // Check each axis that changed
    for (int axis = 0; axis < 3; ++axis) {
        int delta = 0;
        double robot_coord = 0.0;
        double source_coord = 0.0;

        // Extract axis-specific values
        switch (axis) {
            case 0:  // X axis
                delta = diff.x;
                robot_coord = robot_pos.x();
                source_coord = source_center.x();
                break;
            case 1:  // Y axis
                delta = diff.y;
                robot_coord = robot_pos.y();
                source_coord = source_center.y();
                break;
            case 2:  // Z axis
                delta = diff.z;
                robot_coord = robot_pos.z();
                source_coord = source_center.z();
                break;
        }

        // Only calculate penetration for axes that changed
        if (delta != 0) {
            // Calculate position of shared boundary on this axis
            // delta is +1 or -1 indicating direction of transition
            double boundary_pos = source_coord + (delta * half_chunk);

            // Penetration is absolute distance from robot to boundary
            // This works for both positive and negative transitions
            double penetration = std::abs(robot_coord - boundary_pos);

            // For multi-axis transitions (edge/corner), take minimum
            // Robot must exceed threshold on ALL changing axes
            min_penetration = std::min(min_penetration, penetration);
        }
    }

    return min_penetration;
}

TransitionState TransitionManager::computeStateIfSameChunk(
    const TransitionManagerContext& context,
    const ChunkCoord& chunk_coord)
{
    const double chunk_size = csys_.getChunkSize();
    const double hysteresis_zone = chunk_size * hyst_ratio_;

    // Calculate minimum distance to any boundary of current chunk
    double dist_to_boundary = csys_.distanceToBoundary(context.ego_position, chunk_coord);

    // Determine state based on proximity to boundary
    if (dist_to_boundary >= hysteresis_zone) {
        // Far from any boundary - stable operation
        return TransitionState::STABLE_SOURCE;
    }
    else if (dist_to_boundary >= 0 && dist_to_boundary < hysteresis_zone){
        // Close to boundary but still inside chunk
        return TransitionState::HYSTERESIS_SOURCE;
    }
    else {
        // Should never happen (negative distance means outside chunk)
        // This is a logic error in the calling code
        return TransitionState::UNINITIALIZED;
    }
}

TransitionState TransitionManager::computeStateIfNextChunk(
    const TransitionManagerContext& context,
    const ChunkCoord& chunk_coord)
{
    const double chunk_size = csys_.getChunkSize();
    const double hysteresis_zone = chunk_size * hyst_ratio_;

    // Calculate penetration depth into next chunk from shared boundary
    double penetration = calculateBoundaryPenetration(
        context,
        reference_chunk_coord_,  // Source chunk (where we came from)
        chunk_coord              // Next chunk (where we are geometrically)
    );

    // Determine state based on penetration depth
    if (penetration < hysteresis_zone) {
        // Not deep enough into next chunk yet - wait for confirmation
        return TransitionState::HYSTERESIS_NEXT;
    }
    else {
        // Penetrated far enough - confirm transition
        return TransitionState::STABLE_NEXT;
    }
}

void TransitionManager::updateEgoState(const TransitionManagerContext& context)
{
    ego_position_ = context.ego_position;
    ego_velocity_ = context.ego_velocity;
}

void TransitionManager::setRefChunkCoord(const ChunkCoord& coord)
{
    reference_chunk_coord_ = coord;
}

void TransitionManager::setPrevChunkCoord(const ChunkCoord& coord)
{
    previous_chunk_coord_ = coord;
}

void TransitionManager::setRefTransitionState(const TransitionState& state)
{
    reference_t_state_ = state;
}

void TransitionManager::setPrevTransitionState(const TransitionState& state)
{
    previous_t_state_ = state;
}

bool TransitionManager::detectChunkJump(const TransitionManagerContext& context) const
{
    // Compute chunk coordinate from current position
    ChunkCoord coord = csys_.positionToChunkCoordinate(context.ego_position);

    // Calculate Chebyshev distance (max absolute difference across all axes)
    ChunkCoord diff = coord - reference_chunk_coord_;
    uint32_t max_diff = std::max({
        static_cast<uint32_t>(std::abs(diff.x)),
        static_cast<uint32_t>(std::abs(diff.y)),
        static_cast<uint32_t>(std::abs(diff.z))
    });

    // Consider it a jump if moved more than 1 chunk in any direction
    // This handles teleportation, respawns, and large odometry corrections
    return max_diff > 1;
}

TransitionState TransitionManager::getRefTransitionState() const
{
    return reference_t_state_;
}

TransitionState TransitionManager::getPrevTransitionState() const
{
    return previous_t_state_;
}

ChunkCoord TransitionManager::getRefChunkCoord() const
{
    return reference_chunk_coord_;
}

ChunkCoord TransitionManager::getPrevChunkCoord() const
{
    return previous_chunk_coord_;
}

double TransitionManager::getHysteresisRatio() const
{
    return hyst_ratio_;
}

} // namespace RollingBonxai