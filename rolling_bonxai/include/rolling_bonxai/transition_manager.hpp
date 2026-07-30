#pragma once
#include <vector>
#include <cstdint>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include "rolling_bonxai/common.hpp"
#include "rolling_bonxai/coordinate_system.hpp"
#include "rolling_bonxai/logger.hpp"

/**
 * @file transition_manager.hpp
 * @brief Manages chunk transitions with hysteresis to prevent thrashing from localization noise
 * 
 * The TransitionManager implements a state machine that prevents rapid chunk transitions
 * caused by small position estimation errors near chunk boundaries. It requires the robot
 * to penetrate a threshold distance into a new chunk before confirming the transition.
 * 
 * State Flow (V2):
 *   UNINITIALIZED -> STABLE_SOURCE -> HYSTERESIS_SOURCE -> HYSTERESIS_NEXT -> STABLE_NEXT
 *                         ^                                                        |
 *                         |________________________________________________________|
 *                                    (transition confirmed, reset to STABLE_SOURCE)
 * 
 * In V2, only transitions from STABLE_NEXT trigger I/O operations. Other states are
 * tracked for telemetry and future V3 predictive preloading features.
 */

namespace RollingBonxai
{

/**
 * @enum TransitionState
 * @brief States in the chunk transition state machine
 * 
 * UNINITIALIZED:    Initial state before first update
 * STABLE_SOURCE:    Robot is deep inside the current (source) chunk, far from boundaries
 * HYSTERESIS_SOURCE: Robot is approaching a boundary but still within source chunk
 * HYSTERESIS_NEXT:  Robot has crossed boundary but penetration < hysteresis threshold
 * STABLE_NEXT:      Robot has penetrated far enough into next chunk to confirm transition
 */
enum class TransitionState : uint8_t
{
    UNINITIALIZED = 0,
    STABLE_SOURCE = 1,
    HYSTERESIS_SOURCE = 2,
    HYSTERESIS_NEXT = 3,
    STABLE_NEXT = 4
};

/**
 * @brief Convert TransitionState enum to string for logging/debugging
 * @param state The transition state to convert
 * @return String representation of the state
 */
inline std::string reflectTransitionState(const TransitionState &state) {
    std::array<std::string, 5> mapping = {
        "UNINITIALIZED",
        "STABLE_SOURCE",
        "HYSTERESIS_SOURCE",
        "HYSTERESIS_NEXT",
        "STABLE_NEXT"
    };
    return mapping[static_cast<uint8_t>(state)];
}

/**
 * @struct TransitionManagerContext
 * @brief Input context for transition manager updates
 */
struct TransitionManagerContext
{
    Position3D ego_position;        ///< Current robot position in world coordinates
    LinearVelocity3D ego_velocity;  ///< Current robot velocity (for future predictive features)
};

/**
 * @class TransitionManager
 * @brief Manages chunk transitions with hysteresis to prevent I/O thrashing
 * 
 * The TransitionManager prevents rapid chunk transitions caused by localization noise
 * by requiring the robot to penetrate a threshold distance (hysteresis zone) into a
 * new chunk before confirming the transition.
 * 
 * Usage:
 * @code
 *   TransitionManager tm(0.2, coord_system);  // 20% hysteresis
 *   
 *   // In update loop:
 *   TransitionManagerContext ctx{robot_pos, robot_vel};
 *   if (tm.shouldTriggerTransition(ctx)) {
 *       ChunkCoord current = tm.getRefChunkCoord();
 *       // Trigger I/O: load current chunk's neighborhood, evict old chunks
 *   }
 * @endcode
 */
class TransitionManager
{
public:
    /**
     * @brief Construct a new TransitionManager
     * @param hyst_ratio Hysteresis ratio (0.01 to 1.0). Defines threshold as fraction of chunk size.
     *                   Default 0.10 means robot must penetrate 10% of chunk size (e.g., 1m for 10m chunks)
     * @param csys Reference to the coordinate system for chunk/position conversions
     * @param logger Optional logger for degraded-input reporting. When null,
     *               skipped cycles are silent — pass one in so a robot feeding
     *               this a bad pose stream is visible rather than merely
     *               mysteriously stationary.
     *
     * Example: For 10m chunks with hyst_ratio=0.2:
     *   - Hysteresis zone = 2m
     *   - Robot must move 2m into new chunk before transition confirms
     */
    TransitionManager(double hyst_ratio,
                      ChunkCoordinateSystem& csys,
                      Logger::SharedPtr logger = nullptr);

    /**
     * @brief Number of update cycles skipped because the ego pose was not finite
     *
     * Exposed so a caller can surface this as a diagnostic without scraping logs.
     */
    [[nodiscard]] uint64_t getNonFinitePoseSkipCount() const;

    /**
     * @brief Number of update cycles skipped because the state machine reached a
     *        geometrically impossible state
     */
    [[nodiscard]] uint64_t getDegenerateStateSkipCount() const;

    /**
     * @brief Check if a chunk transition should be triggered
     * @param context Current robot state (position, velocity)
     * @return true if I/O operations should be triggered (initial load or transition confirmed)
     *         false if no action needed (same chunk or still in hysteresis zone)
     * 
     * State-specific behavior:
     *   - First call (UNINITIALIZED): Returns true to trigger initial chunk load
     *   - STABLE_SOURCE/HYSTERESIS_SOURCE: Returns false (within current chunk)
     *   - HYSTERESIS_NEXT: Returns false (waiting for confirmation)
     *   - STABLE_NEXT: Returns true (transition confirmed, trigger I/O)
     * 
     * After returning true, call getRefChunkCoord() to get the chunk coordinate
     * that should be used as the center for loading operations.
     */
    bool shouldTriggerTransition(const TransitionManagerContext& context);

    /**
     * @brief Get the current (reference) transition state
     * @return Current state in the transition state machine
     * 
     * This is the "official" state after the most recent update.
     */
    [[nodiscard]] TransitionState getRefTransitionState() const;

    /**
     * @brief Get the previous transition state
     * @return State before the most recent update
     * 
     * Useful for detecting state transitions:
     * @code
     *   if (getPrevTransitionState() == HYSTERESIS_NEXT && 
     *       getRefTransitionState() == STABLE_NEXT) {
     *       // Just confirmed transition!
     *   }
     * @endcode
     */
    [[nodiscard]] TransitionState getPrevTransitionState() const;

    /**
     * @brief Get the current reference chunk coordinate
     * @return The "official" chunk the robot is assigned to
     * 
     * This is the chunk coordinate that should be used for loading operations.
     * It may differ from the geometric chunk coordinate during hysteresis.
     * 
     * Example:
     *   - Robot at x=5.5m (geometrically in Chunk B)
     *   - State is HYSTERESIS_NEXT
     *   - getRefChunkCoord() returns Chunk A (official assignment)
     *   - Only when state becomes STABLE_NEXT does this update to Chunk B
     */
    [[nodiscard]] ChunkCoord getRefChunkCoord() const;

    /**
     * @brief Get the previous reference chunk coordinate
     * @return Chunk coordinate before the most recent confirmed transition
     * 
     * Useful for determining which chunks to evict after a transition.
     */
    [[nodiscard]] ChunkCoord getPrevChunkCoord() const;

    /**
     * @brief Get the configured hysteresis ratio
     * @return Hysteresis threshold as fraction of chunk size (0.01 to 1.0)
     */
    [[nodiscard]] double getHysteresisRatio() const;

private:
    /**
     * @brief Update internal ego state from context
     * @param context Input context with current robot state
     */
    void updateEgoState(const TransitionManagerContext& context);

    /**
     * @brief Set the reference chunk coordinate (official assignment)
     * @param coord New reference chunk coordinate
     */
    void setRefChunkCoord(const ChunkCoord& coord);

    /**
     * @brief Set the previous chunk coordinate
     * @param coord Previous chunk coordinate
     */
    void setPrevChunkCoord(const ChunkCoord& coord);

    /**
     * @brief Set the reference transition state
     * @param state New transition state
     */
    void setRefTransitionState(const TransitionState& state);

    /**
     * @brief Set the previous transition state
     * @param state Previous transition state
     */
    void setPrevTransitionState(const TransitionState& state);

    /**
     * @brief Detect if robot has jumped multiple chunks (teleportation)
     * @param context Current robot state
     * @return true if Chebyshev distance > 1 chunk (non-adjacent jump)
     * 
     * Chunk jumps bypass hysteresis and trigger immediate transition.
     * This handles cases like:
     *   - Robot respawn/reset
     *   - Large odometry corrections
     *   - Map re-localization
     */
    bool detectChunkJump(const TransitionManagerContext& context) const;

    /**
     * @brief Calculate penetration depth into next chunk from shared boundary
     * @param context Current robot state
     * @param reference_chunk Source chunk (where robot came from)
     * @param next_chunk Destination chunk (where robot is geometrically)
     * @return Penetration distance in meters (perpendicular to boundary)
     * 
     * For face transitions (1 axis changes): Returns penetration along that axis
     * For edge transitions (2 axes change): Returns minimum penetration across both axes
     * For corner transitions (3 axes change): Returns minimum penetration across all axes
     * 
     * Example:
     *   - Robot at (5.5, 0, 0), moving from Chunk A (0,0,0) to Chunk B (1,0,0)
     *   - Boundary at x=5.0m
     *   - Returns: 0.5m (penetration into B)
     */
    double calculateBoundaryPenetration(
        const TransitionManagerContext& context,
        const ChunkCoord& reference_chunk,
        const ChunkCoord& next_chunk) const;

    /**
     * @brief Compute state when robot is in the same chunk as reference
     * @param context Current robot state
     * @param current_chunk_coord Geometric chunk coordinate (should equal reference)
     * @return New transition state based on distance to boundary
     * 
     * State logic:
     *   - distance > hysteresis_zone → STABLE_SOURCE
     *   - distance ≤ hysteresis_zone → HYSTERESIS_SOURCE
     */
    TransitionState computeStateIfSameChunk(
        const TransitionManagerContext& context,
        const ChunkCoord& current_chunk_coord);

    /**
     * @brief Compute state when robot is in a different chunk than reference
     * @param context Current robot state
     * @param current_chunk_coord Geometric chunk coordinate (neighbor of reference)
     * @return New transition state based on penetration depth
     * 
     * State logic:
     *   - penetration < hysteresis_zone → HYSTERESIS_NEXT (waiting for confirmation)
     *   - penetration ≥ hysteresis_zone → STABLE_NEXT (transition confirmed)
     */
    TransitionState computeStateIfNextChunk(
        const TransitionManagerContext& context,
        const ChunkCoord& current_chunk_coord);

    /**
     * @brief Report a skipped update cycle without flooding the log
     *
     * Skips arrive at sensor rate, so one line each would bury everything else —
     * the very problem that made the previous per-cloud info logging useless.
     * Instead: warn on the first skip of a run, then every kSkipLogStride-th, so
     * a persistent fault keeps announcing itself but a burst costs a few lines.
     *
     * @param lifetime_count Lifetime skip count for this cause, already incremented.
     *        The rate limit keys off this rather than the run length, so an
     *        alternating good/bad pose stream cannot restart the run at 1 every
     *        cycle and defeat the limit.
     * @param run_length Consecutive-skip counter for this cause, already incremented
     * @param what Human-readable description of the cause
     */
    void reportSkip(uint64_t lifetime_count, uint64_t run_length, const std::string& what);

    /**
     * @brief Log a recovery summary if a run of skipped cycles just ended
     */
    void reportRecoveryIfNeeded();

    /// Warn on the 1st skip of a run and every Nth after it. At a 10-30 Hz
    /// sensor rate this is roughly one line every 15-50 s while a fault lasts.
    static constexpr uint64_t kSkipLogStride = 500;

    // --- Member Variables ---

    bool initialized_{false};  ///< True after first update

    // Robot state
    Position3D ego_position_;         ///< Last updated robot position
    LinearVelocity3D ego_velocity_;   ///< Last updated robot velocity

    // Chunk tracking
    ChunkCoord reference_chunk_coord_;  ///< Official chunk assignment
    ChunkCoord previous_chunk_coord_;   ///< Previous official chunk (before last transition)

    // State machine
    TransitionState reference_t_state_{TransitionState::UNINITIALIZED};  ///< Current state
    TransitionState previous_t_state_{TransitionState::UNINITIALIZED};   ///< Previous state

    // Configuration
    double hyst_ratio_{0.10};      ///< Hysteresis threshold as fraction of chunk size
    double preload_ratio_{0.05};   ///< Preload threshold (reserved for V3 predictive loading)

    // Coordinate system reference
    ChunkCoordinateSystem& csys_;  ///< Reference to coordinate system for conversions

    // Degraded-input reporting
    Logger::SharedPtr logger_;                  ///< Optional; null means stay silent
    uint64_t nonfinite_pose_skips_{0};          ///< Consecutive non-finite-pose skips
    uint64_t degenerate_state_skips_{0};        ///< Consecutive impossible-state skips
    uint64_t total_nonfinite_pose_skips_{0};    ///< Lifetime count
    uint64_t total_degenerate_state_skips_{0};  ///< Lifetime count
    bool warned_current_run_{false};            ///< A warning was emitted for the run in progress
};

} // namespace RollingBonxai