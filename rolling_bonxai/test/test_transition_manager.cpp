#include <gtest/gtest.h>

#include <limits>
#include <sstream>

#include "rolling_bonxai/transition_manager.hpp"
#include "rolling_bonxai/coordinate_system.hpp"

using namespace RollingBonxai;

/**
 * @brief Test fixture for TransitionManager tests
 * 
 * Sets up a standard test environment with:
 * - 10m chunk size
 * - 0.2 hysteresis ratio (2m threshold)
 * - Origin-centered coordinate system
 */
class TransitionManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard 10m chunks with 0.2 (20%) hysteresis = 2m threshold
        chunk_size_ = 10.0;
        hyst_ratio_ = 0.2;
        
        // Create coordinate system
        csys_ = std::make_unique<ChunkCoordinateSystem>(0.2,chunk_size_);
        
        // Create transition manager
        tm_ = std::make_unique<TransitionManager>(hyst_ratio_, *csys_);
    }

    std::string strChunk(const ChunkCoord& chunk) const {
        std::ostringstream oss;
        oss << "(" << chunk.x << " " << chunk.y << " " << chunk.z << ")";
        return oss.str();
    }

    // Helper to create context
    TransitionManagerContext makeContext(double x, double y = 0.0, double z = 0.0,
                                         double vx = 0.0, double vy = 0.0, double vz = 0.0) {
        TransitionManagerContext ctx;
        ctx.ego_position = Position3D(x, y, z);
        ctx.ego_velocity = LinearVelocity3D(vx, vy, vz);
        return ctx;
    }

    double chunk_size_;
    double hyst_ratio_;
    std::unique_ptr<ChunkCoordinateSystem> csys_;
    std::unique_ptr<TransitionManager> tm_;
};

// ============================================================================
// INITIALIZATION TESTS
// ============================================================================

TEST_F(TransitionManagerTest, InitializationAtOrigin) {
    // Robot starts at origin (center of Chunk (0,0,0))
    auto ctx = makeContext(0.0, 0.0, 0.0);
    
    bool triggered = tm_->shouldTriggerTransition(ctx);
    
    // Should trigger initial load
    EXPECT_TRUE(triggered);
    
    // Should be in STABLE_SOURCE (far from any boundary)
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
    
    // Should be assigned to chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
    
    // Previous state should be UNINITIALIZED
    EXPECT_EQ(tm_->getPrevTransitionState(), TransitionState::UNINITIALIZED);
}

TEST_F(TransitionManagerTest, InitializationNearBoundary) {
    // Robot starts near boundary at x=4m (1m from boundary at x=5m)
    auto ctx = makeContext(4.0, 0.0, 0.0);
    
    bool triggered = tm_->shouldTriggerTransition(ctx);
    
    // Should still trigger initial load
    EXPECT_TRUE(triggered);
    
    // Should be in HYSTERESIS_SOURCE (within 2m of boundary)
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    // Should be assigned to chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, InitializationOffCenter) {
    // Robot starts at (15, 7, -3) - chunk (2, 1, 0)
    auto ctx = makeContext(15.0, 7.0, -3.0);
    bool triggered = tm_->shouldTriggerTransition(ctx);
    
    EXPECT_TRUE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    ChunkCoord expected(2, 1, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // STABLE STATE TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, StableStateDeepInChunk) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move slightly but stay deep in chunk
    auto ctx2 = makeContext(1.0, 0.5, -0.5);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should not trigger transition
    EXPECT_FALSE(triggered);
    
    // Should remain in STABLE_SOURCE
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
    
    // Should remain in chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, StableToHysteresisSourceTransition) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move to within hysteresis zone (x=3.5m, 1.5m from boundary)
    auto ctx2 = makeContext(3.5, 0.0, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should not trigger transition
    EXPECT_FALSE(triggered);
    
    // Should transition to HYSTERESIS_SOURCE
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    EXPECT_EQ(tm_->getPrevTransitionState(), TransitionState::STABLE_SOURCE);
    
    // Should remain in chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // HYSTERESIS TESTS - CORE FEATURE
// // ============================================================================

TEST_F(TransitionManagerTest, HysteresisPreventsFalseTransition) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Cross boundary slightly (x=5.5m, 0.5m into next chunk)
    auto ctx2 = makeContext(5.5, 0.0, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should NOT trigger transition (penetration 0.5m < 2m threshold)
    EXPECT_FALSE(triggered);
    
    // Should be in HYSTERESIS_NEXT state
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    // Should STILL be assigned to chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, HysteresisAllowsOscillation) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Oscillate around boundary
    std::vector<double> positions = {4.9, 5.1, 4.95, 5.05, 4.98, 5.02};
    
    for (double x : positions) {
        auto ctx = makeContext(x, 0.0, 0.0);
        bool triggered = tm_->shouldTriggerTransition(ctx);
        
        // None should trigger transition
        EXPECT_FALSE(triggered) << "Position x=" << x << " should not trigger";
        
        // Should remain assigned to chunk (0,0,0)
        ChunkCoord expected(0, 0, 0);
        EXPECT_EQ(tm_->getRefChunkCoord(), expected);
    }
}

TEST_F(TransitionManagerTest, HysteresisConfirmsGenuineTransition) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0); // initialization
    bool triggered = tm_->shouldTriggerTransition(ctx1);
    EXPECT_TRUE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);

    // Move gradually across boundary
    auto ctx2 = makeContext(5.5, 0.0, 0.0);  // 0.5m penetration, inside hysteresis zone
    triggered = tm_->shouldTriggerTransition(ctx2);
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    auto ctx3 = makeContext(6.0, 0.0, 0.0);  // 1.0m penetration, inside hysteresis zone
    triggered = tm_->shouldTriggerTransition(ctx3);
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);

    auto ctx4 = makeContext(6.9, 0.0, 0.0);  // 1.90m penetration inside hysteresis zone
    triggered = tm_->shouldTriggerTransition(ctx4);
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);

    auto ctx5 = makeContext(7.0, 0.0, 0.0);  // 2.0m penetration should trigger transition
    triggered = tm_->shouldTriggerTransition(ctx5);
    EXPECT_TRUE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);

    auto ctx6 = makeContext(7.1, 0.0, 0.0);  // Should be stable source because state is transitionex
    triggered = tm_->shouldTriggerTransition(ctx6);
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
    
    auto ctx7 = makeContext(7.5, 0.0, 0.0);  // 2.5m penetration - EXCEEDS THRESHOLD
    triggered = tm_->shouldTriggerTransition(ctx7);
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
    
    // Should now be assigned to chunk (1,0,0)
    ChunkCoord expected(1, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
    
    // Previous chunk should be (0,0,0)
    ChunkCoord prev_expected(0, 0, 0);
    EXPECT_EQ(tm_->getPrevChunkCoord(), prev_expected);
}

TEST_F(TransitionManagerTest, HysteresisReturnToSource) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Cross boundary slightly
    auto ctx2 = makeContext(5.5, 0.0, 0.0);
    EXPECT_FALSE(tm_->shouldTriggerTransition(ctx2));
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    // Return to source chunk
    auto ctx3 = makeContext(4.5, 0.0, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx3);
    
    // Should not trigger transition
    EXPECT_FALSE(triggered);
    
    // Should return to HYSTERESIS_SOURCE
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    // Should still be assigned to chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // MULTI-AXIS TRANSITION TESTS (EDGE AND CORNER)
// // ============================================================================

TEST_F(TransitionManagerTest, EdgeTransitionHysteresis) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move diagonally across corner (both X and Y change)
    // x=5.5, y=5.5 (0.5m penetration on both axes)
    auto ctx2 = makeContext(5.5, 5.5, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should NOT trigger (min penetration 0.5m < 2m threshold)
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    // Should still be in chunk (0,0,0)
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, EdgeTransitionConfirmation) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move far enough on both axes
    // x=7.5, y=7.5 (2.5m penetration on both axes)
    auto ctx2 = makeContext(7.5, 7.5, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should trigger (min penetration 2.5m > 2m threshold)
    EXPECT_TRUE(triggered);
    
    // Should now be in chunk (1,1,0)
    ChunkCoord expected(1, 1, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, EdgeTransitionAsymmetricPenetration) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Asymmetric penetration: x=7.5m (2.5m), y=5.5m (0.5m)
    auto ctx2 = makeContext(7.5, 5.5, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should NOT trigger (minimum penetration is 0.5m < 2m)
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    ChunkCoord expected(0, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, CornerTransition3D) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move diagonally in 3D space
    // All three axes change: (7.5, 7.5, 7.5)
    auto ctx2 = makeContext(7.5, 7.5, 7.5);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should trigger (all axes exceed 2m threshold)
    EXPECT_TRUE(triggered);
    
    // Should now be in chunk (1,1,1)
    ChunkCoord expected(1, 1, 1);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // NEGATIVE DIRECTION TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, NegativeDirectionTransition) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move in negative X direction
    auto ctx2 = makeContext(-5.5, 0.0, 0.0);  // 0.5m into chunk (-1,0,0)
    EXPECT_FALSE(tm_->shouldTriggerTransition(ctx2));
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    auto ctx3 = makeContext(-7.5, 0.0, 0.0);  // 2.5m penetration
    bool triggered = tm_->shouldTriggerTransition(ctx3);
    
    EXPECT_TRUE(triggered);
    
    ChunkCoord expected(-1, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, NegativeDirectionAllAxes) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Move in all negative directions
    auto ctx2 = makeContext(-7.5, -7.5, -7.5);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    EXPECT_TRUE(triggered);
    
    ChunkCoord expected(-1, -1, -1);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // CHUNK JUMP (TELEPORTATION) TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, ChunkJumpBypassesHysteresis) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Teleport to chunk (3,0,0) - jumps 3 chunks
    auto ctx2 = makeContext(30.0, 0.0, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    // Should immediately trigger transition (bypassing hysteresis)
    EXPECT_TRUE(triggered);
    
    // Should be assigned to chunk (3,0,0)
    ChunkCoord expected(3, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
    
    // Should be in STABLE_SOURCE
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
}

TEST_F(TransitionManagerTest, ChunkJumpNegativeDirection) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Teleport to chunk (-5,0,0)
    auto ctx2 = makeContext(-50.0, 0.0, 0.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    
    EXPECT_TRUE(triggered);
    
    ChunkCoord expected(-5, 0, 0);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

TEST_F(TransitionManagerTest, ChunkJump3D) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Teleport to chunk (2,3,4)
    auto ctx2 = makeContext(25.0, 35.0, 45.0);
    bool triggered = tm_->shouldTriggerTransition(ctx2);
    EXPECT_TRUE(triggered);
    
    ChunkCoord expected(3, 4, 5);
    EXPECT_EQ(tm_->getRefChunkCoord(), expected);
}

// // ============================================================================
// // SEQUENTIAL TRANSITION TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, MultipleSequentialTransitions) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(0, 0, 0));
    
    // Transition to chunk (1,0,0)
    auto ctx2 = makeContext(12.0, 0.0, 0.0);
    EXPECT_TRUE(tm_->shouldTriggerTransition(ctx2));
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(1, 0, 0));
    
    // Transition to chunk (2,0,0)
    auto ctx3 = makeContext(22.0, 0.0, 0.0);
    EXPECT_TRUE(tm_->shouldTriggerTransition(ctx3));
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(2, 0, 0));
    
    // Transition back to chunk (1,0,0)
    auto ctx4 = makeContext(12.0, 0.0, 0.0);
    EXPECT_TRUE(tm_->shouldTriggerTransition(ctx4));
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(1, 0, 0));
}

TEST_F(TransitionManagerTest, RapidBoundaryApproachAndRetreat) {
    // Initialize at origin
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Approach boundary
    auto ctx2 = makeContext(3.5, 0.0, 0.0);
    EXPECT_FALSE(tm_->shouldTriggerTransition(ctx2));
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    // Get even closer
    auto ctx3 = makeContext(4.5, 0.0, 0.0);
    EXPECT_FALSE(tm_->shouldTriggerTransition(ctx3));
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    // Retreat
    auto ctx4 = makeContext(2.0, 0.0, 0.0);
    EXPECT_FALSE(tm_->shouldTriggerTransition(ctx4));
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
    
    // Should remain in chunk (0,0,0) throughout
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(0, 0, 0));
}

// // ============================================================================
// // CONFIGURATION TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, DifferentHysteresisRatios) {
    // Test with very small hysteresis (1% = 0.1m)
    TransitionManager tm_small(0.01, *csys_);
    
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_small.shouldTriggerTransition(ctx1);
    
    // Should trigger with just 0.15m penetration
    auto ctx2 = makeContext(5.15, 0.0, 0.0);
    bool triggered = tm_small.shouldTriggerTransition(ctx2);
    
    EXPECT_TRUE(triggered);
    EXPECT_EQ(tm_small.getRefChunkCoord(), ChunkCoord(1, 0, 0));
}

TEST_F(TransitionManagerTest, LargeHysteresisRatio) {
    // Test with large hysteresis (50% = 5m)
    TransitionManager tm_large(0.5, *csys_);
    
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_large.shouldTriggerTransition(ctx1);
    
    // Even at 8m into next chunk (3m penetration), should NOT trigger
    auto ctx2 = makeContext(8.0, 0.0, 0.0);
    bool triggered = tm_large.shouldTriggerTransition(ctx2);
    
    EXPECT_FALSE(triggered);
    EXPECT_EQ(tm_large.getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    // Need to go to 10m+ (5m penetration) to trigger
    auto ctx3 = makeContext(10.5, 0.0, 0.0);
    triggered = tm_large.shouldTriggerTransition(ctx3);
    
    EXPECT_TRUE(triggered);
    EXPECT_EQ(tm_large.getRefChunkCoord(), ChunkCoord(1, 0, 0));
}

TEST_F(TransitionManagerTest, HysteresisRatioClamping) {
    // Test that ratios outside [0.01, 1.0] are clamped
    TransitionManager tm_low(-0.5, *csys_);
    EXPECT_GE(tm_low.getHysteresisRatio(), 0.01);
    
    TransitionManager tm_high(2.0, *csys_);
    EXPECT_LE(tm_high.getHysteresisRatio(), 1.0);
}

// // ============================================================================
// // STATE TRACKING TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, StateTransitionTracking) {
    // Initialize
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    TransitionState prev = tm_->getPrevTransitionState();
    TransitionState curr = tm_->getRefTransitionState();
    
    EXPECT_EQ(prev, TransitionState::UNINITIALIZED);
    EXPECT_EQ(curr, TransitionState::STABLE_SOURCE);
    
    // Move to hysteresis zone
    auto ctx2 = makeContext(3.5, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx2);
    
    EXPECT_EQ(tm_->getPrevTransitionState(), TransitionState::STABLE_SOURCE);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    
    // Cross boundary
    auto ctx3 = makeContext(5.5, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx3);
    
    EXPECT_EQ(tm_->getPrevTransitionState(), TransitionState::HYSTERESIS_SOURCE);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::HYSTERESIS_NEXT);
    
    // Confirm transition
    auto ctx4 = makeContext(7.5, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx4);
    
    EXPECT_EQ(tm_->getPrevTransitionState(), TransitionState::STABLE_NEXT);
    EXPECT_EQ(tm_->getRefTransitionState(), TransitionState::STABLE_SOURCE);
}

// // ============================================================================
// // BOUNDARY PENETRATION CALCULATION TESTS
// // ============================================================================

TEST_F(TransitionManagerTest, PenetrationCalculationFaceTransition) {
    // This is an indirect test through state behavior
    // Direct penetration calculation is private, but we can verify through states
    
    auto ctx1 = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx1);
    
    // Test various penetration depths
    struct TestCase {
        double x;
        double expected_penetration;
        TransitionState expected_state;
    };
    
    std::vector<TestCase> cases = {
        {5.1, 0.1, TransitionState::HYSTERESIS_NEXT},  // Minimal penetration
        {5.5, 0.5, TransitionState::HYSTERESIS_NEXT},
        {6.0, 1.0, TransitionState::HYSTERESIS_NEXT},
        {6.99, 1.99, TransitionState::HYSTERESIS_NEXT}, // Just below threshold
        {7.0, 2.0, TransitionState::STABLE_SOURCE},       // Exactly at threshold
        {7.5, 2.5, TransitionState::STABLE_SOURCE},
        {9.0, 4.0, TransitionState::STABLE_SOURCE},
    };
    
    for (const auto& test : cases) {
        // Reset to origin for each test
        auto reset_ctx = makeContext(0.0, 0.0, 0.0);
        tm_ = std::make_unique<TransitionManager>(hyst_ratio_, *csys_);
        tm_->shouldTriggerTransition(reset_ctx);
        
        auto ctx = makeContext(test.x, 0.0, 0.0);
        tm_->shouldTriggerTransition(ctx);
        
        EXPECT_EQ(tm_->getRefTransitionState(), test.expected_state)
            << "Failed at x=" << test.x << " (penetration=" << test.expected_penetration << ")";
    }
}

// ============================================================================
// NON-FINITE EGO POSES
// ============================================================================
//
// shouldTriggerTransition() runs inside the point cloud subscription callback,
// so anything it throws escapes the executor and terminates the node. A NaN
// from a diverged localiser used to do exactly that: every comparison in
// isPositionInChunk() is false for NaN, so distanceToBoundary() reported the
// ego as outside the chunk it had just been classified into, which produced
// UNINITIALIZED and hit the throw. These pin the non-throwing behaviour.

TEST_F(TransitionManagerTest, NonFinitePositionDoesNotThrowBeforeInit) {
    auto nan_ctx = makeContext(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);

    EXPECT_NO_THROW({
        EXPECT_FALSE(tm_->shouldTriggerTransition(nan_ctx));
    });
}

TEST_F(TransitionManagerTest, NonFinitePositionDoesNotThrowAfterInit) {
    // Initialise normally at the origin
    auto ctx = makeContext(0.0, 0.0, 0.0);
    EXPECT_TRUE(tm_->shouldTriggerTransition(ctx));

    const ChunkCoord chunk_before = tm_->getRefChunkCoord();
    const TransitionState state_before = tm_->getRefTransitionState();

    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
        auto bad_ctx = makeContext(bad, 0.0, 0.0);
        EXPECT_NO_THROW({
            EXPECT_FALSE(tm_->shouldTriggerTransition(bad_ctx));
        });

        // A bad pose must not disturb the state machine either
        EXPECT_EQ(tm_->getRefChunkCoord(), chunk_before);
        EXPECT_EQ(tm_->getRefTransitionState(), state_before);
    }
}

TEST_F(TransitionManagerTest, RecoversAfterNonFinitePosition) {
    auto ctx = makeContext(0.0, 0.0, 0.0);
    tm_->shouldTriggerTransition(ctx);

    auto nan_ctx = makeContext(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    EXPECT_FALSE(tm_->shouldTriggerTransition(nan_ctx));

    // A valid pose far enough into the next chunk must still transition, i.e.
    // the bad cycle was skipped rather than latching the manager into a
    // permanently broken state.
    auto far_ctx = makeContext(chunk_size_, 0.0, 0.0);
    EXPECT_TRUE(tm_->shouldTriggerTransition(far_ctx));
    EXPECT_EQ(tm_->getRefChunkCoord(), ChunkCoord(1, 0, 0));
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}