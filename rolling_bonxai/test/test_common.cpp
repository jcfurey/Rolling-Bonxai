#include <gtest/gtest.h>
#include "rolling_bonxai/common.hpp"
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <thread>

using namespace RollingBonxai;
using Hasher = RollingBonxai::ChunkCoordHash;
// ============================================================================
// ChunkCoord Tests
// ============================================================================

TEST(ChunkCoordTest, DefaultConstructor) {
    ChunkCoord c;
    EXPECT_EQ(c.x, 0);
    EXPECT_EQ(c.y, 0);
    EXPECT_EQ(c.z, 0);
}

TEST(ChunkCoordTest, ParameterizedConstructor) {
    ChunkCoord c(5, -3, 10);
    EXPECT_EQ(c.x, 5);
    EXPECT_EQ(c.y, -3);
    EXPECT_EQ(c.z, 10);
}

TEST(ChunkCoordTest, EqualityOperator) {
    ChunkCoord c1(1, 2, 3);
    ChunkCoord c2(1, 2, 3);
    ChunkCoord c3(1, 2, 4);
    
    EXPECT_TRUE(c1 == c2);
    EXPECT_FALSE(c1 == c3);
    EXPECT_FALSE(c1 != c2);
    EXPECT_TRUE(c1 != c3);
}

TEST(ChunkCoordTest, AdditionOperator) {
    ChunkCoord c1(1, 2, 3);
    ChunkCoord c2(4, -1, 2);
    ChunkCoord result = c1 + c2;
    
    EXPECT_EQ(result.x, 5);
    EXPECT_EQ(result.y, 1);
    EXPECT_EQ(result.z, 5);
}

// ============================================================================
// ChunkHash Tests
// ============================================================================

TEST(ChunkCoordHashTest, Deterministic)
{
    Hasher h;
    ChunkCoord c{1, 2, 3};

    auto v1 = h(c);
    auto v2 = h(c);

    EXPECT_EQ(v1, v2);
}

TEST(ChunkCoordHashTest, DifferentCoordsDifferentHash)
{
    Hasher h;

    ChunkCoord a{1, 2, 3};
    ChunkCoord b{1, 2, 4};
    ChunkCoord c{1, 3, 3};
    ChunkCoord d{2, 2, 3};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(a), h(d));
}

TEST(ChunkCoordHashTest, AxisSensitivity)
{
    Hasher h;
    ChunkCoord base{10, 20, 30};

    auto h0 = h(base);

    EXPECT_NE(h0, h({11, 20, 30}));
    EXPECT_NE(h0, h({10, 21, 30}));
    EXPECT_NE(h0, h({10, 20, 31}));
}


TEST(ChunkCoordHashTest, HandlesNegativeCoordinates)
{
    Hasher h;

    ChunkCoord a{-1, 0, 0};
    ChunkCoord b{0, -1, 0};
    ChunkCoord c{0, 0, -1};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(b), h(c));
}

TEST(ChunkCoordHashTest, NegativeAndPositivesNotSame)
{
    Hasher h;

    ChunkCoord a{10, 12, 77};
    ChunkCoord b{-10, 12, 77};
    ChunkCoord c{10, -12, 77};
    ChunkCoord d{10, 12, -77};
    ChunkCoord e{-10, -12, -77};

    EXPECT_NE(h(a), h(b));
    EXPECT_NE(h(a), h(c));
    EXPECT_NE(h(a), h(d));
    EXPECT_NE(h(a), h(e));
    EXPECT_NE(h(b), h(c));
    EXPECT_NE(h(b), h(d));
    EXPECT_NE(h(b), h(e));
    EXPECT_NE(h(c), h(d));
    EXPECT_NE(h(c), h(e));
    EXPECT_NE(h(d), h(e));
}

TEST(ChunkCoordHashTest, CoordinateOrderMatters)
{
    Hasher h;

    std::array<ChunkCoord,6> possibilities = {{{1, 2, 3},
                                              {1, 3, 2},
                                              {2, 1, 3},
                                              {2, 3, 1},
                                              {3, 1, 2},
                                              {3, 2, 1}}};

    for (size_t i = 0 ; i < possibilities.size() -1 ; ++i) {
        for (size_t j = i + 1 ; j < possibilities.size() ; ++j) {
            auto &a = possibilities[i];
            auto &b = possibilities[j];
            EXPECT_NE(h(a), h(b));
        }
    }
}

TEST(ChunkCoordHashTest, ZeroCoordinate)
{
    Hasher h;
    ChunkCoord zero{0, 0, 0};

    auto value = h(zero);

    EXPECT_NE(value, 0u);  // Not required by spec, but good sanity
}

TEST(ChunkCoordHashTest, WorksInUnorderedMap)
{
    std::unordered_map<ChunkCoord, int, Hasher> map;
    ChunkCoord a(1,2,3);
    ChunkCoord b(4,5,6);
    map[a] = 42;
    map[b] = 99;

    EXPECT_EQ(map[a], 42);
    EXPECT_EQ(map[b], 99);
}

TEST(ChunkCoordHashTest, LowCollisionSmallGrid)
{
    Hasher h;
    std::unordered_set<std::size_t> hashes;

    constexpr int N = 20;
    for (int x = -N; x <= N; ++x)
        for (int y = -N; y <= N; ++y)
            for (int z = -N; z <= N; ++z)
                hashes.insert(h({x, y, z}));

    const std::size_t expected = (2*N + 1) * (2*N + 1) * (2*N + 1);
    EXPECT_GT(hashes.size(), expected * 0.99); // tolerate tiny collisions
}

// ============================================================================
// ChunkTimestamp Tests
// ============================================================================

class ChunkTimestampTest : public ::testing::Test {
protected:
    // Helper: Get current time in nanoseconds since epoch
    int64_t getCurrentTimeNs() const {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
    
    // Helper: Get time N seconds ago in nanoseconds
    int64_t getTimeNSecondsAgo(int seconds) const {
        auto past = std::chrono::system_clock::now() - std::chrono::seconds(seconds);
        auto duration = past.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(ChunkTimestampTest, DefaultConstructorInitializesToZero) {
    ChunkTimestamp timestamp;
    
    EXPECT_EQ(timestamp.creation_time_ns, 0);
    EXPECT_EQ(timestamp.last_modified_ns, 0);
    EXPECT_EQ(timestamp.last_accessed_ns, 0);
    EXPECT_EQ(timestamp.access_count, 0);
}

// ============================================================================
// Time Point Conversion Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetCreationTimeReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getCurrentTimeNs();
    
    auto time_point = timestamp.getCreationTime();
    auto now = std::chrono::system_clock::now();
    
    // Should be very close to current time (within 1 second)
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_LE(std::abs(diff.count()), 1);
}

TEST_F(ChunkTimestampTest, GetLastModifiedReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(60);  // 60 seconds ago
    
    auto time_point = timestamp.getLastModified();
    auto now = std::chrono::system_clock::now();
    
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_GE(diff.count(), 59);  // At least 59 seconds ago
    EXPECT_LE(diff.count(), 61);  // At most 61 seconds ago
}

TEST_F(ChunkTimestampTest, GetLastAccessedReturnsCorrectTimePoint) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(30);  // 30 seconds ago
    
    auto time_point = timestamp.getLastAccessed();
    auto now = std::chrono::system_clock::now();
    
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - time_point);
    EXPECT_GE(diff.count(), 29);
    EXPECT_LE(diff.count(), 31);
}

TEST_F(ChunkTimestampTest, ConversionWithZeroTimestamp) {
    ChunkTimestamp timestamp;
    // All fields are 0 (epoch)
    
    auto creation = timestamp.getCreationTime();
    auto modified = timestamp.getLastModified();
    auto accessed = timestamp.getLastAccessed();
    
    // Should all be at epoch (January 1, 1970)
    auto epoch = std::chrono::system_clock::time_point{};
    EXPECT_EQ(creation, epoch);
    EXPECT_EQ(modified, epoch);
    EXPECT_EQ(accessed, epoch);
}

// ============================================================================
// Age Calculation Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetAgeWithRecentCreation) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getTimeNSecondsAgo(100);  // Created 100 seconds ago
    
    auto age = timestamp.getAge();
    
    EXPECT_GE(age.count(), 99);   // At least 99 seconds
    EXPECT_LE(age.count(), 101);  // At most 101 seconds
}

TEST_F(ChunkTimestampTest, GetAgeWithOldCreation) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getTimeNSecondsAgo(3600);  // Created 1 hour ago
    
    auto age = timestamp.getAge();
    
    EXPECT_GE(age.count(), 3599);
    EXPECT_LE(age.count(), 3601);
}

TEST_F(ChunkTimestampTest, GetAgeWithZeroCreationTime) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = 0;  // Epoch
    
    auto age = timestamp.getAge();
    
    // Should be approximately current time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected_age = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    // Within 1 second tolerance
    EXPECT_NEAR(age.count(), expected_age.count(), 1);
}

TEST_F(ChunkTimestampTest, GetAgeIncreasesContinuously) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = getCurrentTimeNs();
    
    auto age1 = timestamp.getAge();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    auto age2 = timestamp.getAge();
    
    // Second measurement should show older age
    EXPECT_GT(age2.count(), age1.count());
}

// ============================================================================
// Time Since Modified Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedRecent) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(50);  // Modified 50 seconds ago
    
    auto time_since = timestamp.getTimeSinceModified();
    
    EXPECT_GE(time_since.count(), 49);
    EXPECT_LE(time_since.count(), 51);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedOld) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getTimeNSecondsAgo(7200);  // Modified 2 hours ago
    
    auto time_since = timestamp.getTimeSinceModified();
    
    EXPECT_GE(time_since.count(), 7199);
    EXPECT_LE(time_since.count(), 7201);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedJustNow) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = getCurrentTimeNs();
    
    auto time_since = timestamp.getTimeSinceModified();
    
    // Should be very small (less than 1 second)
    EXPECT_LE(time_since.count(), 1);
}

TEST_F(ChunkTimestampTest, GetTimeSinceModifiedWithZero) {
    ChunkTimestamp timestamp;
    timestamp.last_modified_ns = 0;  // Never modified (or at epoch)
    
    auto time_since = timestamp.getTimeSinceModified();
    
    // Should be approximately time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    EXPECT_NEAR(time_since.count(), expected.count(), 1);
}

// ============================================================================
// Time Since Accessed Tests
// ============================================================================

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedRecent) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(30);  // Accessed 30 seconds ago
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    EXPECT_GE(time_since.count(), 29);
    EXPECT_LE(time_since.count(), 31);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedLongAgo) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getTimeNSecondsAgo(86400);  // Accessed 24 hours ago
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    EXPECT_GE(time_since.count(), 86399);
    EXPECT_LE(time_since.count(), 86401);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedJustNow) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = getCurrentTimeNs();
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    // Should be very small
    EXPECT_LE(time_since.count(), 1);
}

TEST_F(ChunkTimestampTest, GetTimeSinceAccessedWithZero) {
    ChunkTimestamp timestamp;
    timestamp.last_accessed_ns = 0;
    
    auto time_since = timestamp.getTimeSinceAccessed();
    
    // Should be approximately time since epoch
    auto now = std::chrono::system_clock::now();
    auto expected = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()
    );
    
    EXPECT_NEAR(time_since.count(), expected.count(), 1);
}

// ============================================================================
// Realistic Timestamp Scenario Tests
// ============================================================================

TEST_F(ChunkTimestampTest, TypicalChunkLifecycle) {
    ChunkTimestamp timestamp;
    
    // Chunk created 1 hour ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(3600);
    
    // Last modified 30 minutes ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(1800);
    
    // Last accessed 5 minutes ago
    timestamp.last_accessed_ns = getTimeNSecondsAgo(300);
    
    // Accessed 10 times
    timestamp.access_count = 10;
    
    // Verify age
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 3599);
    EXPECT_LE(age.count(), 3601);
    
    // Verify time since modified
    auto since_modified = timestamp.getTimeSinceModified();
    EXPECT_GE(since_modified.count(), 1799);
    EXPECT_LE(since_modified.count(), 1801);
    
    // Verify time since accessed
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_GE(since_accessed.count(), 299);
    EXPECT_LE(since_accessed.count(), 301);
    
    // Verify access count
    EXPECT_EQ(timestamp.access_count, 10);
}

TEST_F(ChunkTimestampTest, NewlyCreatedChunk) {
    ChunkTimestamp timestamp;
    
    auto now_ns = getCurrentTimeNs();
    timestamp.creation_time_ns = now_ns;
    timestamp.last_modified_ns = now_ns;
    timestamp.last_accessed_ns = now_ns;
    timestamp.access_count = 0;
    
    // All time-since values should be near zero
    EXPECT_LE(timestamp.getAge().count(), 1);
    EXPECT_LE(timestamp.getTimeSinceModified().count(), 1);
    EXPECT_LE(timestamp.getTimeSinceAccessed().count(), 1);
    EXPECT_EQ(timestamp.access_count, 0);
}

TEST_F(ChunkTimestampTest, FrequentlyAccessedChunk) {
    ChunkTimestamp timestamp;
    
    // Created a week ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(604800);
    
    // Modified yesterday
    timestamp.last_modified_ns = getTimeNSecondsAgo(86400);
    
    // Accessed just now
    timestamp.last_accessed_ns = getCurrentTimeNs();
    
    // Accessed 1000 times
    timestamp.access_count = 1000;
    
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 604799);
    EXPECT_LE(age.count(), 604801);
    
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_LE(since_accessed.count(), 1);
    
    EXPECT_EQ(timestamp.access_count, 1000);
}

TEST_F(ChunkTimestampTest, StaleChunk) {
    ChunkTimestamp timestamp;
    
    // Created 30 days ago
    timestamp.creation_time_ns = getTimeNSecondsAgo(2592000);
    
    // Last modified 30 days ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(2592000);
    
    // Last accessed 7 days ago
    timestamp.last_accessed_ns = getTimeNSecondsAgo(604800);
    
    // Only accessed once
    timestamp.access_count = 1;
    
    auto age = timestamp.getAge();
    EXPECT_GE(age.count(), 2591999);
    
    auto since_modified = timestamp.getTimeSinceModified();
    EXPECT_GE(since_modified.count(), 2591999);
    
    auto since_accessed = timestamp.getTimeSinceAccessed();
    EXPECT_GE(since_accessed.count(), 604799);
    EXPECT_LE(since_accessed.count(), 604801);
}

// ============================================================================
// Comparison and Relative Time Tests
// ============================================================================

TEST_F(ChunkTimestampTest, CompareModifiedVsAccessed) {
    ChunkTimestamp timestamp;
    
    // Modified 1 hour ago
    timestamp.last_modified_ns = getTimeNSecondsAgo(3600);
    
    // Accessed 10 minutes ago (more recent)
    timestamp.last_accessed_ns = getTimeNSecondsAgo(600);
    
    auto since_modified = timestamp.getTimeSinceModified();
    auto since_accessed = timestamp.getTimeSinceAccessed();
    
    // Accessed should be more recent (smaller time-since value)
    EXPECT_LT(since_accessed.count(), since_modified.count());
}

TEST_F(ChunkTimestampTest, AccessCountRanges) {
    ChunkTimestamp timestamp;
    
    // Test various access counts
    std::vector<uint64_t> counts = {0, 1, 10, 100, 1000, 1000000, UINT64_MAX};
    
    for (auto count : counts) {
        timestamp.access_count = count;
        EXPECT_EQ(timestamp.access_count, count);
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(ChunkTimestampTest, MaxInt64Timestamp) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = INT64_MAX;
    
    // Should not crash or overflow
    [[maybe_unused]] auto time_point = timestamp.getCreationTime();
    
    // This is far in the future, age will be negative when cast
    // (implementation-defined behavior, but should not crash)
    auto age = timestamp.getAge();
    (void)age;  // Just verify it doesn't crash
}

TEST_F(ChunkTimestampTest, NegativeTimestamp) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = -1000000000;  // Before epoch
    
    // Should not crash
    auto time_point = timestamp.getCreationTime();
    auto age = timestamp.getAge();
    (void)time_point;
    (void)age;
}

TEST_F(ChunkTimestampTest, AllFieldsMaxValues) {
    ChunkTimestamp timestamp;
    timestamp.creation_time_ns = INT64_MAX;
    timestamp.last_modified_ns = INT64_MAX;
    timestamp.last_accessed_ns = INT64_MAX;
    timestamp.access_count = UINT64_MAX;
    
    // Should not crash
    EXPECT_EQ(timestamp.creation_time_ns, INT64_MAX);
    EXPECT_EQ(timestamp.last_modified_ns, INT64_MAX);
    EXPECT_EQ(timestamp.last_accessed_ns, INT64_MAX);
    EXPECT_EQ(timestamp.access_count, UINT64_MAX);
}

TEST_F(ChunkTimestampTest, TimeProgressionSimulation) {
    ChunkTimestamp timestamp;
    
    // Simulate chunk lifecycle
    auto start_time = getCurrentTimeNs();
    timestamp.creation_time_ns = start_time;
    timestamp.last_modified_ns = start_time;
    timestamp.last_accessed_ns = start_time;
    timestamp.access_count = 0;
    
    // Wait and access
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_accessed_ns = getCurrentTimeNs();
    timestamp.access_count++;
    
    // Wait and access again
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_accessed_ns = getCurrentTimeNs();
    timestamp.access_count++;
    
    // Wait and modify
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timestamp.last_modified_ns = getCurrentTimeNs();
    
    // Verify state
    EXPECT_EQ(timestamp.access_count, 2);
    EXPECT_GT(timestamp.last_accessed_ns, start_time);
    EXPECT_GT(timestamp.last_modified_ns, start_time);
    EXPECT_EQ(timestamp.creation_time_ns, start_time);
    
    // Age should be at least 150ms
    auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - timestamp.getCreationTime()
    );
    EXPECT_GE(age_ms.count(), 150);
}

// ============================================================================
// Managed Chunk Tests
// ============================================================================

class ManagedChunkTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_coord_ = ChunkCoord(1, 2, 3);
    }

    // Helper function to create a test map
    std::unique_ptr<Bonxai::OccupancyMap> createTestMap() {
        Bonxai::Occupancy::OccupancyOptions opt_default;
        double resolution = 0.2;
        return std::make_unique<Bonxai::OccupancyMap>(resolution,opt_default);
    }

    ChunkCoord test_coord_;
};

// ============================================================================
// Construction Tests
// ============================================================================

TEST_F(ManagedChunkTest, ConstructionWithValidMap) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_EQ(chunk.getConstMap(), map_ptr);
    EXPECT_EQ(chunk.getChunkCoord(), test_coord_);
    EXPECT_FALSE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, ConstructionWithNullMap) {
    std::unique_ptr<Bonxai::OccupancyMap> null_map = nullptr;
    
    ManagedChunk chunk(std::move(null_map), test_coord_, false);
    
    EXPECT_EQ(chunk.getConstMap(), nullptr);
    EXPECT_FALSE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, ConstructionMarkedDirty) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, ConstructionMarkedClean) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
}


// ============================================================================
// Move Semantics Tests
// ============================================================================

TEST_F(ManagedChunkTest, MoveConstructor) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk1(std::move(map), test_coord_, true);
    ManagedChunk chunk2(std::move(chunk1));
    
    // chunk2 should have ownership
    EXPECT_EQ(chunk2.getConstMap(), map_ptr);
    EXPECT_EQ(chunk2.getChunkCoord(), test_coord_);
    EXPECT_TRUE(chunk2.isDirty());
    EXPECT_TRUE(chunk2.isMapValid());
    
    // chunk1 should be in moved-from state (map is nullptr)
    EXPECT_FALSE(chunk1.isMapValid());
}

TEST_F(ManagedChunkTest, MoveAssignment) {
    auto map1 = createTestMap();
    auto map2 = createTestMap();
    auto* map2_ptr = map2.get();
    
    ChunkCoord coord1(1, 1, 1);
    ChunkCoord coord2(2, 2, 2);
    
    ManagedChunk chunk1(std::move(map1), coord1, false);
    ManagedChunk chunk2(std::move(map2), coord2, true);
    
    // Move assign chunk2 to chunk1
    chunk1 = std::move(chunk2);
    
    // chunk1 should now have chunk2's data
    EXPECT_EQ(chunk1.getConstMap(), map2_ptr);
    EXPECT_EQ(chunk1.getChunkCoord(), coord2);
    EXPECT_TRUE(chunk1.isDirty());
    EXPECT_TRUE(chunk1.isMapValid());
    
    // chunk2 should be in moved-from state
    EXPECT_FALSE(chunk2.isMapValid());
}

TEST_F(ManagedChunkTest, MoveAssignmentSelfAssignment) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    // Self-assignment should be safe. The self-move is the point of the test,
    // so silence the warning that correctly flags it everywhere else.
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wself-move"
#endif
    chunk = std::move(chunk);
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif


    // State should be unchanged
    EXPECT_EQ(chunk.getConstMap(), map_ptr);
    EXPECT_EQ(chunk.getChunkCoord(), test_coord_);
    EXPECT_TRUE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
}

// ============================================================================
// Map Access Tests
// ============================================================================

TEST_F(ManagedChunkTest, GetConstMapReadOnly) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    const Bonxai::OccupancyMap* const_map = chunk.getConstMap();
    
    EXPECT_EQ(const_map, map_ptr);
    EXPECT_FALSE(chunk.isDirty()); // Should NOT mark dirty
}

TEST_F(ManagedChunkTest, GetMutableMapMarksDirty) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
    
    Bonxai::OccupancyMap* mutable_map = chunk.getMutableMap();
    
    EXPECT_NE(mutable_map, nullptr);
    EXPECT_TRUE(chunk.isDirty()); // Should mark dirty
}

TEST_F(ManagedChunkTest, MultipleGetMutableMapCalls) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    Bonxai::OccupancyMap* ptr1 = chunk.getMutableMap();
    Bonxai::OccupancyMap* ptr2 = chunk.getMutableMap();
    
    EXPECT_EQ(ptr1, map_ptr);
    EXPECT_EQ(ptr2, map_ptr);
    EXPECT_TRUE(chunk.isDirty());
}

// ============================================================================
// Dirty Tracking Tests
// ============================================================================

TEST_F(ManagedChunkTest, DirtyFlagInitiallyClean) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, DirtyFlagInitiallyDirty) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, MarkDirtyWhenClean) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
    
    chunk.markDirty();
    
    EXPECT_TRUE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, MarkDirtyWhenAlreadyDirty) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
    
    chunk.markDirty();
    
    EXPECT_TRUE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, MarkCleanWhenDirty) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
    
    chunk.markClean();
    
    EXPECT_FALSE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, MarkCleanWhenAlreadyClean) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
    
    chunk.markClean();
    
    EXPECT_FALSE(chunk.isDirty());
}

TEST_F(ManagedChunkTest, DirtyFlagToggling) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
    
    chunk.markDirty();
    EXPECT_TRUE(chunk.isDirty());
    
    chunk.markClean();
    EXPECT_FALSE(chunk.isDirty());
    
    chunk.markDirty();
    EXPECT_TRUE(chunk.isDirty());
}

// ============================================================================
// Coordinate Tests
// ============================================================================

TEST_F(ManagedChunkTest, GetChunkCoord) {
    auto map = createTestMap();
    ChunkCoord coord(5, 10, 15);
    
    ManagedChunk chunk(std::move(map), coord, false);
    
    ChunkCoord retrieved = chunk.getChunkCoord();
    
    EXPECT_EQ(retrieved.x, 5);
    EXPECT_EQ(retrieved.y, 10);
    EXPECT_EQ(retrieved.z, 15);
}

TEST_F(ManagedChunkTest, GetChunkCoordStr) {
    auto map = createTestMap();
    ChunkCoord coord(1, 2, 3);
    
    ManagedChunk chunk(std::move(map), coord, false);
    
    std::string coord_str = chunk.getChunkCoordStr();
    
    EXPECT_EQ(coord_str, "(1,2,3)");
}

TEST_F(ManagedChunkTest, GetChunkCoordStrNegativeCoords) {
    auto map = createTestMap();
    ChunkCoord coord(-5, -10, -15);
    
    ManagedChunk chunk(std::move(map), coord, false);
    
    std::string coord_str = chunk.getChunkCoordStr();
    
    EXPECT_EQ(coord_str, "(-5,-10,-15)");
}

// ============================================================================
// Ownership Transfer Tests
// ============================================================================

TEST_F(ManagedChunkTest, TransferMapOwnership) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
    
    auto transferred_map = chunk.transferMapOwnership();
    
    EXPECT_EQ(transferred_map.get(), map_ptr);
    EXPECT_FALSE(chunk.isDirty()); // Should be marked clean
    EXPECT_FALSE(chunk.isMapValid()); // Map should be null
    EXPECT_EQ(chunk.getConstMap(), nullptr);
}

TEST_F(ManagedChunkTest, TransferMapOwnershipFromNullMap) {
    std::unique_ptr<Bonxai::OccupancyMap> null_map = nullptr;
    
    ManagedChunk chunk(std::move(null_map), test_coord_, false);
    
    auto transferred_map = chunk.transferMapOwnership();
    
    EXPECT_EQ(transferred_map, nullptr);
    EXPECT_FALSE(chunk.isDirty());
    EXPECT_FALSE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, TransferMapOwnershipTwice) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    auto transferred1 = chunk.transferMapOwnership();
    EXPECT_NE(transferred1, nullptr);
    EXPECT_FALSE(chunk.isMapValid());
    
    // Second transfer should give null
    auto transferred2 = chunk.transferMapOwnership();
    EXPECT_EQ(transferred2, nullptr);
    EXPECT_FALSE(chunk.isMapValid());
}

// ============================================================================
// Map Validity Tests
// ============================================================================

TEST_F(ManagedChunkTest, IsMapValidWithValidMap) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_TRUE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, IsMapValidWithNullMap) {
    std::unique_ptr<Bonxai::OccupancyMap> null_map = nullptr;
    
    ManagedChunk chunk(std::move(null_map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, IsMapValidAfterTransfer) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_TRUE(chunk.isMapValid());
    
    auto mapptr = chunk.transferMapOwnership();
    
    EXPECT_FALSE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, DoesMoveLeaveOtherChunkWithInvalidMap) {
    auto map1 = createTestMap();
    auto map2 = createTestMap();

    ManagedChunk chunk1(std::move(map1), test_coord_, true);
    ManagedChunk chunk2(std::move(map2), test_coord_, false);

    // Initially, both are valid
    EXPECT_TRUE(chunk1.isMapValid());
    EXPECT_TRUE(chunk2.isMapValid());

    //Now Lets say Chunk2 gives up ownership of its map
    auto map2ptr = chunk2.transferMapOwnership();
    EXPECT_FALSE(chunk2.isMapValid());

    // Then, chunk1 is moved into chunk2
    chunk2 = std::move(chunk1);
    // Causing chunk2 to become valid
    EXPECT_TRUE(chunk2.isMapValid());
    EXPECT_TRUE(chunk2.isDirty());

    // And now, chunk 1 must be left in an invalid statue
    EXPECT_FALSE(chunk1.isMapValid());
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(ManagedChunkTest, ConcurrentDirtyFlagAccess) {
    auto map = createTestMap();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    std::atomic<int> mark_dirty_count{0};
    std::atomic<int> mark_clean_count{0};
    std::atomic<int> check_count{0};
    
    const int num_threads = 10;
    const int operations_per_thread = 1000;
    
    auto worker = [&](int id) {
        for (int i = 0; i < operations_per_thread; ++i) {
            if (id % 3 == 0) {
                chunk.markDirty();
                mark_dirty_count++;
            } else if (id % 3 == 1) {
                chunk.markClean();
                mark_clean_count++;
            } else {
                bool is_dirty = chunk.isDirty();
                (void)is_dirty; // Use the value
                check_count++;
            }
        }
    };
    
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker, i);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // Test that all operations completed without crashes
    EXPECT_GT(mark_dirty_count.load(), 0);
    EXPECT_GT(mark_clean_count.load(), 0);
    EXPECT_GT(check_count.load(), 0);
    
    // Final state should be consistent (either clean or dirty)
    bool final_dirty = chunk.isDirty();
    EXPECT_TRUE(final_dirty || !final_dirty); // Just check it's a valid bool
}

TEST_F(ManagedChunkTest, ConcurrentGetMutableMap) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    const int num_threads = 10;
    std::vector<std::thread> threads;
    std::atomic<int> access_count{0};
    
    auto worker = [&]() {
        for (int i = 0; i < 100; ++i) {
            Bonxai::OccupancyMap* ptr = chunk.getMutableMap();
            EXPECT_EQ(ptr, map_ptr);
            access_count++;
        }
    };
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    EXPECT_EQ(access_count.load(), num_threads * 100);
    EXPECT_TRUE(chunk.isDirty()); // Should be marked dirty
}

TEST_F(ManagedChunkTest, ConcurrentReadAccess) {
    auto map = createTestMap();
    auto* map_ptr = map.get();
    
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    const int num_threads = 10;
    std::vector<std::thread> threads;
    std::atomic<int> read_count{0};
    
    auto worker = [&]() {
        for (int i = 0; i < 100; ++i) {
            const Bonxai::OccupancyMap* ptr = chunk.getConstMap();
            EXPECT_EQ(ptr, map_ptr);
            read_count++;
        }
    };
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(worker);
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    EXPECT_EQ(read_count.load(), num_threads * 100);
    EXPECT_FALSE(chunk.isDirty()); // Should NOT be marked dirty
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(ManagedChunkTest, TypicalWorkflow) {
    // 1. Create chunk (clean from disk)
    auto map = createTestMap();
    ManagedChunk chunk(std::move(map), test_coord_, false);
    
    EXPECT_FALSE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
    
    // 2. Read-only access
    const Bonxai::OccupancyMap* const_map = chunk.getConstMap();
    EXPECT_NE(const_map, nullptr);
    EXPECT_FALSE(chunk.isDirty());
    
    // 3. Modify the chunk
    Bonxai::OccupancyMap* mutable_map = chunk.getMutableMap();
    EXPECT_NE(mutable_map, nullptr);
    EXPECT_TRUE(chunk.isDirty());
    
    // 4. Save (transfer ownership)
    auto map_for_save = chunk.transferMapOwnership();
    EXPECT_NE(map_for_save, nullptr);
    EXPECT_FALSE(chunk.isDirty());
    EXPECT_FALSE(chunk.isMapValid());
}

TEST_F(ManagedChunkTest, NewChunkWorkflow) {
    // 1. Create new chunk (dirty)
    auto map = createTestMap();
    ManagedChunk chunk(std::move(map), test_coord_, true);
    
    EXPECT_TRUE(chunk.isDirty());
    EXPECT_TRUE(chunk.isMapValid());
    
    // 2. Modify
    Bonxai::OccupancyMap* mutable_map = chunk.getMutableMap();
    auto &accessor = mutable_map->getAccessor();
    Bonxai::CoordT coord;
    coord.x = 0; coord.y = 0 , coord.z = 10;
    Bonxai::Occupancy::CellOcc occ;
    accessor.setValue(coord,occ);
    EXPECT_TRUE(chunk.isDirty());
    
    // 3. Save
    auto map_for_save = chunk.transferMapOwnership();
    EXPECT_NE(map_for_save, nullptr);
    EXPECT_FALSE(chunk.isDirty());
    EXPECT_FALSE(chunk.isMapValid());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(ManagedChunkTest, GetMutableMapOnNullMap) {
    std::unique_ptr<Bonxai::OccupancyMap> null_map = nullptr;
    
    ManagedChunk chunk(std::move(null_map), test_coord_, false);
    
    Bonxai::OccupancyMap* ptr = chunk.getMutableMap();
    
    EXPECT_EQ(ptr, nullptr);
    EXPECT_TRUE(chunk.isDirty()); // Still marks dirty even if map is null
}

TEST_F(ManagedChunkTest, GetConstMapOnNullMap) {
    std::unique_ptr<Bonxai::OccupancyMap> null_map = nullptr;
    
    ManagedChunk chunk(std::move(null_map), test_coord_, false);
    
    const Bonxai::OccupancyMap* ptr = chunk.getConstMap();
    
    EXPECT_EQ(ptr, nullptr);
}

TEST_F(ManagedChunkTest, ZeroCoordinates) {
    auto map = createTestMap();
    ChunkCoord zero_coord(0, 0, 0);
    
    ManagedChunk chunk(std::move(map), zero_coord, false);
    
    EXPECT_EQ(chunk.getChunkCoord(), zero_coord);
    EXPECT_EQ(chunk.getChunkCoordStr(), "(0,0,0)");
}

TEST_F(ManagedChunkTest, LargeCoordinates) {
    auto map = createTestMap();
    ChunkCoord large_coord(1000000, -1000000, 999999);
    
    ManagedChunk chunk(std::move(map), large_coord, false);
    
    ChunkCoord retrieved = chunk.getChunkCoord();
    EXPECT_EQ(retrieved.x, 1000000);
    EXPECT_EQ(retrieved.y, -1000000);
    EXPECT_EQ(retrieved.z, 999999);
}


// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}