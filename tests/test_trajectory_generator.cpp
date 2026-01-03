#include <gtest/gtest.h>
#include <cmath>
#include "the_cruiser/implementations/catmull_rom_generator.hpp"

// --- HELPER MACROS FOR TESTS ---
// Floating point equality with tolerance
#define EXPECT_POINT_NEAR(pt, _x, _y)     EXPECT_NEAR(pt.x, _x, 1e-3);     EXPECT_NEAR(pt.y, _y, 1e-3)

class TrajectoryGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // setup: Standard tuning
        generator_ = std::make_shared<the_cruiser::implementations::CatmullRomGenerator>();
    }
    
    std::shared_ptr<the_cruiser::implementations::CatmullRomGenerator> generator_;
};

// ------------------------------------------------------------------
// GROUP 1: INPUT VALIDATION (Anti-Crash)
// ------------------------------------------------------------------

TEST_F(TrajectoryGeneratorTest, RejectsEmptyInput) {
    std::vector<geometry_msgs::msg::Point> empty;
    auto result = generator_->smooth(empty);
    EXPECT_TRUE(result.empty()) << "Should return empty path for empty input";
}

TEST_F(TrajectoryGeneratorTest, HandlesSmallWaypoints) {
    // < 4 points: Falls back to linear interpolation instead of Catmull-Rom
    std::vector<geometry_msgs::msg::Point> points(3); 
    points[0].x = 0; points[0].y = 0;
    points[1].x = 1; points[1].y = 1;
    points[2].x = 2; points[2].y = 0;
    
    auto result = generator_->smooth(points);
    
    EXPECT_FALSE(result.empty()) << "Should handle < 4 points with linear fallback";
    EXPECT_NEAR(result.front().v, 0.0, 1e-3) << "Should start at zero velocity";
    EXPECT_NEAR(result.back().v, 0.0, 1e-3) << "Should end at zero velocity";
}

TEST_F(TrajectoryGeneratorTest, HandlesDuplicatePoints) {
    // CRITICAL: Duplicate points create Distance = 0.
    // Naive time calc (t = dist/v) would cause Divide-By-Zero.
    std::vector<geometry_msgs::msg::Point> waypoints;
    auto add = [&](double x, double y) { geometry_msgs::msg::Point p; p.x=x; p.y=y; waypoints.push_back(p); };
    
    add(0,0); add(1,0); add(1,0); add(2,0); // Duplicate (1,0)
    
    auto result = generator_->smooth(waypoints);
    
    ASSERT_FALSE(result.empty());
    // Ensure no NaN or Inf values in output
    for(const auto& pt : result) {
        EXPECT_FALSE(std::isnan(pt.v));
        EXPECT_FALSE(std::isinf(pt.t));
    }
}

// ------------------------------------------------------------------
// GROUP 2: PHYSICS COMPLIANCE (Safety)
// ------------------------------------------------------------------

TEST_F(TrajectoryGeneratorTest, RespectsVelocityLimits) {
    // Create a long straight path that would allow high speed
    std::vector<geometry_msgs::msg::Point> waypoints;
    auto add = [&](double x, double y) { geometry_msgs::msg::Point p; p.x=x; p.y=y; waypoints.push_back(p); };
    
    add(0,0); add(10,0); add(20,0); add(30,0);
    
    auto result = generator_->smooth(waypoints);
    
    // Check every single point
    double max_allowed_v = 1.0; // From constructor
    for(const auto& pt : result) {
        EXPECT_LE(pt.v, max_allowed_v + 1e-3) << "Velocity exceeded limit at t=" << pt.t;
    }
}

TEST_F(TrajectoryGeneratorTest, StartsAndStopsAtZeroVelocity) {
    std::vector<geometry_msgs::msg::Point> waypoints;
    auto add = [&](double x, double y) { geometry_msgs::msg::Point p; p.x=x; p.y=y; waypoints.push_back(p); };
    
    add(0,0); add(2,0); add(4,0); add(6,0);
    
    auto result = generator_->smooth(waypoints);
    ASSERT_FALSE(result.empty());
    
    // Start velocity must be 0
    EXPECT_NEAR(result.front().v, 0.0, 1e-3);
    
    // End velocity must be 0
    EXPECT_NEAR(result.back().v, 0.0, 1e-3);
}

// ------------------------------------------------------------------
// GROUP 3: SHAPE ACCURACY
// ------------------------------------------------------------------

TEST_F(TrajectoryGeneratorTest, InterpolatesThroughWaypoints) {
    std::vector<geometry_msgs::msg::Point> waypoints;
    auto add = [&](double x, double y) { geometry_msgs::msg::Point p; p.x=x; p.y=y; waypoints.push_back(p); };
    
    add(0,0); add(1,1); add(2,0); add(3,1);
    
    auto result = generator_->smooth(waypoints);
    
    // The smoother should generate points that are close to the original waypoints
    // It might not hit them EXACTLY due to density, but we check if the end points match
    EXPECT_POINT_NEAR(result.front(), 0.0, 0.0);
    EXPECT_POINT_NEAR(result.back(), 3.0, 1.0);
}
