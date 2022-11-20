#include "simple_local_path_planner/simple_local_path_planner.h"
#include "simple_local_path_planner/simple_local_path_planner_ros.h"
#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

double toRadians(const double& degrees)
{
    return degrees * (M_PI/180.0);
}

double toDegrees(const double& radians)
{
    return radians * (180.0/M_PI);
}

geometry_msgs::PoseStamped generate2DPoseStamped(const double& x, const double& y, const double& yaw_degrees)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0,0,toRadians(yaw_degrees));
    tf::quaternionTFToMsg(q , pose.pose.orientation);
    return pose;
}

std::vector<geometry_msgs::PoseStamped> generatePlan()
{
    // Todo draw ASCII of plan
    std::vector<geometry_msgs::PoseStamped> plan;
    plan.push_back(generate2DPoseStamped(0,0,0));
    plan.push_back(generate2DPoseStamped(0,1,0));
    plan.push_back(generate2DPoseStamped(1,1,179));
    return plan;
}

simple_local_path_planner_ros::Config generateConfig()
{
    simple_local_path_planner_ros::Config config;
    // Default step size is 10, but we're sending a short, low resolution plan for testing.
    config.waypoint_step_size = 1;
    return config;
}

simple_local_path_planner_ros::SimpleLocalPathPlanner getSimpleLocalPathPlanner()
{
    simple_local_path_planner_ros::SimpleLocalPathPlanner slpp;
    slpp.setConfig(generateConfig());
    slpp.setPlan(generatePlan());
    slpp.setRobotPose(generate2DPoseStamped(0,0,0));
    return slpp;
}

// Friend class to give access to private member functions and variables
namespace simple_local_path_planner_ros{
class SimpleLocalPathPlannerTester
{
public:
    SimpleLocalPathPlannerTester()
    {
        m_slpp.setConfig(generateConfig());
        m_slpp.setPlan(generatePlan());
        m_slpp.setRobotPose(generate2DPoseStamped(0,0,0));
    }
    simple_local_path_planner_ros::SimpleLocalPathPlanner& getPlanner()
    {
        return m_slpp;
    }

    // Getting access to the private member functions - would definitely just be easier to implement these as util functions instead, but I guess this way I get to demonstrate friend classes
    double getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
    {
        return m_slpp.getAngularDelta(from, to);
    }

    double getAbsLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
    {
        return m_slpp.getAbsLinearDelta(from, to);
    }

    geometry_msgs::Twist getRotationalTwist(const double& angular_delta) const
    {
        return m_slpp.getRotationalTwist(angular_delta);
    }

    geometry_msgs::Twist getLinearTwist(const double& linear_delta) const
    {
        return m_slpp.getLinearTwist(linear_delta);
    }

private:
    simple_local_path_planner_ros::SimpleLocalPathPlanner m_slpp;

};
}

using namespace simple_local_path_planner_ros;

TEST(TestSuite, testGetters)
{
    SimpleLocalPathPlannerTester slppt;
    SimpleLocalPathPlanner slpp = slppt.getPlanner();
    // Test getters
    ASSERT_EQ(slpp.getRobotPose(), generate2DPoseStamped(0,0,0));
    ASSERT_EQ(slpp.getTargetPose(), generate2DPoseStamped(0,1,0));
    ASSERT_EQ(slpp.getGoalPose(), generate2DPoseStamped(1,1,179));
}

TEST(TestSuite, getAngularDelta)
{
    SimpleLocalPathPlannerTester slppt;

    // Angular deltas should return shortest distance between two angles (note x,y values don't matter)
    ASSERT_DOUBLE_EQ(slppt.getAngularDelta(generate2DPoseStamped(0,0,0), generate2DPoseStamped(1,2,90)), toRadians(90.0)); // | -> __|
    ASSERT_DOUBLE_EQ(slppt.getAngularDelta(generate2DPoseStamped(0,0,90), generate2DPoseStamped(1,2,0)), toRadians(-90.0)); // __| -> |
    ASSERT_DOUBLE_EQ(slppt.getAngularDelta(generate2DPoseStamped(9,10,45), generate2DPoseStamped(5,2,-45)),  toRadians(-90.0)); //  \ -> /
    ASSERT_DOUBLE_EQ(slppt.getAngularDelta(generate2DPoseStamped(0,0,-45), generate2DPoseStamped(1,2,45)), toRadians(90.0)); //   / -> 
}

TEST(TestSuite, getAbsLinearDelta)
{
    SimpleLocalPathPlannerTester slppt;

    ASSERT_DOUBLE_EQ(slppt.getAbsLinearDelta(generate2DPoseStamped(0,1,0), generate2DPoseStamped(0,1,90)), 0.0); 
    ASSERT_DOUBLE_EQ(slppt.getAbsLinearDelta(generate2DPoseStamped(0,0,90), generate2DPoseStamped(1,1,0)), std::sqrt(2)); 
}

TEST(TestSuite, getRotationalTwist)
{
    SimpleLocalPathPlannerTester slppt;
    const Config config = generateConfig();

    // Check max (40)
    ASSERT_TRUE(fabs(slppt.getRotationalTwist(toRadians(90.0)).angular.z) <= toRadians(config.max_angular_velocity_degrees)); 
    // Check min (5)
    ASSERT_TRUE(fabs(slppt.getRotationalTwist(toRadians(2.0)).angular.z) >= toRadians(config.min_angular_velocity_degrees)); 
    // Check values
    ASSERT_DOUBLE_EQ(slppt.getRotationalTwist(toRadians(20.0)).angular.z, toRadians(20.0));
    ASSERT_DOUBLE_EQ(slppt.getRotationalTwist(toRadians(-20.0)).angular.z, toRadians(-20.0)); 

}

TEST(TestSuite, getLinearTwist)
{
    SimpleLocalPathPlannerTester slppt;
    const Config config = generateConfig();

    // Check max (0.5)
    ASSERT_TRUE(slppt.getLinearTwist(1.0).linear.x <= config.max_linear_velocity); 
    // Check min (0.05)
    ASSERT_TRUE(slppt.getLinearTwist(0.02).linear.x >= config.min_linear_velocity); 
    // Check values
    ASSERT_DOUBLE_EQ(slppt.getLinearTwist(0.2).linear.x, 0.2); 
    ASSERT_DOUBLE_EQ(slppt.getLinearTwist(-0.2).linear.x, 0.2); 
}

TEST(TestSuite, traversePlan)
{
    SimpleLocalPathPlannerTester slppt;
    SimpleLocalPathPlanner slpp = slppt.getPlanner();
    const Config config = generateConfig();

    // The following simulates the sequence of events following the plan
    // As defined above, the plan poses are (x,y,yaw): (0,0,0) -> (0,1,0) -> (1,1,179)

    // should rotate in place towards (0,1,0) from starting pose of (0,0,0)
    geometry_msgs::Twist cmd_vel = slpp.getNextCmdVel();
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, toRadians(config.max_angular_velocity_degrees));
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    slpp.setRobotPose(generate2DPoseStamped(0,0,90)); // simulate robot rotating towards (0,1,0);

    // Robot should stop rotating on first call, and then move forwards toward (0,1,0) with no rotating on next call
    cmd_vel = slpp.getNextCmdVel();
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    cmd_vel = slpp.getNextCmdVel(); 
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, config.max_linear_velocity);
    slpp.setRobotPose(generate2DPoseStamped(0,1,90)); // simulate robot arriving within linear tolerance of target pose (0,1,0);

    // Robot should stop on first call, then rotate -90 (clockwise) in place towards (1,1,0) on next call
    cmd_vel = slpp.getNextCmdVel();
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    cmd_vel = slpp.getNextCmdVel(); 
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, -1 * toRadians(config.max_angular_velocity_degrees));
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    slpp.setRobotPose(generate2DPoseStamped(0,1,0)); // simulate robot arriving at target pose (0,1,0);

    // Robot should stop on first call, then move forwards to towards (1,1,0) on next call
    cmd_vel = slpp.getNextCmdVel();
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    cmd_vel = slpp.getNextCmdVel(); 
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, config.max_linear_velocity);
    slpp.setRobotPose(generate2DPoseStamped(1,1,0)); // simulate robot arriving at target pose (1,1,0);

    // Now we're at the goal position, robot should rotate to final goal yaw of 179 degrees
    cmd_vel = slpp.getNextCmdVel(); 
    ASSERT_DOUBLE_EQ(cmd_vel.angular.z, toRadians(config.max_angular_velocity_degrees));
    ASSERT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
    slpp.setRobotPose(generate2DPoseStamped(1,1,179)); // simulate robot arriving at goal pose (1,1,179);


    ASSERT_TRUE(slpp.goalReached());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
//   ros::init(argc, argv, "tester");
//   ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}