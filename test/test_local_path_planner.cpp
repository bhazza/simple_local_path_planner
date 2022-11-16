#include "simple_local_path_planner/simple_local_path_planner.h"
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

// Helper function to quickly generate poses
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
    plan.push_back(generate2DPoseStamped(0,1,0));
    plan.push_back(generate2DPoseStamped(1,1,0));
    plan.push_back(generate2DPoseStamped(2,1,0));
    plan.push_back(generate2DPoseStamped(2,2,180));
    return plan;
}

const geometry_msgs::PoseStamped robot_starting_pose = generate2DPoseStamped(0,0,0);

// Declare a test
TEST(TestSuite, testCase1)
{
    const geometry_msgs::PoseStamped robot_starting_pose = generate2DPoseStamped(0,0,0);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
//   ros::init(argc, argv, "tester");
//   ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}