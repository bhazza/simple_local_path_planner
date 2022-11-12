#include "simple_local_path_planner/simple_local_path_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_local_path_planner::SimpleLocalPathPlanner, nav_core::BaseLocalPlanner)

namespace simple_local_path_planner{

SimpleLocalPathPlanner::SimpleLocalPathPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

SimpleLocalPathPlanner::SimpleLocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

SimpleLocalPathPlanner::~SimpleLocalPathPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void SimpleLocalPathPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
}

bool SimpleLocalPathPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool SimpleLocalPathPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool SimpleLocalPathPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}
}