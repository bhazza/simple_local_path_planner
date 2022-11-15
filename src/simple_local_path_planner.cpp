#include "simple_local_path_planner/simple_local_path_planner.h"
#include <pluginlib/class_list_macros.h>

#include <tf2/utils.h>
//TODO sort
#include <base_local_planner/goal_functions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(simple_local_path_planner::SimpleLocalPathPlanner, nav_core::BaseLocalPlanner)

namespace simple_local_path_planner{

SimpleLocalPathPlanner::SimpleLocalPathPlanner() : 
    m_costmap_ros(NULL), 
    m_tf(NULL), 
    m_target_waypoint_index(0),
    m_initialised(false),
    m_goal_reached(false),
    m_rotating(false),
    m_config() {}

SimpleLocalPathPlanner::SimpleLocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : m_costmap_ros(NULL), m_tf(NULL), m_initialised(false)
{
    initialize(name, tf, costmap_ros);
}

SimpleLocalPathPlanner::~SimpleLocalPathPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void SimpleLocalPathPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!m_initialised)
    {
        m_tf = tf;
        m_costmap_ros = costmap_ros;
        m_initialised = true;
    }
    else
    {
        ROS_WARN("This planner has already been initialized");
    }
}

bool SimpleLocalPathPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

   
    // Save new plan
    m_global_plan.clear();
    m_global_plan = orig_global_plan;
    m_goal_reached = false;

    // New goal so reset index 
    m_target_waypoint_index = m_config.goal_step;

    ROS_DEBUG("Plan Updated");
    return true;
}

bool SimpleLocalPathPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    // Zero twist message
    cmd_vel = zeroTwist();

    //if the global plan passed in is empty... we won't do anything
    if(m_global_plan.empty())
        return false;


    // Get pose of the robot
    geometry_msgs::PoseStamped robot_pose;
    if (!m_costmap_ros->getRobotPose(robot_pose)) {
        ROS_ERROR("Could not get global robot pose");
        return false;
    }

    // Transform the provided plan into same frame as the robot
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    const std::string globalFrame = m_costmap_ros->getGlobalFrameID();
    transformPosesToFrame(m_global_plan, globalFrame, transformed_plan);

    // Check to see if we're at the goal. If we are, do final rotation
    const geometry_msgs::PoseStamped& goal_pose = transformed_plan.back();
    if (getLinearDelta(robot_pose, goal_pose) < m_config.linear_tolerance)
    {
        ROS_DEBUG("Linear Goal Reached");
        const double angular_delta = getAngularDelta(robot_pose, goal_pose);
        if (angular_delta < m_config.angular_tolerance)
        {
            ROS_DEBUG("Goal Reached");
            cmd_vel = zeroTwist();
            m_goal_reached = true;
            return true;
        }
        else
        {
            ROS_DEBUG("Rotating to Goal");
            m_rotating = true;
            cmd_vel = getRotationalTwist(angular_delta);
            return true;
        }
    }

    //Get next target waypoint, making sure not to exceed the size of the vector
    const geometry_msgs::PoseStamped& target_pose = m_target_waypoint_index < transformed_plan.size() ? transformed_plan.at(m_target_waypoint_index) : transformed_plan.back();

    //ROTATE IN PLACE
    // Get bearing from current robot position to next waypoint on transformed plan
    const geometry_msgs::Point& target_position = target_pose.pose.position;
    const double target_bearing = atan2(target_position.y - robot_pose.pose.position.y, target_position.x - robot_pose.pose.position.x);
    
    double current_robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double angular_delta =  angles::shortest_angular_distance(current_robot_yaw, target_bearing);
    ROS_DEBUG("Angular Delta: %f", angular_delta);
    if (fabs(angular_delta) > m_config.angular_tolerance)
    {
        m_rotating = true;
        cmd_vel = getRotationalTwist(angular_delta);
        return true;
    }
    else if (m_rotating) 
    {
        // Target achieved, stop rotating
        cmd_vel = zeroTwist();
        m_rotating = false;
        return true;
    }

    // TRAVERSE TO NEXT GOAL
    const double linear_delta = getLinearDelta(robot_pose, target_pose);
    ROS_DEBUG("Linear Delta: %f", linear_delta);
    if (fabs(linear_delta) > m_config.linear_tolerance)
    {
        cmd_vel = getLinearTwist(linear_delta);
        return true;
    }

    // Increment transfor
    m_target_waypoint_index += m_config.goal_step;

    return true;
}

bool SimpleLocalPathPlanner::isGoalReached()
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return m_goal_reached;
}

bool SimpleLocalPathPlanner::transformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& target_frame, std::vector<geometry_msgs::PoseStamped>& transformed_poses) const
{
    
    const std::string& original_frame = poses.front().header.frame_id;
    const ros::Time& pose_time = poses.front().header.stamp;

    // TODO put this in a try
    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = m_tf->lookupTransform(target_frame, ros::Time(), original_frame, pose_time,
                                                                                  original_frame, ros::Duration(0.5));

    // transform plan
    transformed_poses.clear();
    for (const auto& originalPose : poses)
    {   
        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(originalPose, transformed_pose, plan_to_global_transform);
        transformed_poses.push_back(transformed_pose);
    }

    return true;
}

double SimpleLocalPathPlanner::getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
{
    const double fromYaw = tf2::getYaw(from.pose.orientation);
    const double toYaw = tf2::getYaw(to.pose.orientation);
    return angles::shortest_angular_distance(fromYaw, toYaw);
}

double SimpleLocalPathPlanner::getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
{
    return std::sqrt(std::pow(from.pose.position.x - to.pose.position.x, 2) + std::pow(from.pose.position.y - to.pose.position.y, 2));
}

geometry_msgs::Twist SimpleLocalPathPlanner::getRotationalTwist(const double& angular_delta) const
{
    const double velocity = std::min(m_config.max_angular_velocity, fabs(angular_delta) * m_config.max_angular_velocity * 10.0);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = angular_delta > 0 ? velocity : -1.0 * velocity;
    return cmd_vel;

}

geometry_msgs::Twist SimpleLocalPathPlanner::getLinearTwist(const double& linear_delta) const
{
    const double velocity = fabs(linear_delta) * m_config.max_linear_velocity;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    return cmd_vel;
}

geometry_msgs::Twist SimpleLocalPathPlanner::zeroTwist() const
{
    geometry_msgs::Twist msg;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    return msg;
}

}
