#include "simple_local_path_planner/simple_local_path_planner.h"

#include <angles/angles.h>
#include <ros/console.h> // Debug messages
#include <tf2/utils.h>

namespace simple_local_path_planner_ros{

SimpleLocalPathPlanner::SimpleLocalPathPlanner() : 
    m_current_target_index(0),
    m_motion_state(MotionState::STOPPED),
    m_config(){}


SimpleLocalPathPlanner::~SimpleLocalPathPlanner() {}

void SimpleLocalPathPlanner::reset()
{
    m_plan.clear();
    m_current_target_index = m_config.waypoint_step_size; // Note, this index is checked if out of range in getTargetPose()
}

void SimpleLocalPathPlanner::setConfig(const Config& config)
{
    m_config = config;
}

void SimpleLocalPathPlanner::setRobotPose(const geometry_msgs::PoseStamped& robot_pose)
{
    ROS_DEBUG("Setting robot pose");
    m_robot_pose = robot_pose;
}

void SimpleLocalPathPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_DEBUG("Setting plan");
    // Plan has been changed, start again
    reset();
    m_plan = plan;
}

const geometry_msgs::PoseStamped& SimpleLocalPathPlanner::getRobotPose() const
{
    return m_robot_pose;
}

const geometry_msgs::PoseStamped& SimpleLocalPathPlanner::getTargetPose() const
{
    if (m_current_target_index < m_plan.size())
    {
        return m_plan.at(m_current_target_index);
    }

    return m_plan.back();
}

const geometry_msgs::PoseStamped& SimpleLocalPathPlanner::getGoalPose() const
{
    return m_plan.back();
}

geometry_msgs::Twist SimpleLocalPathPlanner::getRotateToGoal()
{
    m_motion_state = MotionState::ROTATE;
    const double angular_delta = getAngularDelta(getRobotPose(), getGoalPose());
    return getRotationalTwist(angular_delta);
}

// Todo: Consider splitting this function out, and also making it more generic so that it can be called by the getRotateToGoal() function above
geometry_msgs::Twist SimpleLocalPathPlanner::getNextCmdVel()
{
    geometry_msgs::Twist cmd_vel = zeroTwist();

    // Todo: Check if these can change during execution, and if so should we take copies instead of reference?
    const geometry_msgs::PoseStamped& robot_pose = getRobotPose();
    const geometry_msgs::PoseStamped& target_pose = getTargetPose();

    // 1. First check if we need to rotate in place.

    // Get bearing from current robot position to next waypoint on transformed plan
    const geometry_msgs::Point& target_position = target_pose.pose.position;
    const double target_yaw = atan2(target_position.y - robot_pose.pose.position.y, target_position.x - robot_pose.pose.position.x);

    // Get the angle delta between current robot yaw and the target bearing
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double angular_delta =  angles::shortest_angular_distance(robot_yaw, target_yaw);
    ROS_DEBUG("Angular Delta: %f, Threshold %f", angular_delta, toRadians(m_config.angular_tolerance_degrees));

    if (fabs(angular_delta) > toRadians(m_config.angular_tolerance_degrees))
    {
        m_motion_state = MotionState::ROTATE;
        cmd_vel = getRotationalTwist(angular_delta);
        return cmd_vel;
    }
    else if (m_motion_state == MotionState::ROTATE) 
    {
        // Target achieved, stop rotating
        cmd_vel = getStoppedCmdVel();
        m_motion_state = MotionState::STOPPED;
        return cmd_vel;
    }

    // 2. Traverse to next target position
    const double linear_delta = getLinearDelta(robot_pose, target_pose);
    ROS_DEBUG("Linear Delta: %f", linear_delta);
    if (fabs(linear_delta) > m_config.linear_tolerance)
    {
        m_motion_state = MotionState::TRAVERSE;
        cmd_vel = getLinearTwist(linear_delta);
        return cmd_vel;
    }

    // If we've reached the target pose, increment to next one.
    m_current_target_index += m_config.waypoint_step_size;

    return cmd_vel;
}

geometry_msgs::Twist SimpleLocalPathPlanner::getStoppedCmdVel()
{
    m_motion_state = MotionState::STOPPED;
    return zeroTwist();
}

bool SimpleLocalPathPlanner::planAvailable() const
{
    return !m_plan.empty();
}

bool SimpleLocalPathPlanner::goalReached() const 
{
    if (!planAvailable())
    {
        return false;
    }
    ROS_DEBUG("Checking if goal reached");
    return isAtGoalPosition() && isAtGoalOrientation();
}

bool SimpleLocalPathPlanner::isAtGoalPosition() const
{
    if (!planAvailable())
    {
        return false;
    }
    ROS_DEBUG("Checked if robot at goal position");
    return getLinearDelta(getRobotPose(), getGoalPose()) < m_config.linear_tolerance;
}

bool SimpleLocalPathPlanner::isAtGoalOrientation() const
{
    if (!planAvailable())
    {
        return false;
    }
    ROS_DEBUG("Checked if robot at goal orientation");
    const double abs_angular_delta = fabs(getAngularDelta(getRobotPose(), getGoalPose()));
    const double angular_tolerance = toRadians(m_config.angular_tolerance_degrees);
    return abs_angular_delta < angular_tolerance;
}

geometry_msgs::Twist SimpleLocalPathPlanner::zeroTwist() const
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    return cmd_vel;
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
    // velocity proportional to delta, and capped by max and min config
    double velocity = std::min(fabs(angular_delta), toRadians(m_config.max_angular_velocity_degrees)); // enforce max velocity
    velocity = std::max(velocity, toRadians(m_config.min_angular_velocity_degrees)); // enforce min velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = angular_delta > 0 ? velocity : -1.0 * velocity; // Set direction
    return cmd_vel;

}

geometry_msgs::Twist SimpleLocalPathPlanner::getLinearTwist(const double& linear_delta) const
{
    // velocity proportional to delta, and capped by max and min config
    double velocity = std::min(fabs(linear_delta), m_config.max_linear_velocity); // enforce max velocity
    velocity = std::max(velocity, m_config.min_linear_velocity); // enforce min velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    return cmd_vel;
}


double SimpleLocalPathPlanner::toRadians(const double& degrees) const
{
    return degrees * (M_PI/180.0);
}


}
