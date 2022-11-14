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
    m_costmapROS(NULL), 
    m_tf(NULL), 
    m_currentGoalIndex(0),
    m_initialised(false),
    m_goalReached(false),
    m_rotating(false),
    m_config() {}

SimpleLocalPathPlanner::SimpleLocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : m_costmapROS(NULL), m_tf(NULL), m_initialised(false)
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
        m_costmapROS = costmap_ros;
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
    m_globalPlan.clear();
    m_globalPlan = orig_global_plan;
    m_goalReached = false;

    // New goal so reset index
    // m_currentGoalIndex = m_globalPlan.size() - 1;
    m_currentGoalIndex = 10;

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
    cmd_vel = ZeroTwist();

    //if the global plan passed in is empty... we won't do anything
    if(m_globalPlan.empty())
        return false;


    // Get global pose of the robot
    geometry_msgs::PoseStamped globalRobotPose;
    if (!m_costmapROS->getRobotPose(globalRobotPose)) {
        ROS_ERROR("Could not get global robot pose");
        return false;
    }

    // Transform the provided plan into same frame as the robot
    std::vector<geometry_msgs::PoseStamped> transformedPlan;
    const std::string globalFrame = m_costmapROS->getGlobalFrameID();
    TransformPosesToFrame(m_globalPlan, globalFrame, transformedPlan);

    // Check to see if we're at the goal. If we are, do final rotation
    const geometry_msgs::PoseStamped& goalPose = transformedPlan.back();
    if (getLinearDelta(globalRobotPose, goalPose) < m_config.linearTolerance)
    {
        const double angularDelta = getAngularDelta(globalRobotPose, goalPose);
        if (angularDelta < m_config.angularTolerance)
        {
            cmd_vel = ZeroTwist();
            m_goalReached = true;
            return true;
        }
        else
        {
            m_rotating = true;
            cmd_vel = getRotationalTwist(angularDelta);
            return true;
        }
    }

    //ROTATE IN PLACE
    // Get bearing from current robot position to next waypoint on transformed plan
    const double targetBearing = atan2(
        (transformedPlan.at(m_currentGoalIndex).pose.position.y - globalRobotPose.pose.position.y),
            transformedPlan.at(m_currentGoalIndex).pose.position.x - globalRobotPose.pose.position.x);
    
    double currentRobotYaw = tf2::getYaw(globalRobotPose.pose.orientation);
    const double angularDelta =  angles::shortest_angular_distance(currentRobotYaw, targetBearing);
    // const double angularDelta = getAngularDelta(globalRobotPose.pose.orientation, targetYaw);
    ROS_DEBUG("Angular Delta: %f", angularDelta);
    if (fabs(angularDelta) > m_config.angularTolerance)
    {
        m_rotating = true;
        cmd_vel = getRotationalTwist(angularDelta);
        return true;
    }
    else if (m_rotating) 
    {
        // Target achieved, stop rotating
        cmd_vel = ZeroTwist();
        m_rotating = false;
        return true;
    }

    // TRAVERSE TO NEXT GOAL
    const double linearDelta = getLinearDelta(globalRobotPose, transformedPlan.at(m_currentGoalIndex));
    ROS_DEBUG("Linear Delta: %f", linearDelta);
    if (fabs(linearDelta) > m_config.linearTolerance)
    {
        cmd_vel = getLinearTwist(linearDelta);
        return true;
    }

    // INCREMENT INDEX - TODO this is dodgy, fix.
    m_currentGoalIndex = std::min ( m_currentGoalIndex + 10, transformedPlan.size() - 1);
    ROS_DEBUG("Plan Step Index: : %d", (unsigned int)m_currentGoalIndex);

    return true;
}

bool SimpleLocalPathPlanner::isGoalReached()
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    // return m_goalReached;
    return m_currentGoalIndex >= m_globalPlan.size();
}

bool SimpleLocalPathPlanner::TransformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& targetFrame, std::vector<geometry_msgs::PoseStamped>& transformedPoses) const
{
    
    const std::string originalFrame = poses.front().header.frame_id;
    const ros::Time poseTime = poses.front().header.stamp;

    // TODO put this in a try
    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = m_tf->lookupTransform(targetFrame, ros::Time(), originalFrame, poseTime,
                                                                                  originalFrame, ros::Duration(0.5));

    // transform plan
    transformedPoses.clear();
    for (const auto& originalPose : poses)
    {   
        geometry_msgs::PoseStamped transformedPose;
        tf2::doTransform(originalPose, transformedPose, plan_to_global_transform);
        transformedPoses.push_back(transformedPose);
    }

    return true;
}

double SimpleLocalPathPlanner::getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
{
    double fromYaw = tf2::getYaw(from.pose.orientation);
    double toYaw = tf2::getYaw(to.pose.orientation);
    return angles::shortest_angular_distance(fromYaw, toYaw);
}

double SimpleLocalPathPlanner::getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const
{
    return std::sqrt(std::pow(from.pose.position.x - to.pose.position.x, 2) + std::pow(from.pose.position.y - to.pose.position.y, 2));
}

geometry_msgs::Twist SimpleLocalPathPlanner::getRotationalTwist(const double& angularDelta) const
{
    const double velocity = std::min(m_config.maxAngularVelocity, fabs(angularDelta) * m_config.maxAngularVelocity * 10.0);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = angularDelta > 0 ? velocity : -1.0 * velocity;
    return cmd_vel;

}

geometry_msgs::Twist SimpleLocalPathPlanner::getLinearTwist(const double& linearDelta) const
{
    const double velocity = fabs(linearDelta) * m_config.maxLinearVelocity;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    return cmd_vel;
}

geometry_msgs::Twist SimpleLocalPathPlanner::ZeroTwist() const
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
