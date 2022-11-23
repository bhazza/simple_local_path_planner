#include "simple_local_path_planner/simple_local_path_planner_ros.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


PLUGINLIB_EXPORT_CLASS(simple_local_path_planner_ros::SimpleLocalPathPlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_path_planner_ros{

SimpleLocalPathPlannerROS::SimpleLocalPathPlannerROS() : 
    m_costmap_ros(NULL), 
    m_tf(NULL), 
    m_initialised(false),
    m_slpp(){}

SimpleLocalPathPlannerROS::SimpleLocalPathPlannerROS(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : m_costmap_ros(NULL), m_tf(NULL), m_initialised(false)
{
    initialize(name, tf, costmap_ros);
}

SimpleLocalPathPlannerROS::~SimpleLocalPathPlannerROS() 
{
    // Clean up allocated memory
    delete m_server;

    if(m_costmap_ros != NULL)
      delete m_costmap_ros;

    if(m_tf != NULL)
      delete m_tf;
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void SimpleLocalPathPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!m_initialised)
    {
        m_tf = tf;
        m_costmap_ros = costmap_ros;
        m_initialised = true;

        // Set the costmap to be used by the planner
        m_slpp.setCostMap(costmap_ros->getCostmap());

        // Set up private node handle, dynamic reconfigure server and callback
        ros::NodeHandle private_nh("~/simple_local_path_planner");
        m_server = new dynamic_reconfigure::Server<simple_local_path_planner::NodeParametersConfig>(private_nh);
        dynamic_reconfigure::Server<simple_local_path_planner::NodeParametersConfig>::CallbackType cb = [this](auto& config, auto level){ dynamicReconfigureConfig(config, level); };
        m_server->setCallback(cb);
    }
    else
    {
        ROS_WARN("This planner has already been initialized");
    }
}

bool SimpleLocalPathPlannerROS::setPlan(
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

    // Reset the planner
    m_slpp.reset();

    ROS_DEBUG("Plan Updated");
    return true;
}

bool SimpleLocalPathPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(m_global_plan.empty())
        return false;


    // Get pose of the robot
    geometry_msgs::PoseStamped robot_pose;
    if (!m_costmap_ros->getRobotPose(robot_pose)) {
        ROS_ERROR("Could not get global robot pose");
        return false;
    }
    // Set planner
    m_slpp.setRobotPose(robot_pose);

    // Transform the provided plan into same frame as the robot
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    const std::string globalFrame = m_costmap_ros->getGlobalFrameID();
    transformPosesToFrame(m_global_plan, globalFrame, transformed_plan);
    // Set planner
    m_slpp.setPlan(transformed_plan);

    // Get next velocity cmd
    cmd_vel = m_slpp.getNextCmdVel();

    return true;
}

bool SimpleLocalPathPlannerROS::isGoalReached()
{
    if(!m_initialised)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return m_slpp.goalReached();
}

void SimpleLocalPathPlannerROS::dynamicReconfigureConfig(simple_local_path_planner::NodeParametersConfig& node_config, uint32_t)
{
    Config config;
    config.min_angular_velocity_degrees = node_config.min_angular_velocity_degrees;
    config.max_angular_velocity_degrees = node_config.max_angular_velocity_degrees;
    config.min_linear_velocity = node_config.min_linear_velocity;
    config.max_linear_velocity = node_config.max_linear_velocity;
    config.angular_tolerance_degrees = node_config.angular_tolerance_degrees;
    config.linear_tolerance = node_config.linear_tolerance;
    config.waypoint_step_size = node_config.waypoint_step_size;
    config.collision_cost_threshold = node_config.collision_cost_threshold;

    m_slpp.setConfig(config);
}


bool SimpleLocalPathPlannerROS::transformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& target_frame, std::vector<geometry_msgs::PoseStamped>& transformed_poses) const
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

}
