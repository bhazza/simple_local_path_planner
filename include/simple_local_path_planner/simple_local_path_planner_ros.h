#ifndef SIMPLE_LOCAL_PATH_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PATH_PLANNER_ROS_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "simple_local_path_planner/simple_local_path_planner.h"

using namespace std;

namespace simple_local_path_planner_ros{

class SimpleLocalPathPlannerROS : public nav_core::BaseLocalPlanner{
public:

    SimpleLocalPathPlannerROS();
    SimpleLocalPathPlannerROS(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~SimpleLocalPathPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
private:

    bool transformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& target_frame, std::vector<geometry_msgs::PoseStamped>& transformed_poses) const;

    costmap_2d::Costmap2DROS* m_costmap_ros;
    tf2_ros::Buffer* m_tf;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    bool m_initialised;

    // Implementation of the planner
    SimpleLocalPathPlanner m_slpp;
};
};

#endif