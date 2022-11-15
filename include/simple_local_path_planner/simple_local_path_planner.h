#ifndef SIMPLE_LOCAL_PATH_PLANNER_H_
#define SIMPLE_LOCAL_PATH_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

namespace simple_local_path_planner{

class SimpleLocalPathPlanner : public nav_core::BaseLocalPlanner{
public:

    SimpleLocalPathPlanner();
    SimpleLocalPathPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~SimpleLocalPathPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
private:
    struct Config
    {
        Config():
            max_angular_velocity(0.5), // rad/s
            max_linear_velocity(0.5), // m/s
            angular_tolerance(0.01), // rad
            linear_tolerance(0.01),
            goal_step(10){} // metres

        double max_angular_velocity;
        double max_linear_velocity;
        double angular_tolerance;
        double linear_tolerance;
        size_t goal_step;
    };


    bool transformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& target_frame, std::vector<geometry_msgs::PoseStamped>& transformed_poses) const;
    double getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    double getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    geometry_msgs::Twist getRotationalTwist(const double& angular_delta) const;
    geometry_msgs::Twist getLinearTwist(const double& linear_delta) const;
    geometry_msgs::Twist zeroTwist() const;

    costmap_2d::Costmap2DROS* m_costmap_ros;
    tf2_ros::Buffer* m_tf;

    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    size_t m_target_waypoint_index;
    bool m_initialised;
    bool m_goal_reached;
    bool m_rotating;

    Config m_config;
};
};

#endif