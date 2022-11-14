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
            maxAngularVelocity(0.5), // rad/s
            maxLinearVelocity(0.5), // m/s
            angularTolerance(0.01), // rad
            linearTolerance(0.01){} // metres

        double maxAngularVelocity;
        double maxLinearVelocity;
        double angularTolerance;
        double linearTolerance;
    };


    bool TransformPosesToFrame(const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& targetFrame, std::vector<geometry_msgs::PoseStamped>& transformedPoses) const;
    double getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    double getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    geometry_msgs::Twist getRotationalTwist(const double& angularDelta) const;
    geometry_msgs::Twist getLinearTwist(const double& linearDelta) const;
    geometry_msgs::Twist ZeroTwist() const;

    costmap_2d::Costmap2DROS* m_costmapROS;
    tf2_ros::Buffer* m_tf;

    std::vector<geometry_msgs::PoseStamped> m_globalPlan;
    size_t m_currentGoalIndex;
    bool m_initialised;
    bool m_goalReached;
    bool m_rotating;

    Config m_config;
};
};

#endif