#ifndef SIMPLE_LOCAL_PATH_PLANNER_H_
#define SIMPLE_LOCAL_PATH_PLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "simple_local_path_planner/simple_local_path_planner_config.h"

using namespace std;

namespace simple_local_path_planner_ros{

class SimpleLocalPathPlanner{
public:

    SimpleLocalPathPlanner();
    ~SimpleLocalPathPlanner();

    void reset();

    void setConfig(const Config& config);
    void setRobotPose(const geometry_msgs::PoseStamped& robot_pose);
    void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    const geometry_msgs::PoseStamped& getRobotPose() const;
    const geometry_msgs::PoseStamped& getTargetPose() const;
    const geometry_msgs::PoseStamped& getGoalPose() const;

    geometry_msgs::Twist getRotateToGoal();
    geometry_msgs::Twist getNextCmdVel();
    geometry_msgs::Twist getStoppedCmdVel();

    bool planAvailable() const;
    bool goalReached() const;
    bool isAtGoalPosition() const;
    bool isAtGoalOrientation() const;

private:
    enum MotionState
    {
        STOPPED,
        ROTATE,
        TRAVERSE
    };

    // Todo: General functions: consider moving these into some kind of utils namespace - would make it easier to do unit tests for these functions too.
    geometry_msgs::Twist zeroTwist() const;

    double getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    double getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;
    geometry_msgs::Twist getRotationalTwist(const double& angular_delta) const;
    geometry_msgs::Twist getLinearTwist(const double& linear_delta) const;
    double toRadians(const double& degrees) const;

    std::vector<geometry_msgs::PoseStamped> m_plan;
    geometry_msgs::PoseStamped m_robot_pose;
    size_t m_current_target_index;
    Config m_config;
    MotionState m_motion_state;
};
};

#endif