#ifndef SIMPLE_LOCAL_PATH_PLANNER_H_
#define SIMPLE_LOCAL_PATH_PLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "simple_local_path_planner/simple_local_path_planner_config.h"

using namespace std;

namespace simple_local_path_planner_ros{

/**
 * @class SimpleLocalPathPlanner
 * @brief Computes control velocities for a robot given a plan and the robot pose (both of which should be in the 
 * same frame).
 */
class SimpleLocalPathPlanner{
public:

    SimpleLocalPathPlanner();
    ~SimpleLocalPathPlanner();

    /** \brief Clears the plan and current waypoint index. This should be called whenever the plan has been changed. */
    void reset();


    // Setting up the planner

    /** \brief Set the config parameters used by the planner
     * \param config config values
     */
    void setConfig(const Config& config);

    /** \brief Set the robot pose. Note that consitent frame of reference should be used for both robot and plan poses.
     * \param robot_pose Pose of robot, in the same frame as the plan
     */
    void setRobotPose(const geometry_msgs::PoseStamped& robot_pose);

    /** \brief Set the plan that the robot is to follow. 
     * Note that consitent frame of reference should be used for both robot and plan poses.
     * \param plan Vector of waypoints for the robot to follow, in the same frame as the robot pose
     */
    void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);


    // Pose getters

    /** \brief Get the current pose of the robot
     * \return Current pose of the robot
     */
    const geometry_msgs::PoseStamped& getRobotPose() const;

    /** \brief Get the pose of the current target waypoint in the plan
     * \return Target waypoint pose
     */
    const geometry_msgs::PoseStamped& getTargetPose() const;

    /** \brief Get the final goal pose of the plan, i.e. the last pose in the vector plan.
     * \return Goal pose
     */
    const geometry_msgs::PoseStamped& getGoalPose() const;


    // Twist getters

    /** \brief Get the rotational twist velocity in the angular direction of the final goal pose, 
     * i.e. the last pose in the vector plan. Velocity will be proportion to the angular distance from
     * robot pose to goal, and will be bound by the min/max angular thresholds set in config.
     * \return Rotational twist message
     */
    geometry_msgs::Twist getRotateToGoal();
    
    /** \brief Get the twist velocity in the direction of the current target waypoint of the plan.
     * The provided velocities will first rotate in place so that the robot is aimed at the next waypoint.
     * Subsequent calls will then result in forward velocity towards the goal.
     * Velocity will be proportion to the linear/angular distance from robot pose to next waypoint, 
     * and will be bound by the min/max angular thresholds set in config.
     * \return Rotational OR translational twist message
     */
    geometry_msgs::Twist getNextCmdVel();

    /** \brief Get a twist message with all zero velocity
     * \return Zero twist message
     */
    geometry_msgs::Twist getStoppedCmdVel();


    // Goal checker flags

    /** \brief Check that plan isn't empty
     * \return True if plan is populated, false if empty
     */
    bool planAvailable() const;

    /** \brief Check if robot pose is at the final goal pose (for both translational and rotational values),
     * within the linear and angular tolerances set in the config
     * \return True if at goal
     */
    bool goalReached() const;

    /** \brief Check if robot xy position is at the final goal xy position, within the linear_tolerance set in config
     * \return True if at goal xy position
     */
    bool isAtGoalPosition() const;

    /** \brief Check if robot yaw angle is at the final goal yaw angle, within the angular_tolerance set in config
     * \return True if at goal angular position
     */
    bool isAtGoalOrientation() const;

private:

    /** \brief Enum used to keep track of robot velocity state */
    enum MotionState
    {
        STOPPED,
        ROTATE,
        TRAVERSE
    };

    // Todo: General functions: consider moving these into some kind of utils namespace - would make it easier to do unit tests for these functions too.
        
    /** \brief Provide twist message with all zero velocities. 
     * Important: This is private because it should not be called by user - they should instead use 
     * getStoppedCmdVel() which will also set motion state used by other functions.
     * \return Zero twist message
     */
    geometry_msgs::Twist zeroTwist() const;

    /** \brief Get the shortest angular difference between two poses
     * \param from From pose
     * \param to To pose
     * \return The shortest angular distance between from and to poses
     */
    double getAngularDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;

    /** \brief Get the linear distance between two poses
     * \param from From pose
     * \param to To pose
     * \return The linear distance between from and to poses
     */
    double getLinearDelta(const geometry_msgs::PoseStamped& from, const geometry_msgs::PoseStamped& to) const;

    /** \brief Get rotation velocity in the direction of the angular delta. This will be proportional to the 
     * magnitude of the angular_delta, bound by the min and max thresholds defined in the config.
     * Important: This is private because it should not be called by user - they should instead use 
     * getNextCmdVel()/getRotateToGoal() which will also set motion state used by other functions.
     * \param angular_delta Angular delta with direction, as per right-hand rule.
     * \return Rotational twist message 
     */
    geometry_msgs::Twist getRotationalTwist(const double& angular_delta) const;

    /** \brief Get linear velocity proportional to the magnitude of the angular_delta, 
     * bound by the min and max thresholds defined in the config.
     * Important: This is private because it should not be called by user - they should instead use 
     * getNextCmdVel() which will also set motion state used by other functions.
     * \param linear_delta The linear delta 
     * \return Rotational twist message 
     */
    geometry_msgs::Twist getLinearTwist(const double& linear_delta) const;

    /** \brief Converts degrees to radians
     * \param degrees Degrees
     * \return Radians
     */
    double toRadians(const double& degrees) const;

    std::vector<geometry_msgs::PoseStamped> m_plan;
    geometry_msgs::PoseStamped m_robot_pose;
    size_t m_current_target_index;
    Config m_config;
    MotionState m_motion_state;
};
};

#endif