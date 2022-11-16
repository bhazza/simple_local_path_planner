#ifndef SIMPLE_LOCAL_PATH_PLANNER_CONFIG_H_
#define SIMPLE_LOCAL_PATH_PLANNER_CONFIG_H_
namespace simple_local_path_planner_ros{

struct Config
{
    Config(){loadDefaults();}
    
    void loadDefaults()
    {
        min_angular_velocity_degrees = 5;   // deg/s
        max_angular_velocity_degrees = 40;  // deg/s
        min_linear_velocity = 0.05;         // m/s
        max_linear_velocity = 0.5;          // m/s
        angular_tolerance_degrees = 5;      // deg
        linear_tolerance = 0.05;            // m    
        goal_step = 10;                     // metres
    }

    double min_angular_velocity_degrees;
    double max_angular_velocity_degrees;
    double min_linear_velocity;
    double max_linear_velocity;
    double angular_tolerance_degrees;
    double linear_tolerance;
    size_t goal_step;
};
}

#endif