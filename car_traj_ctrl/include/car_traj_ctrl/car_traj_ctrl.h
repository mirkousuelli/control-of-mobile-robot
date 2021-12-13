#ifndef CAR_TRAJ_CTRL_H_
#define CAR_TRAJ_CTRL_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include "bicycle_ode.h"

#define NAME_OF_THIS_NODE "car_traj_ctrl"


class car_traj_ctrl
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehicleState_publisher;
    ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;
    double L;
    double x0, y0, theta0, phi0;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    bicycle_ode* simulator;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* CAR_TRAJ_CTRL_H_ */
