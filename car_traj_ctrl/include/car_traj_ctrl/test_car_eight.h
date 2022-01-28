#ifndef TEST_CAR_EIGHT_H_
#define TEST_CAR_EIGHT_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#define PI 3.14159265358979323846

#define NAME_OF_THIS_NODE "test_car_eight"


class test_car_eight
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;
    ros::Subscriber vehicleHeading_subscriber;
    ros::Subscriber vehiclePosition_subscriber;
    
    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double xref, dxref, ddxref, yref, dyref, ddyref;
    double speed, steer;

    double dt;
    double L;
    double a, T, w;
    double theta, eps;

    double Vxp, Vyp;
    double xp, yp;
    double Kpx, Kpy;
    double Tix, Tiy;
    double Ts;

    double err_x, err_y;
    double I_x_now, I_y_now;
    double I_x_prev, I_y_prev;

    /* ROS topic callbacks */
    void vehicleHeading_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void vehiclePosition_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* TEST_CAR_EIGHT_H_ */
