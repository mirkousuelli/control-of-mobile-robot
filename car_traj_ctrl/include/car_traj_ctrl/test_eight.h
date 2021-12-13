#ifndef TEST_HEIGHT_H_
#define TEST_HEIGHT_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#define RUN_PERIOD_DEFAULT 0.01
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */

#define NAME_OF_THIS_NODE "test_height"


class test_eight
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;
    
    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double xref, dxref, ddxref, yref, dyref, ddyref;
    double speed, steer;

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* TEST_HEIGHT_H_ */
