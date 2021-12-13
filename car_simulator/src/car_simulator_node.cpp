#include "car_simulator/car_simulator.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_simulator car_simulator_node;
  
  car_simulator_node.Prepare();
  
  car_simulator_node.RunPeriodically();
  
  car_simulator_node.Shutdown();
  
  return (0);
}

