#include "singletrack_sim/singletrack_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  singletrack_sim singletrack_sim_node;
   
  singletrack_sim_node.Prepare();
  
  singletrack_sim_node.RunPeriodically();
  
  singletrack_sim_node.Shutdown();
  
  return (0);
}

