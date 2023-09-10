#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_sam_qn_node");
  ros::NodeHandle nh_private("~");

  FAST_LIO_SAM_QN_CLASS fast_lio_sam_qn_(nh_private);

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();

  fast_lio_sam_qn_.~FAST_LIO_SAM_QN_CLASS(); // Explicit call of destructor
 
  return 0;
}