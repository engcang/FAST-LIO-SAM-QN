#include "fast_lio_sam_qn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_sam_qn_node");
  ros::NodeHandle nh_private("~");

  FastLioSamQn fast_lio_sam_qn_(nh_private);

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();

  fast_lio_sam_qn_.~FastLioSamQn(); // Explicit call of destructor
 
  return 0;
}
