#include "main.h"
#include <signal.h>
void signal_handler(sig_atomic_t s)
{
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_sam_qn_node");
  ros::NodeHandle nh_private("~");

  FAST_LIO_SAM_QN_CLASS fast_lio_sam_qn_(nh_private);

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}