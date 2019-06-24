
#include "ur3e_hmd/ur3e_hmd.h"
#include <signal.h>

using namespace ur_nu;

bool kill_process = false;
void SigIntHandler(int signal)
{
  kill_process = true;
  ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

int main(int argc, char **argv)
{

  int thread_sampling_freq;

  ros::init(argc, argv, "ur3e_hmd");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  // Thread Sampling Frequency
  int thread_sampling_freq_hz;
  if (node_handle.getParam("interface_sampling_freq_hz", thread_sampling_freq_hz)) {
    thread_sampling_freq = thread_sampling_freq_hz;
  } else
    thread_sampling_freq = 80;

  ros::Rate loop_rate(thread_sampling_freq);
  ROS_INFO_STREAM("thread sampling frequency: " << thread_sampling_freq << "hz");

  UR3e_hmd ur(node_handle);

  while (!kill_process) {
//    if (in.getStatus() == I_RUNNING) {
//      if (in.getPoseFK() == EXIT_SUCCESS)
//        in.sendPoseCmd();
//    }

    ur.control_loop();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
