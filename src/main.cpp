#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
   ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);

  /**
  ros::AsyncSpinner spinner(20); // Use 4 threads
  spinner.start();
  */

  mapper.run();
  return 0;
}