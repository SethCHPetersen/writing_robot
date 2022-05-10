#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include "ros/console.h"
#include "../include/moveUR/moveUR_node.h"
#include <sstream>

using namespace std;
float upOrDown = 0; // 1 = in positive z dirrection, -1 = negative z dirrection
ros::Publisher MovePub;

void get_target_tcp_pose()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "get_target_tcp_pose()";
  msg.data = ss.str();
  MovePub.publish(msg);
}

void zeroFTSensor()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "zero_ftsensor()";
  msg.data = ss.str();
  MovePub.publish(msg);
  return;
}
void MoveRobot(float x, float y, float z, float acceleration, float veloctity)
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "movel(pose_trans(get_target_tcp_pose(), p[0,0," + to_string(z) + ",0,0,0]), a=" + to_string(acceleration) + ", v=" + to_string(veloctity) + ", t=0, r=0)";
  ss << "movel([], a=" + to_string(acceleration) + ", v=" + to_string(veloctity) + ", t=0, r=0)";
  msg.data = ss.str();
  MovePub.publish(msg);
  return;
}

void moveRobotToForce(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  float zForce = msg->wrench.force.z;
  if (zForce >= -15)
  {
    if (upOrDown != 1)
    {
      cout << "changing direction to positive " << endl;
      // MoveRobotZ(.05, .1, .02); // move in positive z
      upOrDown = 1;
    }
  }

  if (zForce < -20)
  {
    if (upOrDown != -1)
    {
      cout << "changing direction to negative  " << endl;
      // MoveRobotZ(-.05, .1, .02);
      upOrDown = -1;
      cout << upOrDown << " up or down" << endl;
    }
  }
  return;
}

void chatterCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  get_target_tcp_pose();
  msg->wrench.force.z;
  std::cout << "got data" << std::endl;
  std::cout << msg->wrench.force.z << std::endl;
  // moveRobotToForce(msg);
  ros::Rate rate = ros::Rate(1);
  rate.sleep();
  return;
}

int main(int argc, char **argv)
{

  // ros::init(argc, argv, "getingForce");

  // Link link1(0, 0, .15185, 3.14159265358979 / 2); // got these from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
  // Link link2(0, -.24355, 0, 0);
  // Link link3(0, -.2132, 0, 0);
  // Link link4(0, 0, .13102, 3.14159265358979 / 2);
  // Link link5(0, 0, 0.08535, -3.14159265358979 / 2);
  // Link link6(0, 0, 0.0921, 0);
  // vector<float> currentPos;
  // currentPos.push_back(0);
  // currentPos.push_back(0);
  // currentPos.push_back(0);
  // currentPos.push_back(0);
  // currentPos.push_back(0);
  // currentPos.push_back(0);

  // moveUR ur3(currentPos, link1, link2, link3, link4, link5, link6);

  // vector<float> newPOS;
  // newPOS.push_back(.287);
  // newPOS.push_back(-.350);
  // newPOS.push_back(.140);
  // newPOS.push_back(3.092);
  // newPOS.push_back(-.163);
  // newPOS.push_back(.041);

  // ur3.connectToRobot();
  // ur3.zeroFTSensor();

  
  //ur3.moveRobot(newPOS, .05, .02);

  ros::NodeHandle n;
  MovePub = n.advertise<std_msgs::String>("/ur_hardware_interface/script_command", 10000);
  zeroFTSensor();
  int attempts = 0;
  while (MovePub.getNumSubscribers() < 1)
  {
    attempts++;
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
    cout << "still waiting for subscriber" << endl;
    if (attempts == 10)
    {
      return 0;
    }
  }

  ros::Subscriber sub = n.subscribe("wrench", 10, chatterCallback);

  cout << "moving" << endl;
  ros::spin();

  return 0;
}