#include "turn_on_tank/tank_robot.h"

#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tank_robot");        //ROS initializes and sets the node name //ROS初始化 并设置节点名称
  turn_on_robot Robot_Control;                //Instantiate an object //实例化一个对象
  ROS_INFO("tank_robot node has turned on "); //Prompt that the wheeltec_robot node has been created //提示已创建wheeltec_robot节点
  Robot_Control.Control();                    //Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作

  return 0;
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  // int count = 7;
  while (ros::ok())
  // {
  //   if (count)
    {
      can_msgs::Frame can_frame;
      can_frame.id = 0x0CFF98ED;
      can_frame.is_extended = true;
      can_frame.is_rtr = false;
      can_frame.is_error = false;
      can_frame.dlc = 8;

      can_frame.data[0] = 0x11;
      can_frame.data[1] = 0x11;
      can_frame.data[2] = 0x00;
      can_frame.data[3] = 0x00;
      can_frame.data[4] = 0x00;
      can_frame.data[5] = 0x00;
      can_frame.data[6] = 0x00;
      can_frame.data[7] = 0x00;

      Can_pub.publish(can_frame);
      ROS_INFO_STREAM("motor  enable");

      // count--;
    ros::Duration(0.5).sleep();
  
    ros::spinOnce(); //The loop waits for the callback function //循环等待回调函数
  }

}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  ROS_INFO("Into Callback");

  uint8_t transition[8]; //intermediate variable //中间变量
  uint8_t flag = 0;
  short temp_x, temp_z;
  can_msgs::Frame can_frame;

  memset(&transition, 0, sizeof(transition));

  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度

  temp_x = twist_aux.linear.x * 2000; //将浮点数放大2000倍

  transition[3] = temp_x >> 8;
  transition[2] = temp_x & 0xFF;

  temp_x = twist_aux.linear.x * 2000; //将浮点数放大2000倍
  temp_x =  - temp_x;
  transition[1] = temp_x >> 8;
  transition[0] = temp_x & 0xFF;

  //The target angular velocity of the robot's Z axis
  //机器人z轴的目标角速度
  temp_z = twist_aux.angular.z * 2000;

  if (twist_aux.angular.z > 0)
  {

    transition[1] = temp_z >> 8;
    transition[0] = temp_z & 0xFF;

    transition[3] = temp_z >> 8;
    transition[2] = temp_z & 0xFF;
  }
  else if (twist_aux.angular.z < 0)
  {
    transition[1] = temp_z >> 8;
    transition[0] = temp_z & 0xFF;

    transition[3] = temp_z >> 8;
    transition[2] = temp_z & 0xFF;
  }

  can_frame.id = 0x0CFF98D1;
  can_frame.is_extended = true;
  can_frame.is_rtr = false;
  can_frame.is_error = false;
  can_frame.dlc = 8;
  for (uint8_t i = 0; i < can_frame.dlc; i++)
  {
    can_frame.data[i] = transition[i];
  }

  Can_pub.publish(can_frame);
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot() : Sampling_Time(0)
{
  //Clear the data
  //清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));

  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("cmd_vel", cmd_vel, "cmd_vel");                   //Velocity control command //速度控制指令
  private_nh.param<std::string>("sent_messages", sent_messages, "sent_messages"); //Velocity control command //速度控制指令

  Can_pub = n.advertise<can_msgs::Frame>("sent_messages", 100);

  //Set the velocity control command callback function
  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub = n.subscribe(cmd_vel, 100, &turn_on_robot::Cmd_Vel_Callback, this);

  ROS_INFO_STREAM("Node ready"); //Prompt message //提示信息

  can_msgs::Frame can_frame;
  can_frame.id = 0x0CFF98ED;
  can_frame.is_extended = true;
  can_frame.is_rtr = false;
  can_frame.is_error = false;
  can_frame.dlc = 8;

  can_frame.data[0] = 0x11;
  can_frame.data[1] = 0x11;
  can_frame.data[2] = 0x00;
  can_frame.data[3] = 0x00;
  can_frame.data[4] = 0x00;
  can_frame.data[5] = 0x00;
  can_frame.data[6] = 0x00;
  can_frame.data[7] = 0x00;

  Can_pub.publish(can_frame);
  ROS_INFO_STREAM("motor  enable");
  //Sends the stop motion command to the lower machine before the turn_on_tank object ends
  //对象turn_on_robot开始向下位机发送电机使能运动命令
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  ROS_INFO_STREAM("motor stop enable");
  //Sends the stop motion command to the lower machine before the turn_on_tank object ends
  //对象turn_on_robot结束向下位机发送电机停止运动命令
  can_msgs::Frame can_frame;

  can_frame.id = 0x0CFF98ED;
  can_frame.is_extended = true;
  can_frame.is_rtr = false;
  can_frame.is_error = false;
  can_frame.dlc = 8;
  can_frame.data[0] = 0x00;
  can_frame.data[1] = 0x00;
  can_frame.data[2] = 0x00;
  can_frame.data[3] = 0x00;
  can_frame.data[4] = 0x00;
  can_frame.data[5] = 0x00;
  can_frame.data[6] = 0x00;
  can_frame.data[7] = 0x00;
}