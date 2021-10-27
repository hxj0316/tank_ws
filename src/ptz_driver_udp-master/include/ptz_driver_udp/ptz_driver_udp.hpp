#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <stdbool.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h> //for sockaddr_in
#include <arpa/inet.h>  //for socket
#include <sys/types.h>
#include <sys/socket.h>



using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK 1 //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK 0 //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER 0XFF //Frame head //帧头
#define addr 0x01;

#define RECEIVE_DATA_SIZE 255 //The length of the data sent by the lower computer

#define SEND_DATA_SIZE 7 //The length of data sent by ROS to the lower machine //ROS向云台发送的数据内容


#define SRC_PORT 8888  //本地端口
#define SRC_IP_ADDRESS "192.168.8.222" //本地地址

#define DEST_PORT 6666 //远端端口，即ptz端口
#define DSET_IP_ADDRESS "192.168.8.200" //远端地址，即ptz地址


#define ptz_addr 0x01

#define ROS_DEBUG_FLAG 0 //为1开启信息输出

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{   
    uint8_t tx[SEND_DATA_SIZE];
    unsigned char Check_Sum;
} SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float temper;
    unsigned char Frame_Tail;
} RECEIVE_DATA;

class ptz_driver
{
public:
    ptz_driver();
    ~ptz_driver();
    void Control(); //Loop control code //循环控制代码

private:
    ros::NodeHandle n;       //Create a ROS node handle //创建ROS节点句柄
    ros::Subscriber ptz_cmd; //创建命令订阅者
    ros::Publisher motion_publisher; //Initialize the topic publisher //初始化话题发布者

    void Cmd_Vel_Callback(const std_msgs::Float32MultiArray &cmd);

    //初始化
    void init_udp();
    void init_ptz();

    //UDP收发数据
    void UDP_Send(const uint8_t Data[], int len);
    bool Get_Data();
    void Hori_Control(const std_msgs::Float32MultiArray &cmd);
    void Vert_Control(const std_msgs::Float32MultiArray &cmd);
    void Speed_Control(const std_msgs::Float32MultiArray &cmd);
    unsigned char Check_Sum(); //check function //校验函数

    void Publish_motion();


    RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
    SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

    std::string ptz_cmd_vel;

    int fd, r; //定义套接字
    struct sockaddr_in addr_to; //目标服务器地址
    struct sockaddr_in addr_from;


    float angle_h , angle_v ; //定义水平角度和垂直角度
    float speed_h , speed_v ; //定义水平速度和垂直速度
    bool isSend_H = false;
    bool isSend_V = false;
};
