#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdlib.h>   
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <stdbool.h>
#include <string> 
#include <stdlib.h>
#include <unistd.h>

using namespace std;


//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
//下位机发送过来的数据的长度 ADC每通道2Byte 共16bytes 加上温度2bytes 帧头尾2bytes 按键数据4byte 共24Bytes
#define RECEIVE_DATA_SIZE 24        //The length of the data sent by the lower computer 
//帧头尾2Bytes 数据2Bytes
#define SEND_DATA_SIZE    4         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];       
		unsigned char Frame_Tail; 
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float temper;	
		unsigned char Frame_Tail;
}RECEIVE_DATA;

class panelboard_turn_on
{
    public:
        panelboard_turn_on();
        ~panelboard_turn_on();
        void Control();          //Loop control code //循环控制代码
        serial::Serial board_serial;

    private:
        ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄

        ros::Publisher temper_publisher; //Initialize the topic publisher //初始化话题发布者
        ros::Publisher joy_publisher; 

        ros::Subscriber panel_set;
        void Publish_temper();   //Pub the power supply voltage topic //发布电源电压话题
        void Publish_Joy(); //发布摇杆话题
        void Cmd_Vel_Callback(const std_msgs::UInt16 &cmd);
 
        //从串口(ttyUSB)读取数据
        bool Get_Data();   
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数

        string usart_port_name ; //Define the related variables //定义相关变量
         int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体
        sensor_msgs::Joy joy_data; //定义摇杆数据
        std::string joy_dev;
        std::string panel_cmd;

        float temper_value;       //温度


};
