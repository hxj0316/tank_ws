#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Int8MultiArray.h>
using namespace std;


serial::Serial ser; //声明串口对象 
uint8_t s_buffer[10]; 


void write_callback(const std_msgs::Int8MultiArray &cmd)
{
    if (cmd.data[0] == 1&&cmd.data[1] == 0&&cmd.data[2] == 0&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x01;
      s_buffer[8]=0xB0;
      s_buffer[9]=0x52;
      ser.write(s_buffer,10);   //发送串口数据 
     }
     if(cmd.data[0] == 0&&cmd.data[1] == 1&&cmd.data[2] == 0&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT2 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x02;
      s_buffer[8]=0xF0;
      s_buffer[9]=0x53;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 0&&cmd.data[1] == 0&&cmd.data[2] == 1&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT3 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x04;
      s_buffer[8]=0x70;
      s_buffer[9]=0x51;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 0&&cmd.data[1] == 0&&cmd.data[2] == 0&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x08;
      s_buffer[8]=0x70;
      s_buffer[9]=0x54;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 1&&cmd.data[2] == 0&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT2 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x03;
      s_buffer[8]=0x31;
      s_buffer[9]=0x93;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 0&&cmd.data[2] == 1&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT3 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x05;
      s_buffer[8]=0xB1;
      s_buffer[9]=0x91;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 0&&cmd.data[2] == 0&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x09;
      s_buffer[8]=0xB1;
      s_buffer[9]=0x94;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 0&&cmd.data[1] == 1&&cmd.data[2] ==1&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT2,OUT3 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x06;
      s_buffer[8]=0xF1;
      s_buffer[9]=0x90;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 0&&cmd.data[1] == 1&&cmd.data[2] == 0&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT2,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0A;
      s_buffer[8]=0xF1;
      s_buffer[9]=0x95;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] ==0&&cmd.data[1] == 0&&cmd.data[2] == 1&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT3,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0C;
      s_buffer[8]=0x71;
      s_buffer[9]=0x97;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 1&&cmd.data[2] == 1&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT2,OUT3 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x07;
      s_buffer[8]=0x30;
      s_buffer[9]=0x50;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 0&&cmd.data[2] == 1&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT3,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0D;
      s_buffer[8]=0xB0;
      s_buffer[9]=0x57;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 1&&cmd.data[2] == 1&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT2,OUT3 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0E;
      s_buffer[8]=0xF0;
      s_buffer[9]=0x56;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 1&&cmd.data[2] == 0&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT2,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0B;
      s_buffer[8]=0x30;
      s_buffer[9]=0x53;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 1&&cmd.data[1] == 1&&cmd.data[2] == 1&&cmd.data[3] == 1)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("OUT1,OUT2,OUT3,OUT4 opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x0F;
      s_buffer[8]=0x31;
      s_buffer[9]=0x96;
      ser.write(s_buffer,10);   //发送串口数据 
     }
    if(cmd.data[0] == 0&&cmd.data[1] == 0&&cmd.data[2] == 0&&cmd.data[3] == 0)
    { ROS_INFO_STREAM("Writing to serial port"); 
      ROS_INFO_STREAM("NO OUT opened"); 
      s_buffer[0]=0xFE;
      s_buffer[1]=0x0F;
      s_buffer[2]=0x00;
      s_buffer[3]=0x00;
      s_buffer[4]=0x00;
      s_buffer[5]=0x04;
      s_buffer[6]=0x01;
      s_buffer[7]=0x00;
      s_buffer[8]=0x71;
      s_buffer[9]=0x92;
      ser.write(s_buffer,10);   //发送串口数据 
     }
}



int main (int argc, char** argv) 
{ 
 
  //初始化节点 
  ros::init(argc, argv, "tank_fire_node"); 
  //声明节点句柄 
  ros::NodeHandle nh; 
  //订阅主题，并配置回调函数 
  ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
  //发布主题 
  ros::Publisher read_pub = nh.advertise<std_msgs::Int8MultiArray>("read", 1000); 


  try 
  { 
   //设置串口属性，并打开串口 
   ser.setPort("/dev/ttyUSB0"); 
   ser.setBaudrate(9600); 
   serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
   ser.setTimeout(to); 
   ser.open(); 
  } 
  catch (serial::IOException& e) 
  { 
   ROS_ERROR_STREAM("Unable to open port "); 
   return -1; 
  } 
  //检测串口是否已经打开，并给出提示信息 
  if(ser.isOpen()) 
  { 
   ROS_INFO_STREAM("Serial Port opend"); 
  } 
  else 
  { 
   return -1; 
  } 
  //指定循环的频率 
  ros::Rate loop_rate(50); 
  while(ros::ok()) 
  {  
    ros::spinOnce(); 
    loop_rate.sleep(); 
  } 

} 