//communication.cpp

#include "panelboard_com.hpp"



/**************************************
Date: January 28, 2021
Function: Publish temperate-related information
功能: 发布温度相关信息
***************************************/
void panelboard_turn_on::Publish_temper()
{
    std_msgs::Float32 temper_msgs;  //定义温度发布话题的数据类型
    static float Count_temper_Pub=0;
    if(Count_temper_Pub++>5)
      {
        Count_temper_Pub=0;  
        temper_msgs.data = temper_value; //温度获取
        temper_publisher.publish(temper_msgs);//发布温度信息
      }
}


/**************************************
Date: January 28, 2021
Function: Publish temperate-related information
功能: 发布摇杆相关信息
***************************************/
void panelboard_turn_on::Publish_Joy()
{
  
  joy_publisher.publish(joy_data);

}


/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char panelboard_turn_on::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据
***************************************/
bool panelboard_turn_on::Get_Data()
{ 
  short transition_16=0, j=0, Header_Pos=0, Tail_Pos=0; //Intermediate variable //中间变量
  float axies_temp;
  uint32_t temp; 
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  board_serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据

  /*
  //View the received raw data directly and debug it for use//直接查看接收到的原始数据，调试使用
  ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
  Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],Receive_Data_Pr[7],
  Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13],Receive_Data_Pr[14],Receive_Data_Pr[15],
  Receive_Data_Pr[16],Receive_Data_Pr[17],Receive_Data_Pr[18],Receive_Data_Pr[19],Receive_Data_Pr[20],Receive_Data_Pr[21],Receive_Data_Pr[22],Receive_Data_Pr[23]);
  */

  //Record the position of the head and tail of the frame //记录帧头帧尾位置
  for(j=0;j<24;j++)
  {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
  }

  if(Tail_Pos==(Header_Pos+23))
  {
    //If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
    //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
    // ROS_INFO("1----");
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
  }
  else if(Header_Pos==(1+Tail_Pos))
  {
    //如果帧头在帧尾后面，纠正数据位置后复制数据包到Receive_Data.rx
    // If the header is behind the end of the frame, copy the packet to receive_data.rx after correcting the data location
    // ROS_INFO("2----");
    for(j=0;j<24;j++)
    Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%24];
  }
  else 
  {
    //其它情况则认为数据包有错误
    // In other cases, the packet is considered to be faulty
    // ROS_INFO("3----");
    return false;
  }    
  //Check receive_data.rx for debugging use //查看Receive_Data.rx，调试使用
  // ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
  // Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
  // Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
  // Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]); 
  
  Receive_Data.Frame_Header= Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail= Receive_Data.rx[23];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER ) //Judge the frame header //判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //Judge the end of the frame //判断帧尾
    { 
      joy_data.header.stamp = ros::Time().now();
      joy_data.axes.resize(8);//需要Resize，否则报错
      joy_data.buttons.resize(25);
      joy_data.header.frame_id = joy_dev.c_str();
      temp = Receive_Data.rx[20]<< 16 | Receive_Data.rx[19] << 8 | Receive_Data.rx[18];

      //Get the battery voltage
      //获取电池电压
      transition_16 = 0;
      transition_16 |=  Receive_Data.rx[21]<<8;
      transition_16 |=  Receive_Data.rx[22];  
      temper_value = transition_16/1000+(transition_16 % 1000)*0.001; //温度


      for(int8_t i = 0;i<8;i++)
      {

        joy_data.axes[i] = Receive_Data.rx[i*2+1]<<8 | Receive_Data.rx[i*2+2] ;

      }

      


      for(int8_t i = 0;i<25;i++)
      {
        
        joy_data.buttons[i] =  (temp >> i) & 0x01;

      }
      

      return true;
    }
  } 
  return false;
}


/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void panelboard_turn_on::Control()
{
  while(ros::ok())
  {
    if(Get_Data())
    {
      Publish_temper();   //Pub the topic of power supply voltage //发布电源电压话题
      Publish_Joy();
    }
  
    ros::spinOnce();   //The loop waits for the callback function //循环等待回调函数
  }
    
    
}


void panelboard_turn_on::Cmd_Vel_Callback(const std_msgs::UInt16 &cmd)
{
  uint8_t temp1,temp2;
  uint16_t temp;
  temp = cmd.data;
  temp1 = (temp >> 8) & 0xFF;
  temp2 = temp & 0xFF;
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = temp1;
  Send_Data.tx[2] = temp2;
  Send_Data.tx[3] = FRAME_TAIL;
  
  try
  {
    board_serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    //else ROS_INFO("akm mode, not subcribe cmd_vel");
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }

}


/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
panelboard_turn_on::panelboard_turn_on()
{
  //Clear the data
  //清空数据
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));

  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  ros::Rate loop_rate(10);
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyUSB0"); //Fixed serial port number //固定串口号
  private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200); //Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  private_nh.param<std::string>("panelboard" , joy_dev ,"panelboard");
  private_nh.param<std::string>("panel_vel" , panel_cmd , "panel_vel");

  temper_publisher = n.advertise<std_msgs::Float32>("boardtemper", 100); //Create a battery-voltage topic publisher //创建温度话题发布者
  joy_publisher = n.advertise<sensor_msgs::Joy>("Joy", 100); //发布摇杆数据


  //Set callback function
  //回调函数设置
  panel_set  = n.subscribe(panel_cmd, 100, &panelboard_turn_on::Cmd_Vel_Callback, this); 
  //Akm_cmd_Vel_Sub = n.subscribe(akm_cmd_vel, 100, &turn_on_robot::Akm_cmd_Vel_Callback, this); 

  ROS_INFO_STREAM("Node ready"); //Prompt message //提示信息
  
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    board_serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    board_serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    board_serial.setTimeout(_time);
    board_serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(board_serial.isOpen())
  {
    ROS_INFO_STREAM("Serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
panelboard_turn_on::~panelboard_turn_on()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令


  Send_Data.tx[0]=FRAME_HEADER;
 // Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[20]=FRAME_TAIL; 

  try
  {
    board_serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unpanelboardable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  board_serial.close(); //Close the serial port //关闭串口  
  ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
}



int main(int argc , char** argv)
{
  ros::init(argc, argv, "panelboard_turn_on"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  
  panelboard_turn_on panelboard; //Instantiate an object //实例化一个对象
  panelboard.Control();
 // Robot_Control.Control(); //Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  ROS_INFO("panelboard_turn_on node has turned on "); //Prompt that the wheeltec_robot node has been created //提示已创建wheeltec_robot节点 
}
