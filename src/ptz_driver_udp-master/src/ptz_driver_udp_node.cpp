#include "ptz_driver_udp.hpp"

/**************************************
Date: 2021/10/04
功能: UDP发送程序
***************************************/
void ptz_driver::UDP_Send(const uint8_t Data[], int len)
{
    int sendbyte_len;
    sendbyte_len = sendto(fd, Data, len, 0, (struct sockaddr *)&addr_to, sizeof(addr_to));

    if (sendbyte_len == -1)
    {
        ROS_INFO_STREAM("send falure!");
    }
    else
    {
        if (ROS_DEBUG_FLAG)
            ROS_INFO("%d bytes have been sended successfully!\n", sendbyte_len);
    }
}

/**************************************
Date: 2021/10/01
功能: 订阅云台控制命令回调函数
***************************************/
void ptz_driver::Cmd_Vel_Callback(const std_msgs::Float32MultiArray &cmd)
{

    if (cmd.data[6] == 1) //停止指令
    {
        Send_Data.tx[0] = 0xFF;
        Send_Data.tx[1] = addr;
        Send_Data.tx[2] = 0;
        Send_Data.tx[3] = 0; //停止转动
        Send_Data.tx[4] = 0; //停止转动
        Send_Data.tx[5] = 0; //停止转动
        Send_Data.tx[6] = Check_Sum();
        UDP_Send(Send_Data.tx, sizeof(Send_Data.tx));
        if (ROS_DEBUG_FLAG)
            ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
                     Send_Data.tx[0], Send_Data.tx[1], Send_Data.tx[2], Send_Data.tx[3], Send_Data.tx[4], Send_Data.tx[5], Send_Data.tx[6]);
    }
    else
    {
        int flag_a, flag_s;
        flag_a = cmd.data[0];
        flag_s = cmd.data[3];
        if(cmd.data[0] == 1 & cmd.data[3] == 1)
        {
            flag_a = 1;
            flag_s = 0;  //优先角度控制
        }
        

        if (flag_a) //角度控制标志位
        {

            if (!isSend_H)
            {
                if (!(cmd.data[1]<(angle_h + 0.15) & cmd.data[1]>(angle_h - 0.15)))
                    Hori_Control(cmd);
            }

            if (!(cmd.data[2]<(angle_v + 0.15) & cmd.data[2]>(angle_v - 0.15)) & !isSend_V)
                Vert_Control(cmd);
            else
                isSend_V = false;
        }
        else if (flag_s) //速度控制指令
        {
            Speed_Control(cmd);
        }
    }
}

/**************************************
Date: 2021/10/04
功能: 水平角度控制函数
***************************************/
void ptz_driver::Speed_Control(const std_msgs::Float32MultiArray &cmd)
{
    float temp_h, temp_v;
    Send_Data.tx[0] = 0xFF;
    Send_Data.tx[1] = addr;
    Send_Data.tx[2] = 0;

    temp_h = cmd.data[4] * 10;
    temp_v = cmd.data[5] * 10;

    //判断转动方向，以右上为正数值
    if (cmd.data[4] > 0) //水平向右为正
    {
        if (cmd.data[5] > 0)
            Send_Data.tx[3] = 0x0A; //右上方向
        else if (cmd.data[5] < 0)
        {
            Send_Data.tx[3] = 0x12; //右下方向
            temp_v = -temp_v;
        }

        else if (cmd.data[5] == 0)
            Send_Data.tx[3] = 0x02; //右方向
    }
    else if (cmd.data[4] < 0) //水平向左
    {
        temp_h = -temp_h;
        if (cmd.data[5] > 0)
            Send_Data.tx[3] = 0x0C; //左上方向
        else if (cmd.data[5] < 0)
        {
            Send_Data.tx[3] = 0x14; //左下方向
            temp_v = -temp_v;
        }

        else if (cmd.data[5] == 0)
            Send_Data.tx[3] = 0x04; //左方向
    }
    else if (cmd.data[4] == 0)
    {
        if (cmd.data[5] > 0)
            Send_Data.tx[3] = 0x08; //上方向
        else if (cmd.data[5] < 0)
        {
            Send_Data.tx[3] = 0x10; //下方向
            temp_v = -temp_v;
        }
        else if (cmd.data[5] == 0)
            Send_Data.tx[3] = 0x00; //停止转动
    }

    Send_Data.tx[4] = (int(temp_h) & 0xFF);
    Send_Data.tx[5] = (int(temp_v) & 0xFF);
    Send_Data.tx[6] = Check_Sum();

    UDP_Send(Send_Data.tx, sizeof(Send_Data.tx));
    if (ROS_DEBUG_FLAG)
        ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
                 Send_Data.tx[0], Send_Data.tx[1], Send_Data.tx[2], Send_Data.tx[3], Send_Data.tx[4], Send_Data.tx[5], Send_Data.tx[6]);
}
/**************************************
Date: 2021/10/04
功能: 水平角度控制函数
***************************************/
void ptz_driver::Hori_Control(const std_msgs::Float32MultiArray &cmd)
{
    float temp;
    Send_Data.tx[0] = 0xFF;
    Send_Data.tx[1] = addr;
    Send_Data.tx[2] = 0;
    Send_Data.tx[3] = 0x4B; //水平方向
    temp = cmd.data[1] * 100;

    Send_Data.tx[4] = (int(temp) >> 8 & 0xFF);
    Send_Data.tx[5] = (int(temp) & 0xFF);
    Send_Data.tx[6] = Check_Sum();

    UDP_Send(Send_Data.tx, sizeof(Send_Data.tx));
    if (ROS_DEBUG_FLAG)
        ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
                 Send_Data.tx[0], Send_Data.tx[1], Send_Data.tx[2], Send_Data.tx[3], Send_Data.tx[4], Send_Data.tx[5], Send_Data.tx[6]);
    isSend_H = true;
}

/**************************************
Date: 2021/10/04
功能: 垂直角度控制函数
***************************************/
void ptz_driver::Vert_Control(const std_msgs::Float32MultiArray &cmd)
{
    float temp;
    Send_Data.tx[0] = 0xFF;
    Send_Data.tx[1] = addr;
    Send_Data.tx[2] = 0;
    Send_Data.tx[3] = 0x4D; //垂直方向

    //限制角度
    if (cmd.data[2] > 60)
        temp = 60 * 100;
    else if (cmd.data[2] < -60)
        temp = -60 * 100;

    temp = cmd.data[2] * 100;
    Send_Data.tx[4] = (int(temp) >> 8 & 0xFF);
    Send_Data.tx[5] = (int(temp) & 0xFF);
    Send_Data.tx[6] = Check_Sum();

    UDP_Send(Send_Data.tx, sizeof(Send_Data.tx));
    if (ROS_DEBUG_FLAG)
        ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
                 Send_Data.tx[0], Send_Data.tx[1], Send_Data.tx[2], Send_Data.tx[3], Send_Data.tx[4], Send_Data.tx[5], Send_Data.tx[6]);
    isSend_V = true;
}

/**************************************
Date: January 28, 2021
Function: Publish temperate-related information
功能: 发布角度相关信息
***************************************/
void ptz_driver::Publish_motion()
{
    std_msgs::Float32MultiArray motion_msgs; //定义温度发布话题的数据类型
    motion_msgs.data.resize(4);

    motion_msgs.data[0] = angle_h;
    motion_msgs.data[1] = angle_v;
    motion_msgs.data[2] = speed_h;
    motion_msgs.data[3] = speed_v;

    motion_publisher.publish(motion_msgs);
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 计算CRC校验值
***************************************/
unsigned char ptz_driver::Check_Sum()
{
    unsigned char check_sum = 0, j;

    for (j = 1; j < 6; j++)
    {
        check_sum = Send_Data.tx[j] + check_sum;
    }
    check_sum = check_sum & 0x00FF;

    return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void ptz_driver::Control()
{
    // ros::Rate r(10); // 10 hz

    // ros::Duration(1).sleep(); // sleep for half a second


    while (ros::ok())
    {

        if (Get_Data())
        {
            Publish_motion();
        }

        ros::spinOnce(); //The loop waits for the callback function //循环等待回调函数
        //r.sleep();
    }
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据
***************************************/
bool ptz_driver::Get_Data()
{
    short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0; //Intermediate variable //中间变量
    int len, t;
    uint32_t temp, checkSum = 0;
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
    t = recvfrom(fd, Receive_Data_Pr, sizeof(Receive_Data_Pr), 0, (struct sockaddr *)&addr_from, (socklen_t *)&len);
    //View the received raw data directly and debug it for use//直接查看接收到的原始数据，调试使用
    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    //Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],
    //Receive_Data_Pr[7],Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13]);

    for (j = 1; j < 6; j++)
    {
        checkSum = Receive_Data_Pr[j] + checkSum;
    }
    checkSum = checkSum & 0x00FF;

    //ROS_INFO("%x", checkSum);
    //Record the position of the head and tail of the frame //记录帧头帧尾位置
    for (j = 0; j < 7; j++)
    {

        if (Receive_Data_Pr[j] == FRAME_HEADER)
            Header_Pos = j;
        else if (Receive_Data_Pr[j] == checkSum)
            Tail_Pos = j;

        if (Receive_Data_Pr[Tail_Pos - 6] == 0xFF)
        {
            Header_Pos = 0;
        }
        else if (Receive_Data_Pr[Header_Pos + 6] == checkSum)
        {
            Tail_Pos = 6;
        } //避免头尾多次匹配
    }

    //ROS_INFO("%x-%x", Header_Pos, Tail_Pos);

    if (Tail_Pos == (Header_Pos + 6))
    {
        //If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
        //如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
        // ROS_INFO("1----");
        memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else
    {
        //其它情况则认为数据包有错误
        // In other cases, the packet is considered to be faulty
        // ROS_INFO("3----");
        return false;
    }

    //Check receive_data.rx for debugging use //查看Receive_Data.rx，调试使用
    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
    //Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6]);

    Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
    Receive_Data.Frame_Tail = Receive_Data.rx[6];   //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

    if (Receive_Data.Frame_Header == FRAME_HEADER) //Judge the frame header //判断帧头
    {
        if (Receive_Data.Frame_Tail == checkSum) //Judge the end of the frame //判断帧尾
        {

            //ROS_INFO("%x-%x-%x-%x-%x-%x-%x",
            //         Receive_Data.rx[0], Receive_Data.rx[1], Receive_Data.rx[2], Receive_Data.rx[3], Receive_Data.rx[4], Receive_Data.rx[5], Receive_Data.rx[6]);

            switch (Receive_Data.rx[3])
            {
            case 0x59:
                angle_h = float(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) / 100;
                break;
            case 0x5b:
                if ((Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) > 0x8CA0)
                {
                    angle_v = float((~(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) - 1) & 0xFFFF) / 100;
                    angle_v = -angle_v;
                }
                else
                {
                    angle_v = float(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) / 100;
                }
                break;
            default:
                break;
            }

            if (Receive_Data.rx[2] == 0xC5) //读取角度
            {

                switch (Receive_Data.rx[3])
                {
                case 0x02:
                    isSend_H = false;

                    break;
                case 0x03:
                    isSend_V = false;
                    break;

                default:
                    break;
                }
            }
            else if (Receive_Data.rx[2] == 0xd0) //读取转速
            {

                if (Receive_Data.rx[3] == 0x03)
                {
                    speed_h = float(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) / 100;
                }
                else if (Receive_Data.rx[3] == 0x04)
                {

                    speed_v = float(Receive_Data.rx[4] << 8 | Receive_Data.rx[5]) / 100;
                }
            }
            return true;
        }
    }
    return false;
}

void ptz_driver::init_udp()
{
    ROS_INFO("Connect to %s,Port: %d", DSET_IP_ADDRESS, DEST_PORT);
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1)
    {
        ROS_INFO_STREAM("socket create error!");
        exit(-1);
    }
    ROS_INFO("socket fd=%d", fd);

    /* Enable address reuse */
    int on = 1;
    int ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    addr_to.sin_family = AF_INET;
    addr_to.sin_port = htons(DEST_PORT);
    addr_to.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);

    addr_from.sin_family = AF_INET;
    addr_from.sin_port = htons(SRC_PORT);                  //获得本地端口
    addr_from.sin_addr.s_addr = inet_addr(SRC_IP_ADDRESS); //获得本机地址

    r = bind(fd, (struct sockaddr *)&addr_from, sizeof(addr_from)); //绑定操作

    if (r == -1)
    {
        ROS_INFO_STREAM("Bind error!");
        close(fd);
        exit(-1);
    }
    ROS_INFO_STREAM("Bind successfully.");

    // 设置超时
    struct timeval timeout;
    timeout.tv_sec = 0.5; //秒
    timeout.tv_usec = 0;  //微秒
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    {
        ROS_INFO_STREAM("setsockopt failed");
    }
}

void ptz_driver::init_ptz()
{
    int len;
    uint8_t temp[7]{0xff, ptz_addr, 0xe1, 0x01, 0, 0x64, 0x47}; //角度实时回传
    uint8_t temp1[7]{0xff, ptz_addr, 0xc5, 0x01, 0, 0, 0xc7};   //角度定位回传

    uint8_t temp2[7]{0xff, ptz_addr, 0xdc, 0x01, 0, 0x64, 0x42};   //转速实时回传

    len = sendto(fd, temp, sizeof(temp), 0, (struct sockaddr *)&addr_to, sizeof(addr_to));

    if (len == -1)
    {
        ROS_INFO_STREAM("Send falure!");
    }
    else
    {
        ROS_INFO_STREAM("PTZ  Initialized Succesfuly");
    }

    len = sendto(fd, temp2, sizeof(temp2), 0, (struct sockaddr *)&addr_to, sizeof(addr_to));

    if (len == -1)
    {
        ROS_INFO_STREAM("Send falure!");
    }
    else
    {
        ROS_INFO_STREAM("PTZ Speed Feedback Initialized Succesfuly");
    }

    len = sendto(fd, temp1, sizeof(temp1), 0, (struct sockaddr *)&addr_to, sizeof(addr_to));

    if (len == -1)
    {
        ROS_INFO_STREAM("Send falure!");
    }
    else
    {
        ROS_INFO_STREAM("PTZ AngleSet Feedback Initialized Succesfuly");
    }

    
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
ptz_driver::ptz_driver()
{
    //Clear the data
    //清空数据
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    memset(&Send_Data, 0, sizeof(Send_Data));

    ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
    ros::Rate loop_rate(10);
    //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
    //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
    private_nh.param<std::string>("ptz_cmd_vel", ptz_cmd_vel, "ptz_cmd_vel");

    //temper_publisher = n.advertise<std_msgs::Float32>("boardtemper", 100); //Create a battery-voltage topic publisher //创建温度话题发布者

    motion_publisher = n.advertise<std_msgs::Float32MultiArray>("ptz_motion", 100); //发布云台角度数据

    //Set callback function
    //回调函数设置
    //panel_set  = n.subscribe(panel_cmd, 100, &panelboard_turn_on::Cmd_Vel_Callback, this);
    ptz_cmd = n.subscribe(ptz_cmd_vel, 100, &ptz_driver::Cmd_Vel_Callback, this);

    ROS_INFO_STREAM("Initializing UDP");

    init_udp();

    ROS_INFO_STREAM("Initializing PTZ");
    init_ptz();

    ROS_INFO_STREAM("Node ready"); //Prompt message //提示信息
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
ptz_driver::~ptz_driver()
{
    //Sends the stop motion command to the lower machine before the turn_on_robot object ends
    //对象turn_on_robot结束前向下位机发送停止运动命令

    Send_Data.tx[0] = FRAME_HEADER;
    // Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[7] = 0;
    close(fd);
    ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptz_driver"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称

    ptz_driver ptz_driver;                          //Instantiate an object //实例化一个对象
    ROS_INFO("ptz_driver_udp node has turned on "); //Prompt that the ptz node has been created //提示已创建ptz节点
    ptz_driver.Control();                           //Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
}