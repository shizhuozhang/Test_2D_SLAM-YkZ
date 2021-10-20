/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
// 3.发布里程计主题/odm
3.发布超声波检测到的障碍信息
原代码的串口通信说明：
// 1.写入串口
// （1）内容：左右轮速度，单位为mm/s
// （2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
// 2.读取串口
// （1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
// （2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
串口通信指令说明：
向平台发送的控制指令有：
    L指令  用24字节的Linear_data存放
        功能：平台沿直线轨迹运动
        具体内容：<L0#SPD#ACC#ALPH#W>  即<指令标识符 # 运动线速度 # 运动线加速度 # 平台系X轴与地面系的初始夹角 # 运动角速度>
                注意： 运动的加速度是用于电机启动和刹车的，实际应用时可作为固定参数给定；速度单位为米/秒；角度单位为弧度，PI值取3.14159
        实际应用：（1）不涉及转动的纯粹的直线运动：前进<L0#0.2#0.5#0#0>  后退<L0#0.2#0.5#3.14159#0>  
                （2）涉及转动的直线运动：前进和逆时针转动合成左前方向的运动<L0#0.2#0.5#0#0.7854>         前进和顺时针转动合成右前方向的运动<L0#0.2#0.5#0#-0.7854>
                                     后退和顺时针转动合成左后方向的运动<L0#0.2#0.5#3.14159#-0.7854>  后退和逆时针转动合成右后方向的运动<L0#0.2#0.5#3.14159#0.7854>
    N指令  用19字节的Angular_data存放
        功能：使平台原地旋转固定角度
        具体内容：<M0#SPD#ACC>  即<指令标识符 # 转动速度  #转动加速度> （注意，这里是转动速度和转动加速度，应由转动角速度和平台的半径计算得到）
        实际应用：控制平台逆时针转动一定的角度：<M0#-0.05#0.1>
                控制平台逆时针转动一定的角度：<M0#0.05#0.1>
    S指令  用4字节的Stop_data存放
        功能：使平台停止一切运动
        具体内容：<S0>  只有指令标识符，没有参数
向平台的获取信息指令与获取的信息内容
    T指令
        功能：获取超声波和红外数据信息
        具体内容：获取超声波信息： 发送：<T0>  返回：<T0#33.5#50.5#103.0#67.2#33.5#50.5>  分别为6个超声波测量得到的障碍距离信息（单位：厘米）
                获取红外信息：   发送：<T0>  返回：<T1#1#1#1#0#1#0>  0代表有障碍物，1代表没有障碍物
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
float ratio = 0.10f ;   //转速转换比例，执行速度调整比例
float PI =3.14159f ; //PI值
float D  = 0.50f ;    //地盘直径，单位是m


/****************************************************/
unsigned char data_start=0x3c;  //“<"字符
unsigned char data_terminal=0x3e;  //“>"字符
//unsigned char speed_data[10]={0};   //要发给串口的数据
unsigned char Linear_data[24]={0};
unsigned char Angular_data[10]={0};
//发送给下位机的左右轮速度，里程计的坐标和方向
// union floatData //union的作用为实现char数组和float之间的转换
// {
//     float d;
//     unsigned char data[4];
// }vel_linear,vel_angular,vel_acc,alph_span;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
unsigned char Stop_data[4]={0x3c,0X53,0X30,0x3E};// 对应ASCII码<S0>
string rec_buffer;  //串口数据接收变量
string send_buffer;  //串口数据发送变量
/************************************************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    string port("/dev/ttyUSB0");    //小车串口号
    unsigned long baud = 115200;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s  
    
    if(linear_temp !=0) //只要线速度不为零，就使用L指令控制
    {
       //L指令的格式 <L0#SPD#ACC#ALPH#W> 
        if(linear_temp < 0) 
        {
            if(angular_temp == 0)
              {
                 send_buffer = "<L0#0.05#0.05#3.14159#0> ";//小于0表示向X轴的负方向运动，此时初始角度为PI
                   my_serial.write (send_buffer);//串口数据发送变量 
              }  
            else 
                 my_serial.write(Stop_data,4);  //串口发送停止指令           
        }         
        else 
        {
            if(angular_temp == 0)
            {
                send_buffer = "<L0#0.05#0.05#0#0>";//小于0表示向X轴的负方向运动，
                my_serial.write (send_buffer);//串口数据发送变量 
            }              
            else 
                 my_serial.write(Stop_data,4);  //串口发送停止指令
        }                  
    }
    else if (angular_temp !=0) //线速度为零，角速度不为零，使用M指令控制
    {
        //M指令<M0#SPD#ACC>
        if(angular_temp < 0)
            send_buffer = "<M0#-0.05#0.1>"; //以0.1米每秒的速度顺时针旋转        
         else
             send_buffer =  "<M0#0.05#0.1>";//以0.1米每秒的速度逆时针旋转
         my_serial.write (send_buffer);//串口数据发送变量       
    }
    else  //线速度和角速度均为零，使用S指令，停止
        my_serial.write(Stop_data,4);  //串口发送停止指令
}

int main(int argc, char **argv)
{
    string port("/dev/ttyUSB0");//小车串口号
    unsigned long baud = 115200;//小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::spin();
    return 0;
}
