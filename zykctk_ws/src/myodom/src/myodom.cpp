#include "ros/ros.h" 
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
double x = 0.0,y = 0.0,th = 0.0;  
double vx = 0.0,vy = -0.0,vth = 0.0;

void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
     double delta_x = 0.0, delta_y = 0.0, delta_th = 0.0;
   
    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s 

    if(linear_temp !=0) //只要线速度不为零，就使用L指令控制
    {
        if(linear_temp < 0) 
            { vx = -0.05; vy = 0.0; vth = 0.0;  }                         
        else 
            {vx = 0.05; vy = 0.0; vth = 0.0; }             
    }
    else if (angular_temp !=0) //线速度为零，角速度不为零，那么只使用N指令控制
    {
        //N指令的具体格式<N0#ANGLE#SPD#ACC>
        if(angular_temp < 0)
              { vx = 0.0; vy = 0.0; vth = -0.1; }          
         else
             {  vx = 0.0; vy =0.0; vth = 0.1; }         
    }
    else  //线速度和角速度均为零，使用S指令，停止
    {  vx = 0.0; vy = 0.0; vth = 0.0; } 
}

 int main(int argc, char** argv)
 {
      ros::init(argc, argv, "odometry_publisher");    
      ros::NodeHandle n;
      ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
      tf::TransformBroadcaster odom_broadcaster;

      ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题

      ros::Time current_time, last_time;
      current_time = ros::Time::now();
      last_time = ros::Time::now();

      ros::Rate r(2.0);

      while(n.ok())
      {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
    
        x  += delta_x;
        y  += delta_y;
        th += delta_th;
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;   

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;

        r.sleep();
     }  
}

