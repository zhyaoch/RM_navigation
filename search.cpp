#include"ros/ros.h" 
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include"geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include<vector>
#include<algorithm>
#include<deque>
#include<math.h>
using namespace std;
ros::Publisher vel_pub;
vector<geometry_msgs::PoseStamped> global_Path;
double P_Parameter = 0.5; //jiao zheng xi shu
double a_Parameter = 0.1;  //jia su du
geometry_msgs::Twist current_vel_back; 
void pathCallback(const nav_msgs::Path::ConstPtr& msg){
    geometry_msgs::Twist current_vel,new_vel;
    current_vel = current_vel_back;
     int aim=-1;
     int stop_flag = 0;
     float vel_x,vel_y;
     float vel_len,vel_new_len,vel_P;
     tf::TransformListener listener;
    tf::Quaternion startZ,endZ;
    tf::Vector3 m_vector3;//roll pitch yaw  z x y
    geometry_msgs::PointStamped map_point_start,map_point_end;
    geometry_msgs::PointStamped base_point_start,base_point_end;
    map_point_start.header.frame_id = "map";   //将这个点绑定到雷达坐标系下
    map_point_start.header.stamp = ros::Time();
    map_point_end.header.frame_id = "map";   //将这个点绑定到雷达坐标系下
    map_point_end.header.stamp = ros::Time();

    listener.waitForTransform("base_link","map",ros::Time(0),ros::Duration(3));//ros::Time(0)表示使用缓冲中最新的tf数据
    global_Path.clear();
    for(int i=0;i<msg->poses.size();i++){
        global_Path.push_back(msg->poses[i]);
    }
    ROS_INFO("receive : %d",global_Path.size());
    ROS_INFO("current_vel(x,y,w):(%f,%f,%f)",current_vel.linear.x,current_vel.linear.y,current_vel.angular.z);
    
     if((pow(global_Path[global_Path.size()-1].pose.position.x-global_Path[0].pose.position.x,2)+
        pow(global_Path[global_Path.size()-1].pose.position.x-global_Path[0].pose.position.x,2)+
        pow(global_Path[global_Path.size()-1].pose.position.x-global_Path[0].pose.position.x,2))<1.5){
            stop_flag=1;
        }
    
    for(int i=0;i<global_Path.size();i++){
        if((pow(global_Path[i].pose.position.x-global_Path[0].pose.position.x,2)+
        pow(global_Path[i].pose.position.x-global_Path[0].pose.position.x,2)+
        pow(global_Path[i].pose.position.x-global_Path[0].pose.position.x,2))>1){
            aim=i;
            break;   
        }
    }

    if(aim==-1) aim=global_Path.size()-1;
    map_point_start.point.x = global_Path[0].pose.position.x;
    map_point_start.point.y = global_Path[0].pose.position.y;
    map_point_end.point.x = global_Path[aim].pose.position.x;
    map_point_end.point.y = global_Path[aim].pose.position.y;
    ROS_INFO("(x,y,z_rad):(%f,%f,%f_rad)",map_point_start.point.x,map_point_start.point.y,m_vector3[0]);
    

    listener.transformPoint("base_link",map_point_start,base_point_start);//将map中的点变换到base_link中去
    listener.transformPoint("base_link",map_point_end,base_point_end);//将map中的点变换到base_link中去

    ROS_INFO("(vel,vel_new)=(%f,%f)",vel_len,vel_new_len);
    ROS_INFO("(vel,vel_new)=(%f,%f)",vel_len,vel_new_len);


    vel_len = pow((pow(current_vel.linear.x,2)+pow(current_vel.linear.y,2)),0.5);
    ROS_INFO("(vel1)=(%lf)",vel_len);
    if(stop_flag ){
        vel_len = vel_len*0.5;   // jian su
   } 
   else if((a_Parameter + vel_len)<3) {
    ROS_INFO("++++");
    vel_len = vel_len+a_Parameter ;  //jia su du
;  //zeng jia su du
   }
   ROS_INFO("(vel2)=(%lf)",vel_len);


    vel_P = pow((pow(base_point_end.point.x-base_point_start.point.x,2)+pow(base_point_end.point.y-base_point_start.point.y,2)),0.5);
    vel_x = current_vel.linear.x+ P_Parameter*(base_point_end.point.x-base_point_start.point.x)*vel_len/vel_P;
    vel_y = current_vel.linear.y+ P_Parameter*(base_point_end.point.y-base_point_start.point.y)*vel_len/vel_P;
    vel_new_len = pow(pow(vel_x,2)+pow(vel_y,2),0.5);

   ROS_INFO("(vel,vel_new)=(%f,%f)",vel_len,vel_new_len);
    vel_x = vel_len/vel_new_len*vel_x;
    vel_y = vel_len/vel_new_len*vel_y;
    new_vel.linear.x = vel_x;
    new_vel.linear.y = vel_y;
    ROS_INFO("new_current_vel(x,y,w):(%f,%f)",new_vel.linear.x,new_vel.linear.y);
    vel_pub.publish(new_vel);

    return;
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg){
    current_vel_back = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "new_search");  // 节点名称
	ros::NodeHandle n;

    ros::Subscriber path_sub = n.subscribe("/global_planner_node/path", 1000, pathCallback); 
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 1000, velCallback); 
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::spin();  
    return 0;
}