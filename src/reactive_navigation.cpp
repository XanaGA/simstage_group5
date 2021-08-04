#include <math.h> 

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <stack>
#include <tf/transform_listener.h>

#include "nav_msgs/Odometry.h"


class Robot{

  protected:
    // Declare the nested class Option
    class Option{
      protected:
        //Declare the variables for the pose and orientation
        float x, y, yaw;

        //Declare the action of that pose
        std::string action;
    };

    // Declare a stack to store the options
    std::stack<Option> opt_stack;

    // Declare Node Handler
    ros::NodeHandle n;

    // Declare subscribers for the laser and the odometry
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    // Declare a publisher for /cmd_vel
    ros::Publisher cmd_vel_pub; 

    // Declare the variables to store the current pose
    float curr_x, curr_y;
    tf::Quaternion curr_ori;

    // Declare the variables for the distance
    float dist_left, dist_front, dist_right;

    // Temporal variables for the Quaternion elements 
    float x,y,z,w;

    // Declare an array for the current state [left, front, rigth]
    std::array<int, 3> curr_st;
    std::array<int, 3> new_st;

    // Declare a threshold
    float thresh = 0.5;

    // Create a message for moving forward
    geometry_msgs::Twist move_front;


    
  public:

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      float dist_left = *std::min_element (msg->ranges.begin()+45, msg->ranges.begin()+135);
      float dist_front = *std::min_element (msg->ranges.begin()+136, msg->ranges.begin()+225);
      float dist_right = *std::min_element (msg->ranges.begin()+226, msg->ranges.begin()+315);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg){
      curr_x = odometry_msg->pose.pose.position.x; 
      curr_y = odometry_msg->pose.pose.position.y; 

      x = odometry_msg->pose.pose.orientation.x;
      y = odometry_msg->pose.pose.orientation.y;
      z = odometry_msg->pose.pose.orientation.z;
      w = odometry_msg->pose.pose.orientation.w;

      curr_ori = tf::Quaternion(x,y,z,w); 
    }

    Robot(){
      // Initialize the Node Handler
      n = ros::NodeHandle();

      // Suscribe to the laser and odometry topics
      laser_sub = n.subscribe("/base_scan", 1000, &Robot::laserCallback, this); 
      odom_sub = n.subscribe("/odom", 1000, &Robot::odomCallback, this );

      // Set the subscriber to /cmd_vel
      cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

      // Fill the front message
      move_front.linear.x = 0.5;
      move_front.angular.z = 0.0;
    }

    void curr_state_init(){
      if(dist_left <= thresh){
        curr_st[0] = 1;
      }else{
        curr_st[0] = 0;
      }

      if(dist_right <= thresh){
        curr_st[1] = 1;
      }else{
        curr_st[1] = 0;
      }

      if(dist_front <= thresh){
        curr_st[2] = 1;
      }else{
        curr_st[1] = 0;
      }
    }

    void move(){
      // Publish the message for moving forward
      cmd_vel_pub.publish(move_front);

      do{

        if(dist_left <= thresh){
          new_st[0] = 1;
        }else{
          new_st[0] = 0;
        }

        if(dist_right <= thresh){
          new_st[1] = 1;
        }else{
          new_st[1] = 0;
        }

        if(dist_front <= thresh){
          new_st[2] = 1;
        }else{
          new_st[1] = 0;
        }

        ros::spinOnce();

      }while(curr_st == new_st);

      curr_st = new_st;
    }

    void evaluate_options(){
      // Updates the stack with new options depending on the curr_st
    }

    void m_decition(){

      evaluate_options();

      if(!opt_stack.empty()){
        // Retrieve the first option of the stack
        //Option opt = *opt_stack.top();
        opt_stack.pop();
        
        // Calculations for modifiying the angle

        // Angular velocity positive or negative depending on the calculations
      }else{
        ROS_INFO("You are done!");
      }
    }

    void run(){

      // Allow callbacks
      ros::spinOnce();

      // Initialize the current state
      curr_state_init();

      while (ros::ok())
      {
        // Make a decition
        m_decition();

        // Move to the front until the state changes
        move();

        // Allow callbacks
        ros::spinOnce();

      }
    }
};


int main(int argc, char **argv){
  // Initialize ROS
  ros::init(argc, argv, "reactive_navigation"); 

  // Create the robot object 
  Robot robot;

  // Run it
  robot.run();

  return 0;
}