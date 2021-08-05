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
      public:
        // Declare the variables for the pose and orientation
        float x, y, yaw;

        // Declare the variable for the orientation as quaternion
        tf::Quaternion ori; 

        // Declare the action of that pose:
        // RIGHT  LEFT  FRONT BACK 
        // -90     90     0    180
        float action;
    };

    // Declare a stack (inplemented as a vector) to store the options
    std::stack<Option, std::vector<Option> > opt_stack;

    // Declare Node Handler
    ros::NodeHandle n;

    // Declare subscribers for the laser and the odometry
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    // Declare a publisher for /cmd_vel
    ros::Publisher cmd_vel_pub; 

    // Declare the variables to store the current pose
    float curr_x, curr_y, curr_yaw;
    tf::Quaternion curr_ori;

    // Declare the variables for the distance
    float dist_left, dist_front, dist_right;

    // Temporal variables for the Quaternion elements 
    float x,y,z,w;

    // Declare an array for the current state [left, front, rigth]
    std::array<int, 3> curr_st;
    std::array<int, 3> new_st;

    // Declare a threshold
    float thresh = 0.21;

    // Create a message for moving forward
    geometry_msgs::Twist move_front;


    
  public:
    // CALLBACKS
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

      // tf::Quaternion q = tf::Quaternion(x,y,z,w);
      // tf::Quaternion r = tf::Quaternion(x,y,z,w);
      // tf::Quaternion p = q*r;

      static double siny_cosp, cosy_cosp;
      siny_cosp = 2 * (w * z + x * y);
      cosy_cosp = 1 - 2 * (y * y + z * z);
      curr_yaw = atan2(siny_cosp, cosy_cosp);
    }

    // BASIC OPERATIONS

    void rotate_angle(float angle_to_rotate){
      // Message to be published
      static auto msg = geometry_msgs::Twist();

      // Calculate the target angle
      float t_angle;
      t_angle = fmod(curr_yaw + angle_to_rotate, 360.);

      while(abs(curr_yaw - t_angle) > 0.1){
        msg.linear.x = 0.;
        // The direction will be 1 for turning left & -1 for turning right
        msg.angular.z = abs(angle_to_rotate)/angle_to_rotate * 0.5;
        this->cmd_vel_pub.publish(msg);
        ros::spinOnce();
      }
    }

    void rotate_target(float target){
      // Message to be published
      auto msg = geometry_msgs::Twist();

      // Calculate the direction
      float difference = target - curr_yaw;
      
      int direction = abs(difference) < 180;

      while(abs(curr_yaw - target) > 0.1){
        msg.linear.x = 0.;
        // The direction will be 1 for turning left & -1 for turning right
        msg.angular.z = direction * 0.5;
        this->cmd_vel_pub.publish(msg);
        ros::spinOnce();
      }
    }

    void move_to_target(float target_x, float target_y ){
        // Message to be published
        auto msg = geometry_msgs::Twist();
        while(sqrt(pow((target_x - curr_x), 2) + pow((target_y - curr_y), 2)) > 0.5){
            msg.linear.x = 0.5;
            msg.angular.z = 0.;
            this->cmd_vel_pub.publish(msg);
            ros::spinOnce();
        }
    }

    // CONSTRUCTOR
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

    // CLASS METHODS
    void curr_state_init(){

      update_st_array(curr_st);

    }

    void update_st_array(std::array<int, 3> arr){
      if(dist_left <= thresh){
          arr[0] = 1;
        }else{
          arr[0] = 0;
        }

        if(dist_right <= thresh){
          arr[1] = 1;
        }else{
          arr[1] = 0;
        }

        if(dist_front <= thresh){
          arr[2] = 1;
        }else{
          arr[1] = 0;
        }
    }

    void move(){
      
      do{
        // Publish the message for moving forward
        cmd_vel_pub.publish(move_front);

        update_st_array(new_st);

        ros::spinOnce();

      }while(curr_st == new_st);

      curr_st = new_st;
    }

    bool evaluate_options(){
      // Updates the stack with new options depending on the curr_st
      // Returns True if new options (different from going BACK) are added, False if not
      bool res = false;

      if(!opt_stack.empty()){
        Robot::Option back;
        back.x = curr_x;
        back.y = curr_y;
        back.yaw = curr_yaw;
        back.action = 180;

        opt_stack.push(back);
      }

      if(!new_st[0]){
        Robot::Option left;
        left.x = curr_x;
        left.y = curr_y;
        left.yaw = curr_yaw;
        left.action = -90;

        opt_stack.push(left);

        res = true;
      }

      if(!new_st[1]){
        Robot::Option front;
        front.x = curr_x;
        front.y = curr_y;
        front.yaw = curr_yaw;
        front.action = 0;

        opt_stack.push(front);

        res = true;
      }

      if(!new_st[2]){
        Robot::Option right;
        right.x = curr_x;
        right.y = curr_y;
        right.yaw = curr_yaw;
        right.action = 90;

        opt_stack.push(right);

        res = true;
      }

      return res;
    }

    void m_decition(){
      // We have to distinguish between the case where the option is a new one 
      //or it is revisited 

      if(evaluate_options()){
        Robot::Option opt = opt_stack.top();
        opt_stack.pop();

        rotate_angle(opt.action);

        curr_st = new_st;

      }
      else if(!opt_stack.empty()){
        // Retrieve the top option of the stack
        Robot::Option opt = opt_stack.top();
        opt_stack.pop();

        // Turn back, as in this case we always be returning to a visited node
        rotate_angle(opt.action);

        // Retrieve the next top option
        opt = opt_stack.top();
        opt_stack.pop();

        // Move to the previous node
        move_to_target(opt.x,opt.y);
        
        // Boolean to know if we have arrived at the end of the maze (if the queue is empty)
        bool keep_exploring;
        keep_exploring = 1;

        float target_angle;

        // While the option retrieved is going back, don't "explore"
        while(opt.action = 180){
          
          // Calculate the angle to rotate
          target_angle = fmod(opt.yaw + opt.action,360.);
          
          rotate_target(target_angle);

          // Move to the previous node
          move_to_target(opt.x,opt.y);

          if(!opt_stack.empty()){
            opt = opt_stack.top();
            opt_stack.pop();
          }else{
            keep_exploring=0;
            break;
          }

          ros::spinOnce();
        }

        if(keep_exploring){
          // Rotate and continue "exploring"
          target_angle = fmod(opt.yaw + opt.action, 360.);
          
          rotate_target(target_angle);

          update_st_array(new_st);
          curr_st = new_st;

        }else{
          ROS_INFO("You are done!");
          ros::shutdown();
        }
        
      }
      else{
        ROS_INFO("You are done!");
        ros::shutdown();
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

  // // TESTING
  // robot.m_decition();

  // Run it
  robot.run();

  return 0;
}