#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#define GEAR_RATIO 1.375 // value between 1.35 and 1.4, see in "reduction gearbox" or estimate
#define RPM_TO_RADS 0.104719755
#define RADIUS 0.1575
#define BASELINE 28.3

typedef struct data {
  double speed_fl;
  double speed_fr;
  double speed_rl;
  double speed_rr;
} Data;

class twist_stamped {
private:
  ros::NodeHandle twist_stamped_node;
  ros::Publisher twist_stamped_pub;

public:

  twist_stamped() {
    twist_stamped_pub = twist_stamped_node.advertise<geometry_msgs::TwistStamped>("/twist_stamped", 50);
  }

  void publish_twist_stamped(double *linear_velocity, double *angular_velocity) {

    geometry_msgs::TwistStamped twist_stamped;

    twist_stamped.header.stamp = ros::Time::now();
    twist_stamped.header.frame_id = "twist_stamped";

    twist_stamped.twist.linear.x = *linear_velocity;
    twist_stamped.twist.linear.y = 0.0;
    twist_stamped.twist.linear.z = 0.0;

    twist_stamped.twist.angular.x = 0.0;
    twist_stamped.twist.angular.y = 0.0;
    twist_stamped.twist.angular.z = *angular_velocity;

    twist_stamped_pub.publish(twist_stamped);
  }

};



void angular_velocity_estimator(Data *data, double *linear_velocity, double *angular_velocity){
  /* TO DO
  float apparent_baseline;                      // È UNA COSTANTE CHE VA STIMATA
  // aparent_baseline = (2 * y0), where y0 is the difference between the position of the right center
  // of istantaneous rotation and the left center of istantaneous rotation */
  
  // get an average left wheels rpm, also taking into account the reduction gear
  double left_wheel_avg_rpm  = (data->speed_fl + data->speed_rl) / (2 * GEAR_RATIO);
  double right_wheel_avg_rpm = (data->speed_fr + data->speed_rr) / (2 * GEAR_RATIO);

  double left_avg_velocity  = left_wheel_avg_rpm  * RADIUS * RPM_TO_RADS;
  double right_avg_velocity = right_wheel_avg_rpm * RADIUS * RPM_TO_RADS;

  *linear_velocity = (left_avg_velocity + right_avg_velocity) / (2.0);

  //double angular_velocity = ( right_avg_velocity + left_avg_velocity ) / (BASELINE); //estimate apparent_baseline
  //double left_angular_velocity = (linear_velocity - (-BASELINE) * angular_velocity) / (RADIUS);
  //double right_angular_velocity = (linear_velocity - BASELINE * angular_velocity) / (RADIUS); //estimate apparent_baseline
  *angular_velocity = 12321.0;

  //ROS_INFO ("Linear velocity is : [%f]", linear_velocity);  
  //ROS_INFO ("My angular velocity : [%f]\n", angular_velocity);                                                                
}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4, 
              const nav_msgs::Odometry::ConstPtr& msg5, Data *data, twist_stamped *my_twist_stamped) {
  
  double linear_velocity;
  double angular_velocity;

  data->speed_fl = msg1->rpm;
  data->speed_fr = msg2->rpm;
  data->speed_rl = msg3->rpm;
  data->speed_rr = msg4->rpm;

  //ROS_INFO("Their angular velocity: [%f]", msg5->twist.twist.angular.z);
  angular_velocity_estimator(data, &linear_velocity, &angular_velocity);
  my_twist_stamped->publish_twist_stamped(&linear_velocity, &angular_velocity);

  ROS_INFO ("Linear velocity is : [%f]", linear_velocity);  
  ROS_INFO ("My angular velocity : [%f]\n", angular_velocity); 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    
    Data data;
    twist_stamped *my_twist_stamped;
    my_twist_stamped = new twist_stamped();

    ros::NodeHandle sync_node;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(sync_node, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(sync_node, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(sync_node, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(sync_node, "motor_speed_rr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub5(sync_node, "scout_odom", 1);

  
    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      nav_msgs::Odometry> sync(sub1, sub2, sub3, sub4, sub5, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, &data, my_twist_stamped));

    ros::spin();

    return 0;
}