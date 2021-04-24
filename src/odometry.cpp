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

typedef struct pose {
  double x;
  double y;
  double theta;
} Pose;

typedef struct rpm {
  double fl;
  double fr;
  double rl;
  double rr;
} Wheels_rpm;

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

class skid_steering {

private:
  Pose current_pose;

  double linear_velocity;
  double angular_velocity;

  ros::NodeHandle skid_steering_node;
  ros::Publisher skid_steering_pub;

public:


};


void angular_velocity_estimator(Wheels_rpm *rpm, double *linear_velocity, double *angular_velocity){

  // get an average left wheels rpm, also taking into account the reduction gear
  double left_wheel_avg_rpm  = (rpm->fl + rpm->rl) / (2 * GEAR_RATIO);
  double right_wheel_avg_rpm = (rpm->fr + rpm->rr) / (2 * GEAR_RATIO);

  double left_avg_velocity  = left_wheel_avg_rpm  * RADIUS * RPM_TO_RADS;
  double right_avg_velocity = right_wheel_avg_rpm * RADIUS * RPM_TO_RADS;

  *linear_velocity = (left_avg_velocity + right_avg_velocity) / (2.0);

  //double left_angular_velocity = (linear_velocity - (-BASELINE) * angular_velocity) / (RADIUS);
  //double right_angular_velocity = (linear_velocity - BASELINE * angular_velocity) / (RADIUS); //estimate apparent_baseline
  //*angular_velocity = 12321.0;
  *angular_velocity = ( right_avg_velocity + left_avg_velocity ) / (BASELINE); //estimate apparent_baseline


  //ROS_INFO ("Linear velocity is : [%f]", linear_velocity);  
  //ROS_INFO ("My angular velocity : [%f]\n", angular_velocity);                                                                
}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4, 
              const nav_msgs::Odometry::ConstPtr& msg5, Wheels_rpm *rpm, twist_stamped *my_twist_stamped) {
  
  double linear_velocity;
  double angular_velocity;

  rpm->fl = msg1->rpm;
  rpm->fr = msg2->rpm;
  rpm->rl = msg3->rpm;
  rpm->rr = msg4->rpm;

  ROS_INFO("Their angular velocity: [%f]", msg5->twist.twist.angular.z);
  angular_velocity_estimator(data, &linear_velocity, &angular_velocity);
  my_twist_stamped->publish_twist_stamped(&linear_velocity, &angular_velocity);

  //ROS_INFO ("Linear velocity is : [%f]", linear_velocity);  
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