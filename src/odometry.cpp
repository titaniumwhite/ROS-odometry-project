#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


#define GEAR_RATIO 1.375 // value between 1.35 and 1.4, see in "reduction gearbox" or estimate
#define RPM_TO_RADS 0.104719755
#define RADIUS 0.1575
#define BASELINE 28.753  

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

typedef struct velocity {
  double linear;
  double angular;
} Velocity;

class twist_stamped {
private:
  ros::NodeHandle twist_stamped_node;
  ros::Publisher twist_stamped_pub;

public:

  twist_stamped() {
    twist_stamped_pub = twist_stamped_node.advertise<geometry_msgs::TwistStamped>("/twist_stamped", 50);
  }

  void publish_twist_stamped(Velocity *velocity) {

    geometry_msgs::TwistStamped twist_stamped;

    twist_stamped.header.stamp = ros::Time::now();
    twist_stamped.header.frame_id = "twist_stamped";

    twist_stamped.twist.linear.x = velocity->linear;
    twist_stamped.twist.linear.y = 0.0;
    twist_stamped.twist.linear.z = 0.0;

    twist_stamped.twist.angular.x = 0.0;
    twist_stamped.twist.angular.y = 0.0;
    twist_stamped.twist.angular.z = velocity->angular;

    twist_stamped_pub.publish(twist_stamped);
  }

};

class skid_steering {
private:
  Pose current_pose;

  double linear_velocity;
  double angular_velocity;

  double current_time;
  double prev_time;

  int method; // 0 for Eurler, 1 for Runge-Kutta

  ros::NodeHandle skid_steering_node;
  ros::Publisher skid_steering_pub;
  
  boost::shared_ptr<geometry_msgs::PoseStamped const> initial_pose_shared;
  geometry_msgs::PoseStamped initial_pose;

public:

  skid_steering() {
    get_initial_pose(&current_pose.x, &current_pose.y, &current_pose.theta);

    linear_velocity = 0;
    angular_velocity = 0;

    skid_steering_pub = skid_steering_node.advertise<nav_msgs::Odometry>("/odometry", 50);
  }

  void get_initial_pose(double *x, double *y, double *theta) {
    initial_pose_shared = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gt_pose", skid_steering_node);

    if (initial_pose_shared != NULL) {
      initial_pose = *initial_pose_shared;

      tf::Quaternion q(
        initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y,
        initial_pose.pose.orientation.z,
        initial_pose.pose.orientation.w
      );

      tf::Matrix3x3 m(q);
      m.getRPY(*std::unique_ptr<double>(new double),
               *std::unique_ptr<double>(new double),
               *theta);

      *x = initial_pose.pose.position.x;
      *y = initial_pose.pose.position.y;

      ROS_INFO("Initial pose: [%f, %f, %f]", current_pose.x, current_pose.y, current_pose.theta);
    }
  }

};

void euler_integration() {

}


void angular_velocity_estimator(Wheels_rpm *rpm, Velocity *velocity){

  // get an average left wheels rpm, also taking into account the reduction gear
  double left_wheel_avg_rpm  = (rpm->fl + rpm->rl) / (2 * GEAR_RATIO);
  double right_wheel_avg_rpm = (rpm->fr + rpm->rr) / (2 * GEAR_RATIO);

  double left_avg_velocity  = left_wheel_avg_rpm  * RADIUS * RPM_TO_RADS;
  double right_avg_velocity = right_wheel_avg_rpm * RADIUS * RPM_TO_RADS;

  velocity->linear = (left_avg_velocity + right_avg_velocity) / (2.0);

  //double left_angular_velocity = (linear_velocity - (-BASELINE) * angular_velocity) / (RADIUS);
  //double right_angular_velocity = (linear_velocity - BASELINE * angular_velocity) / (RADIUS); //estimate apparent_baseline
  //*angular_velocity = 12321.0;
  velocity->angular = ( right_avg_velocity + left_avg_velocity ) / (BASELINE); //estimate apparent_baseline


  //ROS_INFO ("Linear velocity is : [%f]", linear_velocity);  
  //ROS_INFO ("My angular velocity : [%f]\n", angular_velocity);                                                                
}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4, 
              const nav_msgs::Odometry::ConstPtr& msg5, Wheels_rpm *wheels_rpm, twist_stamped *my_twist_stamped,
              Velocity *velocity) {

  wheels_rpm->fl = msg1->rpm;
  wheels_rpm->fr = msg2->rpm;
  wheels_rpm->rl = msg3->rpm;
  wheels_rpm->rr = msg4->rpm;

  angular_velocity_estimator(wheels_rpm, velocity);
  my_twist_stamped->publish_twist_stamped(velocity);

  /*ROS_INFO ("Linear velocity is : [%f]", velocity->linear);  
  ROS_INFO("Their angular velocity: [%f]", msg5->twist.twist.angular.z);
  ROS_INFO ("My angular velocity : [%f]\n", velocity->angular);*/ 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    
    Wheels_rpm wheels_rpm;
    twist_stamped *my_twist_stamped;
    Velocity velocity;
    my_twist_stamped = new twist_stamped();

    skid_steering *my_skid_steering;
    my_skid_steering = new skid_steering();    

    ros::NodeHandle sync_node;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(sync_node, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(sync_node, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(sync_node, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(sync_node, "motor_speed_rr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub5(sync_node, "scout_odom", 1);

    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      nav_msgs::Odometry> sync(sub1, sub2, sub3, sub4, sub5, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, &wheels_rpm, my_twist_stamped, &velocity));

    ros::spin();

    return 0;
}