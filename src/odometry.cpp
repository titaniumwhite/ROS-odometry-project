#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <project_robotics/dynrecConfig.h>
#include <dynamic_reconfigure/server.h>

#define GEAR_RATIO 0.02615575                 
#define RPM_TO_RADS 0.104719755
#define RADIUS 0.1575
#define APPARENT_BASELINE 1.03334887          


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
  Pose prev_pose;
  Velocity velocity;

  double prev_time;

  int method; // 0 for Eurler, 1 for Runge-Kutta
  bool still_not_set; // to avoid setting initial pose repeatedly

  ros::NodeHandle skid_steering_node;
  ros::Publisher skid_steering_pub;
  
  boost::shared_ptr<geometry_msgs::PoseStamped const> initial_pose_shared;
  geometry_msgs::PoseStamped initial_pose;

public:

  skid_steering() {

    skid_steering_pub = skid_steering_node.advertise<nav_msgs::Odometry>("/Odometry", 50);

    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.theta = 0;

    velocity.linear = 0;
    velocity.angular = 0;

    prev_time = 0;

    method = 0;
    still_not_set = true;
  }


  void set_initial_pose(double x, double y, double theta) {
    if(this->still_not_set){
      this->prev_pose.x = x;
      this->prev_pose.y = y;
      this->prev_pose.theta = theta;
      this->still_not_set = false;
    }
  }

  void euler_integration(Velocity *velocity, double current_time) {
    double delta_time = current_time - prev_time;
    current_pose.x = prev_pose.x + velocity->linear * delta_time * cos(prev_pose.theta);
    current_pose.y = prev_pose.y + velocity->linear * delta_time * sin(prev_pose.theta);
    current_pose.theta = prev_pose.theta + velocity->angular * delta_time;

    ROS_INFO ("EULER Position [x, y, theta] [%f, %f, %f]", current_pose.x, current_pose.y, current_pose.theta);
    
    prev_pose.x = current_pose.x;
    prev_pose.y = current_pose.y;
    prev_pose.theta = current_pose.theta;
    prev_time = current_time;
  }

  void runge_kutta_integration(Velocity *velocity, double current_time) {
    double delta_time = current_time - prev_time;
    current_pose.x = prev_pose.x + velocity->linear * delta_time * cos(prev_pose.theta + ((velocity->angular * delta_time) / 2 ));
    current_pose.y = prev_pose.y + velocity->linear * delta_time * sin(prev_pose.theta + ((velocity->angular * delta_time) / 2 ));
    current_pose.theta = prev_pose.theta + velocity->angular * delta_time;

    ROS_INFO("RUKKA Position [x, y, theta] [%f, %f, %f]", current_pose.x, current_pose.y, current_pose.theta);

    prev_pose.x = current_pose.x;
    prev_pose.y = current_pose.y;
    prev_pose.theta = current_pose.theta;
    prev_time = current_time;
  }

  void publish_odometry(Velocity *velocity) {
    nav_msgs::Odometry odometry;

    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "odom";

    odometry.pose.pose.position.x = current_pose.x;
    odometry.pose.pose.position.y = current_pose.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_pose.theta);

    odometry.child_frame_id = "base_link";
    odometry.twist.twist.linear.x = velocity->linear;
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.linear.z = 0;
    odometry.twist.twist.angular.z = 0;
    odometry.twist.twist.angular.y = 0;
    odometry.twist.twist.angular.z = velocity->angular;

    // ROS_INFO ("My linear  velocity is : [%f]", odometry.twist.twist.linear.x);  
    // ROS_INFO ("My angular velocity is : [%f]\n", odometry.twist.twist.angular.z);

    skid_steering_pub.publish(odometry);
  }

  double get_theta_from_quaternion(tf::Quaternion q) {
    double theta;
  
    tf::Matrix3x3 m(q);  
    m.getRPY(*std::unique_ptr<double>(new double),
             *std::unique_ptr<double>(new double),
              theta);

    return theta;
  } 

  void set_integration_method(int m) {
    if (m == 0 || m == 1) method = m;
  }

  void select_integration(Velocity *velocity, double current_time) {
    if (method == 0) euler_integration(velocity, current_time);
    else if (method == 1) runge_kutta_integration(velocity, current_time);
  }

};

class tf_sub_pub {

public:

  tf_sub_pub() {
    tf_sub = tf_node.subscribe("/Odometry", 500, &tf_sub_pub::callback, this); 
    
  } 


  void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) ); 
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  }


private:
  ros::NodeHandle tf_node; 
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Subscriber tf_sub;  

};


void angular_velocity_estimator(Wheels_rpm *rpm, Velocity *velocity){

  // get an average left wheels rpm, also taking into account the reduction gear
  double left_wheels_avg_rpm  = - ((rpm->fl + rpm->rl) * GEAR_RATIO ) / 2;
  double right_wheels_avg_rpm =   ((rpm->fr + rpm->rr) * GEAR_RATIO ) / 2;

  double left_avg_velocity  = left_wheels_avg_rpm  * RADIUS * RPM_TO_RADS;
  double right_avg_velocity = right_wheels_avg_rpm * RADIUS * RPM_TO_RADS;


  velocity->linear  = ( left_avg_velocity + right_avg_velocity ) / (2.0);
  velocity->angular = ( right_avg_velocity - left_avg_velocity ) / (APPARENT_BASELINE);                                                              
}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4, 
              const nav_msgs::Odometry::ConstPtr& msg5, Wheels_rpm *wheels_rpm, Velocity *velocity, 
              twist_stamped *my_twist_stamped, skid_steering *my_skid_steering) {

  wheels_rpm->fl = msg1->rpm;
  wheels_rpm->fr = msg2->rpm;
  wheels_rpm->rl = msg3->rpm;
  wheels_rpm->rr = msg4->rpm;

  tf::Quaternion q(
        msg5->pose.pose.orientation.x,
        msg5->pose.pose.orientation.y,
        msg5->pose.pose.orientation.z,
        msg5->pose.pose.orientation.w
  );

  double theta = my_skid_steering->get_theta_from_quaternion(q);
  
  angular_velocity_estimator(wheels_rpm, velocity);
  my_twist_stamped->publish_twist_stamped(velocity);
  my_skid_steering->set_initial_pose(msg5->pose.pose.position.x, msg5->pose.pose.position.y, theta);
  my_skid_steering->select_integration(velocity, msg1->header.stamp.toSec());
  my_skid_steering->publish_odometry(velocity);


  ROS_INFO ("Their  Position [x, y, theta] [%f %f %f]\n", msg5->pose.pose.position.x, msg5->pose.pose.position.y, theta);
  /*
  ROS_INFO ("Their linear velocity is  : [%f]", msg5->twist.twist.linear.x);
  ROS_INFO ("My linear velocity is     : [%f]\n", velocity->linear);  
  ROS_INFO ("Their angular velocity is : [%f]", msg5->twist.twist.angular.z);
  ROS_INFO ("My angular velocity is    : [%f]\n", velocity->angular);
  */  
}

void dynrec_callback(project_robotics::dynrecConfig &config, uint32_t level, skid_steering *my_skid_steering) {
  /*
  * /// Usage of dynamic reconfiguration ///
  *     While running the code, you can reconfigure the integration method by typing the following command
  *     Euler -> 'rosrun dynamic_reconfigure dynparam set /odometry integration_method 0'
  *     Rukka -> 'rosrun dynamic_reconfigure dynparam set /odometry integration_method 1'
  */
  
  my_skid_steering->set_integration_method(config.integration_method);
  ROS_INFO("dynRec request %d", config.integration_method);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    
    Wheels_rpm wheels_rpm;
    twist_stamped *my_twist_stamped;
    Velocity velocity;
    my_twist_stamped = new twist_stamped();

    skid_steering *my_skid_steering;
    my_skid_steering = new skid_steering();

    tf_sub_pub my_tf_sub_pub;    

    ros::NodeHandle sync_node;

    dynamic_reconfigure::Server<project_robotics::dynrecConfig> server;
    dynamic_reconfigure::Server<project_robotics::dynrecConfig>::CallbackType f; 
    f = boost::bind(&dynrec_callback, _1, _2, my_skid_steering);
    server.setCallback(f);

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(sync_node, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(sync_node, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(sync_node, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(sync_node, "motor_speed_rr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub5(sync_node, "scout_odom", 1);

    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      nav_msgs::Odometry> sync(sub1, sub2, sub3, sub4, sub5, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, &wheels_rpm, &velocity, my_twist_stamped, my_skid_steering));

    ros::spin();

    return 0;
}






/*
* DEPRECATA get_initial_pose perché nun me piace

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

      *x = 0;
      *y = 0;
      *theta = 0;

      ROS_INFO("Initial pose: [%f, %f, %f]", *x, *y, *theta);
  
    }
  }
*/
