#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>

/* Do we really need a policy??? Idk... */
//typedef message_filters::sync_policies::ExactTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
void angular_velocity_estimator( const float motor_speed_fl, const float motor_speed_fr,
                                 const float motor_speed_rl, const float motor_speed_rr){

  float gear_ratio = 1.375;                     // value between 1.35 and 1.4, see in "reduction gearbox " or estimate
  float rpm_to_rad_per_sec = 0.104719755;       // we'll use it in the conversion later 
  float wheel_radius = 0.1575;                  // all gear_ratio, rpm_ecc. and wheel_radius are const
  /* TO DO
  float apparent_baseline;                      // È UNA COSTANTE CHE VA STIMATA
  // aparent_baseline = (2 * y0), where y0 is the difference between the position of the right center
  // of istantaneous rotation and the left center of istantaneous rotation
  */

  float left_wheel_avg_rpm  = ( motor_speed_fl + motor_speed_rl) / (2 * gear_ratio);   // get an average left wheels rpm, also taking into account the reduction gear
  float right_wheel_avg_rpm = ( motor_speed_fr + motor_speed_rr) / (2 * gear_ratio);

  float left_avg_velocity  = left_wheel_avg_rpm  * wheel_radius * rpm_to_rad_per_sec;
  float right_avg_velocity = right_wheel_avg_rpm * wheel_radius * rpm_to_rad_per_sec;

  float linear_velocity = (left_avg_velocity + right_avg_velocity) / 2;

  // float angular_velocity = (- left_avg_velocity + right_avg_velocity ) / (2 * apparent_baseline) //estimate apparent_baseline

  ROS_INFO ("Linear velocity is : [%f]\n\n", linear_velocity);                                 

}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4) {

  ROS_INFO ("Received: [%f, %f, %f, %f]", msg1->rpm, msg2->rpm, msg3->rpm, msg4->rpm);
  angular_velocity_estimator( msg1->rpm, msg2->rpm, msg3->rpm, msg4->rpm);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  
    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> sync(sub1, sub2, sub3, sub4, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}