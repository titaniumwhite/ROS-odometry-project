#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>

#define GEAR_RATIO 1.375 // value between 1.35 and 1.4, see in "reduction gearbox" or estimate
#define RPM_TO_RADS 0.104719755
#define RADIUS 0.1575

typedef struct data {
  double speed_fl;
  double speed_fr;
  double speed_rl;
  double speed_rr;
} Data;

void angular_velocity_estimator(Data *data){
  /* TO DO
  float apparent_baseline;                      // È UNA COSTANTE CHE VA STIMATA
  // aparent_baseline = (2 * y0), where y0 is the difference between the position of the right center
  // of istantaneous rotation and the left center of istantaneous rotation */
  
  // get an average left wheels rpm, also taking into account the reduction gear
  double left_wheel_avg_rpm  = (data->speed_fl + data->speed_rl) / (2 * GEAR_RATIO);
  double right_wheel_avg_rpm = (data->speed_fr + data->speed_rr) / (2 * GEAR_RATIO);

  double left_avg_velocity  = left_wheel_avg_rpm  * RADIUS * RPM_TO_RADS;
  double right_avg_velocity = right_wheel_avg_rpm * RADIUS * RPM_TO_RADS;

  double linear_velocity = (left_avg_velocity + right_avg_velocity) / 2;

  // float angular_velocity = (- left_avg_velocity + right_avg_velocity ) / (2 * apparent_baseline) //estimate apparent_baseline

  ROS_INFO ("Linear velocity is : [%f]\n\n", linear_velocity);                                 
  ROS_INFO ("[Left velocity %f :: Right velocity %f]\n\n", left_avg_velocity, right_avg_velocity);                                 
}


void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4, 
              Data *data) {

  data->speed_fl = msg1->rpm;
  data->speed_fr = msg2->rpm;
  data->speed_rl = msg3->rpm;
  data->speed_rr = msg4->rpm;

  //ROS_INFO ("Received: [%f, %f, %f, %f]", msg1->rpm, msg2->rpm, msg3->rpm, msg4->rpm);
  angular_velocity_estimator(data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    Data data;

    ros::NodeHandle n;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  
    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> sync(sub1, sub2, sub3, sub4, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, &data));

    ros::spin();

    return 0;
}