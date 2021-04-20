#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robotics_hw1/MotorSpeed.h>

/* Do we really need a policy??? Idk... */
//typedef message_filters::sync_policies::ExactTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;

void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1, const robotics_hw1::MotorSpeed::ConstPtr& msg2,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3, const robotics_hw1::MotorSpeed::ConstPtr& msg4) {

  //ROS_INFO ("Received: [%f, %f, %f, %f]", msg1, msg2, msg3, msg4);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "/speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "/speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "/speed_rl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "/speed_rr", 1);
  
    message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, 
                                      robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> sync(sub1, sub2, sub3, sub4, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}