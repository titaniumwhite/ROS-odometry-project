#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/* Do we really need a policy??? Idk... */
//typedef message_filters::sync_policies::ExactTime<std_msgs::Float64, std_msgs::Float64, std_msgs::Float64, std_msgs::Float64> MySyncPolicy;

void callback(const std_msgs::Float64ConstPtr& msg1, const std_msgs::Float64ConstPtr& msg2,
              const std_msgs::Float64ConstPtr& msg3, const std_msgs::Float64ConstPtr& msg4) {

  ROS_INFO ("Received: [%f, %f, %f, %f]", msg1->data, msg2->data, msg3->data, msg4->data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    message_filters::Subscriber<std_msgs::Float64> sub1(n, "/speed_fl", 1);
    message_filters::Subscriber<std_msgs::Float64> sub2(n, "/speed_fr", 1);
    message_filters::Subscriber<std_msgs::Float64> sub3(n, "/speed_rl", 1);
    message_filters::Subscriber<std_msgs::Float64> sub4(n, "/speed_rr", 1);
  
    message_filters::TimeSynchronizer<std_msgs::Float64, std_msgs::Float64, 
                                      std_msgs::Float64, std_msgs::Float64> sync(sub1, sub2, sub3, sub4, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}