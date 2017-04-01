#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
    static tf::TransformBroadcaster br;
    
    // Create the transform
    tf::Transform transform;

    // Set the translation from the world origin
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0));

    // Add in the attitude
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);

    // Mark the transform with a timestamp and link it to the turtle
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_broadcaster_node");
    ros::NodeHandle nh;
    
    // Get the turtle name
    if (argc != 2) {
        ROS_ERROR("Give me a turtle name as an argument and I'll be happy");
        return -1;
    }
    turtle_name = argv[1];

    // Create a pose subscriber that listens to the turtle's pose
    ros::Subscriber sub = nh.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}