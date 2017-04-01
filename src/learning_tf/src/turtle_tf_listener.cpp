#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

#define TF_EXAMPLE 4

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_listener_node");
    ros::NodeHandle nh;

    // Spawn another turtle
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    // Publisher used to command the turtle's velocity
    ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // Create the transform listener
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (nh.ok()) {

        tf::StampedTransform transform;

        try {
            // Change this `1` to a `0` if you want to use the carrot to control
            std::string target_frame = (1) ? "/turtle1" : "/carrot1";

#if TF_EXAMPLE == 1
            // ros::Time(0) just says "give me the latest"
            listener.lookupTransform("/turtle2", target_frame, ros::Time(0), transform);

#elif TF_EXAMPLE == 2
            // Instead of getting the latest tf, let's wait
            ros::Time now = ros::Time::now();
            listener.waitForTransform("/turtle2", target_frame, now, ros::Duration(1.0));
            listener.lookupTransform("/turtle2", target_frame, now, transform);

#elif TF_EXAMPLE == 3
            // Control current pose of turtle2 based on old pose between turtle2 and turtle1.
            // This results in uncontrollable motion.
            ros::Time past = ros::Time::now() - ros::Duration(5.0);
            listener.waitForTransform("/turtle2", target_frame, past, ros::Duration(1.0));
            listener.lookupTransform("/turtle2", target_frame, past, transform);

#elif TF_EXAMPLE == 4
            // Control current pose of turtle2 based on current turtle2 and old turtle1.
            ros::Time now = ros::Time::now();
            ros::Time past = now - ros::Duration(5.0);

            // The world frame must be specified as the frame that does not change over time
            listener.waitForTransform("/turtle2", now, target_frame, past, "/world", ros::Duration(1.0));
            listener.lookupTransform("/turtle2", now, target_frame, past, "/world", transform);

#endif

            // source (from): "/turtle2"
            // target   (to): "/turtle1"
            //
            // So the vector points from turtle2 to turtle1

        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep(); // sleep for 1 second
        }

        // std::cout << transform.getOrigin().x() << std::endl;

        // Now that we have the transformation from turtle2 to turtle1, we can command turtle2 to go to turtle1.
        // This is essentially a P controller since `transform` contains the difference between turtle1 to turtle2.
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);

        // Allow ROS networking to happen
        rate.sleep();
    }

    return 0;
}