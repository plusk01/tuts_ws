#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_frame_tf_broadcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0); // 10 Hz
    while (nh.ok()) {
        // Create the carrot, always to the right 2 units
        transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

        // send the tf: Parent -- turtle1; Child -- carrot1
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));

        rate.sleep();
    }

    return 0;
}