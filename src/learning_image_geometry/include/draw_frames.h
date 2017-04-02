#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

class FrameDrawer
{
public:
    FrameDrawer(const std::vector<std::string>& frame_ids);
    ~FrameDrawer();

    void imageCB(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

private:
    // ROS node handle
    ros::NodeHandle nh_;

    // image pub/sub
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;

    // transform listener
    tf::TransformListener tf_listener_;

    // camera model
    image_geometry::PinholeCameraModel cam_model_;

    // A place to store frame IDs from tf...
    std::vector<std::string> frame_ids_;

    CvFont font_;

};