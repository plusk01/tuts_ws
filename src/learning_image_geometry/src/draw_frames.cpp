#include "draw_frames.h"

FrameDrawer::FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
{
    // find the topic "image" using the node's namespace
    std::string image_topic = nh_.resolveName("image");

    // set up image pub/sub
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCB, this);
    pub_ = it_.advertise("iamge_out", 1);

    // initialize a font for writing on OpenCV images
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
}

// ----------------------------------------------------------------------------

FrameDrawer::~FrameDrawer() { }

// ----------------------------------------------------------------------------

void FrameDrawer::imageCB(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;

    try {
        // Get an OpenCV Mat from the ROS message using cv_bridge
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    } catch (cv_bridge::Exception &ex) {
        ROS_ERROR("[draw_frames] Failed to convert image");
        return;
    }

    cam_model_.fromCameraInfo(info_msg);

    for (const auto& frame_id : frame_ids_) {
        tf::StampedTransform transform;

        try {
            ros::Time acquisition_time = info_msg->header.stamp;
            ros::Duration timeout(1.0 / 30);

            tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id, acquisition_time, timeout);
            tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id, acquisition_time, transform);

        } catch (tf::TransformException &ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }

        tf::Point pt = transform.getOrigin();
        cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

        static const int RADIUS = 3;
        cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
        CvSize text_size;
        int baseline;
        cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
        CvPoint origin = cvPoint(uv.x - text_size.width/2, uv.y - RADIUS - baseline - 3);
        cv::putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255, 0, 0));
    }

    // send the image out
    pub_.publish(input_bridge->toImageMsg());
}

// ----------------------------------------------------------------------------