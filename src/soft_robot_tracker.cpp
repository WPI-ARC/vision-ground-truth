#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/image_encodings.h>

#define _USE_MATH_DEFINES
#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

class SoftRobotTracker
{
protected:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    int resized_width_;
    int resized_height_;
    bool convert_to_bw_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;
    ros::Publisher tracking_pub_;

public:

    SoftRobotTracker(ros::NodeHandle &n, std::string camera_base_topic, std::string debug_output_topic, std::string tracked_angle_topic, int resized_width, int resized_height) : nh_(n), it_(n)
    {
        resized_width_ = resized_width;
        resized_height_ = resized_height;
        image_sub_ = it_.subscribe(camera_base_topic, 1, &SoftRobotTracker::camera_cb, this);
        debug_pub_ = it_.advertise(debug_output_topic, 1, true);
        tracking_pub_ = nh_.advertise<std_msgs::Float64>(tracked_angle_topic, 1, true);
        std::string transport_in = image_sub_.getTransport();
        ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
    }

    ~SoftRobotTracker()
    {
    }

    void loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image)
    {
        ROS_INFO("Got new image to resize and track");
        // Convert to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Make destination
        cv::Mat resized(cv::Size(resized_width_, resized_height_), CV_8UC3);
        // Resize image
        if (resized_width_ < image->width && resized_height_ < image->height)
        {
            // If we're resizing smaller, use CV_INTER_AREA interpolation
            cv::resize(cv_ptr->image, resized, resized.size(), 0.0, 0.0, CV_INTER_AREA);
        }
        else
        {
            // If we're resizing bigger, use CV_INTER_LINEAR interpolation
            cv::resize(cv_ptr->image, resized, resized.size(), 0.0, 0.0, CV_INTER_LINEAR);
        }
        // Filter the resized image
        cv::Mat filtered_resized(cv::Size(resized_width_, resized_height_), CV_8UC3);
        cv::Mat color_mean_kernel(1, 1, CV_32FC1);
        color_mean_kernel.setTo(1.0);
        cv::filter2D(resized, filtered_resized, CV_8UC3, color_mean_kernel);
        ////////////////////////////////////////////////
        ///// Mask the image to what we care about /////
        ////////////////////////////////////////////////
        // We're going to (safely) assume that these corners won't change during testing
        std::vector<cv::Point2i> vertices(4);
        vertices[0] = cv::Point2i(210, 50);
        vertices[1] = cv::Point2i(700, 50);
        vertices[2] = cv::Point2i(700, 420);
        vertices[3] = cv::Point2i(210, 420);
        // Mask out everything beyond the corners to avoid noise
        cv::Mat mask(cv::Size(resized_width_, resized_height_), CV_8UC1);
        mask.setTo(0);
        cv::fillConvexPoly(mask, vertices, cv::Scalar(0xff));
        cv::Mat masked_resized(cv::Size(resized_width_, resized_height_), CV_8UC3);
        masked_resized.setTo(cv::Vec3b(0,0,0));
        filtered_resized.copyTo(masked_resized, mask);
        ///////////////////////////////////////////////////////////
        ///// Track the orange tip marker in the masked image /////
        ///////////////////////////////////////////////////////////
        cv::Point2f tracking_marker(0.0, 0.0);
        int tracked_pixels = 0;
        // Define the color bounds for the tracking marker (RGB!)
        cv::Scalar lower_bound(140, 30, 0);
        cv::Scalar upper_bound(225, 120, 60);
        // Because InRangeS is shit, use our own
        for (size_t i = 0; i < masked_resized.rows; i++)
        {
            for (size_t j = 0; j < masked_resized.cols; j++)
            {
                cv::Vec3b pixel = masked_resized.at<cv::Vec3b>(i,j);
                uint8_t blue = pixel[2];
                uint8_t green = pixel[1];
                uint8_t red = pixel[0];
                // First, check if the pixel is inside the range we want
                if (blue >= lower_bound[2] && blue <= upper_bound[2] && green >= lower_bound[1] && green <= upper_bound[1] && red >= lower_bound[0] && red <= upper_bound[0])
                {
                    // Now, check to make sure the pixel has the right dominant color
                    if (red > (blue + 20) && green > (blue + 20))
                    {
                        tracking_marker.x += (float)j;
                        tracking_marker.y += (float)i;
                        tracked_pixels++;
                    }
                }
            }
        }
        // Now, compute the average x and y values for the tracked marker
        tracking_marker.x = tracking_marker.x / (float)tracked_pixels;
        tracking_marker.y = tracking_marker.y / (float)tracked_pixels;
        ROS_INFO("Tracking marker found with center %f (x) %f (y)", tracking_marker.x, tracking_marker.y);
        //////////////////////////////////////////////
        ///// Estimate the angle of the actuator /////
        //////////////////////////////////////////////
        // We assume we know the base location of the actuator
        cv::Point2f actuator_base(530.0, 105.0);
        // Let's do some trig
        double delta_x = tracking_marker.x - actuator_base.x;
        double delta_y = tracking_marker.y - actuator_base.y;
        double estimated_angle = -(M_PI_2 - atan2(delta_y, delta_x));
        ROS_INFO("Estimated actuator angle %f", estimated_angle);
        /////////////////////////////////////////////////////
        ///// Publish the tracked angle and debug image /////
        /////////////////////////////////////////////////////
        // Publish the estimated angle
        std_msgs::Float64 tracking_msg;
        tracking_msg.data = estimated_angle;
        tracking_pub_.publish(tracking_msg);
        // Make the debug image with checkboard centers, actuator base, and actuator tip drawn in
        cv::circle(masked_resized, cv::Point2i(snap(actuator_base.x), snap(actuator_base.y)), 5, cv::Scalar(0xff, 0x00, 0x00), -1);
        cv::circle(masked_resized, cv::Point2i(snap(tracking_marker.x), snap(tracking_marker.y)), 5, cv::Scalar(0xff, 0x8c, 0x00), -1);
        // Convert back to ROS
        sensor_msgs::Image debug_image;
        cv_bridge::CvImage debug_converted(image->header, sensor_msgs::image_encodings::RGB8, masked_resized);
        debug_converted.toImageMsg(debug_image);
        debug_pub_.publish(debug_image);
        ROS_INFO("Tracking operation finished");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "soft_robot_tracker");
    ROS_INFO("Starting soft robot tracker...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string camera_base_topic;
    std::string debug_output_topic;
    std::string tracked_angle_topic;
    int resized_width;
    int resized_height;
    nhp.param(std::string("camera_base_topic"), camera_base_topic, std::string("logitech_hd_cam/image"));
    nhp.param(std::string("debug_output_topic"), debug_output_topic, std::string("fea/debug"));
    nhp.param(std::string("checkerboard_center_topic"), tracked_angle_topic, std::string("fea/position"));
    nhp.param(std::string("resized_width"), resized_width, 960);
    nhp.param(std::string("resized_height"), resized_height, 540);
    SoftRobotTracker tracker(nh, camera_base_topic, debug_output_topic, tracked_angle_topic, resized_width, resized_height);
    ROS_INFO("...startup complete");
    tracker.loop();
    return 0;
}
