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

struct trackingItem {
    cv::Point2f trackingMarker;
    int trackedPixels;
    cv::Scalar lowerBound;
    cv::Scalar upperBound;
    cv::Scalar replace;
};

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

    void checkColor(trackingItem *item, cv::Mat filtered, size_t i, size_t j) {
        cv::Vec3b pixel = filtered.at<cv::Vec3b>(i,j);

        uint8_t blue  = pixel[2];
        uint8_t green = pixel[1];
        uint8_t red   = pixel[0];

        if (blue >=  item->lowerBound[2] && blue <=  item->upperBound[2] &&
            green >= item->lowerBound[1] && green <= item->upperBound[1] &&
            red >=   item->lowerBound[0] && red <=   item->upperBound[0])
        {
            item->trackingMarker.x += (float)j;
            item->trackingMarker.y += (float)i;
            item->trackedPixels++;

            pixel[2] = item->replace[2];
            pixel[1] = item->replace[1];
            pixel[0] = item->replace[0];
            filtered.at<cv::Vec3b>(i,j) = pixel;
        }
    }

    void camera_cb(const sensor_msgs::ImageConstPtr& image) {
        // ROS_INFO("Got new image to resize and track");
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

        // Filter the image
        cv::Mat filtered(cv::Size(image->width, image->height), CV_8UC3);
        cv::Mat color_mean_kernel(1, 1, CV_32FC1);
        color_mean_kernel.setTo(1.0);
        cv::filter2D(cv_ptr->image, filtered, CV_8UC3, color_mean_kernel);

        ///////////////////////////////////////////////////////////
        ///// Track the orange tip marker in the masked image /////
        ///////////////////////////////////////////////////////////
        struct trackingItem greenSpot;
        greenSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        greenSpot.trackedPixels = 0;
        greenSpot.lowerBound = cv::Scalar(150, 180, 120);
        greenSpot.upperBound = cv::Scalar(175, 220, 160);
        greenSpot.replace    = cv::Scalar(0, 0, 0);

        struct trackingItem yellowSpot;
        yellowSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        yellowSpot.trackedPixels = 0;
        yellowSpot.lowerBound = cv::Scalar(200, 200, 0);
        yellowSpot.upperBound = cv::Scalar(255, 255, 140);
        yellowSpot.replace    = cv::Scalar(0, 0, 0);

        struct trackingItem orangeSpot;
        orangeSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        orangeSpot.trackedPixels = 0;
        orangeSpot.lowerBound = cv::Scalar(170, 130, 80);
        orangeSpot.upperBound = cv::Scalar(230, 145, 130);
        orangeSpot.replace    = cv::Scalar(0, 0, 0);

        struct trackingItem blueSpot;
        blueSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        blueSpot.trackedPixels = 0;
        blueSpot.lowerBound = cv::Scalar(0, 0, 100);
        blueSpot.upperBound = cv::Scalar(70, 70, 255);
        blueSpot.replace    = cv::Scalar(0, 0, 0);

        // Because InRangeS is shit, use our own
        for (size_t i = 0; i < filtered.rows; i++)
        {
            for (size_t j = 0; j < filtered.cols; j++)
            {

                checkColor(&greenSpot,  filtered, i, j);
                checkColor(&yellowSpot, filtered, i ,j);
                checkColor(&orangeSpot, filtered, i, j);
                checkColor(&blueSpot,   filtered, i, j);
            }
        }
        // Now, compute the average x and y values for the tracked marker
        greenSpot.trackingMarker.x = greenSpot.trackingMarker.x / (float)greenSpot.trackedPixels;
        greenSpot.trackingMarker.y = greenSpot.trackingMarker.y / (float)greenSpot.trackedPixels;
        //ROS_INFO("Green tracking marker found with center %f (x) %f (y)", greenSpot.trackingMarker.x, greenSpot.trackingMarker.y);

        yellowSpot.trackingMarker.x = yellowSpot.trackingMarker.x / (float)yellowSpot.trackedPixels;
        yellowSpot.trackingMarker.y = yellowSpot.trackingMarker.y / (float)yellowSpot.trackedPixels;
        //ROS_INFO("Red tracking marker found with center %f (x) %f (y)", yellowSpot.trackingMarker.x, yellowSpot.trackingMarker.y);

        orangeSpot.trackingMarker.x = orangeSpot.trackingMarker.x / (float)orangeSpot.trackedPixels;
        orangeSpot.trackingMarker.y = orangeSpot.trackingMarker.y / (float)orangeSpot.trackedPixels;
        //ROS_INFO("Yellow tracking marker found with center %f (x) %f (y)", orangeSpot.trackingMarker.x, orangeSpot.trackingMarker.y);

        blueSpot.trackingMarker.x = blueSpot.trackingMarker.x / (float)blueSpot.trackedPixels;
        blueSpot.trackingMarker.y = blueSpot.trackingMarker.y / (float)blueSpot.trackedPixels;
        //ROS_INFO("Blue tracking marker found with center %f (x) %f (y)", blueSpot.trackingMarker.x, blueSpot.trackingMarker.y);

        // blue, yellow, orange, green
        ROS_INFO(", %f, %f, ||, %f, %f, ||, %f, %f, ||, %f, %f",
                 blueSpot.trackingMarker.x,   blueSpot.trackingMarker.y,
                 yellowSpot.trackingMarker.x, yellowSpot.trackingMarker.y,
                 orangeSpot.trackingMarker.x, orangeSpot.trackingMarker.y,
                 greenSpot.trackingMarker.x,  greenSpot.trackingMarker.y);
//        ROS_ERROR(", %f, %f, ||, %f, %f, ||, %f, %f, ||, %f, %f",
//                 blueSpot.trackingMarker.x,   blueSpot.trackingMarker.y,
//                 yellowSpot.trackingMarker.x, yellowSpot.trackingMarker.y,
//                 orangeSpot.trackingMarker.x, orangeSpot.trackingMarker.y,
//                 greenSpot.trackingMarker.x,  greenSpot.trackingMarker.y);

        /////////////////////////////////////////////////////
        ///// Publish the debug image /////
        /////////////////////////////////////////////////////
        // Make the debug image with checkboard centers, actuator base, and actuator tip drawn in
        cv::circle(filtered, cv::Point2i(snap(greenSpot.trackingMarker.x), snap(greenSpot.trackingMarker.y)), 5, cv::Scalar(0xff, 0x00, 0xff), -1);
        cv::circle(filtered, cv::Point2i(snap(yellowSpot.trackingMarker.x), snap(yellowSpot.trackingMarker.y)), 5, cv::Scalar(0x00, 0xff, 0xff), -1);
        cv::circle(filtered, cv::Point2i(snap(orangeSpot.trackingMarker.x), snap(orangeSpot.trackingMarker.y)), 5, cv::Scalar(0xff, 0x80, 0x00), -1);
        cv::circle(filtered, cv::Point2i(snap(blueSpot.trackingMarker.x), snap(blueSpot.trackingMarker.y)), 5, cv::Scalar(0x00, 0x00, 0xff), -1);
        // Convert back to ROS
        sensor_msgs::Image debug_image;
        cv_bridge::CvImage debug_converted(image->header, sensor_msgs::image_encodings::RGB8, filtered);
        debug_converted.toImageMsg(debug_image);
        debug_pub_.publish(debug_image);
        //ROS_INFO("Tracking operation finished");
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
    nhp.param(std::string("camera_base_topic"), camera_base_topic, std::string("usb_cam/image"));
    nhp.param(std::string("debug_output_topic"), debug_output_topic, std::string("fea/debug"));
    nhp.param(std::string("checkerboard_center_topic"), tracked_angle_topic, std::string("fea/position"));
    nhp.param(std::string("resized_width"), resized_width, 960);
    nhp.param(std::string("resized_height"), resized_height, 540);
    SoftRobotTracker tracker(nh, camera_base_topic, debug_output_topic, tracked_angle_topic, resized_width, resized_height);
    ROS_INFO("...startup complete");
    tracker.loop();
    return 0;
}
