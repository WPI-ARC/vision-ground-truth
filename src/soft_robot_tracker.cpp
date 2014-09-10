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
        // We're /*going to (safely) assume that these corners won't change during testing
        std::vector<cv::Point2i> vertices(4);
        vertices[0] = cv::Point2i(0, 0);
        vertices[1] = cv::Point2i(1000, 0);
        vertices[2] = cv::Point2i(1000, 600);
        vertices[3] = cv::Point2i(0, 600);
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
        struct trackingItem greenSpot;
        greenSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        greenSpot.trackedPixels = 0;
        greenSpot.lowerBound = cv::Scalar(0, 121, 0);
        greenSpot.upperBound = cv::Scalar(120, 255, 120);

        struct trackingItem redSpot;
        redSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        redSpot.trackedPixels = 0;
        redSpot.lowerBound = cv::Scalar(121, 0, 0);
        redSpot.upperBound = cv::Scalar(255, 120, 120);

        struct trackingItem yellowSpot;
        yellowSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        yellowSpot.trackedPixels = 0;
        yellowSpot.lowerBound = cv::Scalar(121, 121, 0);
        yellowSpot.upperBound = cv::Scalar(255, 255, 120);

        struct trackingItem blueSpot;
        blueSpot.trackingMarker = cv::Point2f(0.0, 0.0);
        blueSpot.trackedPixels = 0;
        blueSpot.lowerBound = cv::Scalar(0, 0, 121);
        blueSpot.upperBound = cv::Scalar(120, 120, 255);

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
                if (blue >=  greenSpot.lowerBound[2] && blue <=  greenSpot.upperBound[2] &&
                    green >= greenSpot.lowerBound[1] && green <= greenSpot.upperBound[1] &&
                    red >=   greenSpot.lowerBound[0] && red <=   greenSpot.upperBound[0])
                {
                    // Now, check to make sure the pixel has the right dominant color
                    if (green > (red + 20) && green > (blue + 20))
                    {
                        greenSpot.trackingMarker.x += (float)j;
                        greenSpot.trackingMarker.y += (float)i;
                        greenSpot.trackedPixels++;
                    }
                }
                else if (blue >=  redSpot.lowerBound[2] && blue <=  redSpot.upperBound[2] &&
                         green >= redSpot.lowerBound[1] && green <= redSpot.upperBound[1] &&
                         red >=   redSpot.lowerBound[0] && red <=   redSpot.upperBound[0])
                {
                    // Now, check to make sure the pixel has the right dominant color
                    if (red > (blue + 20) && red > (green + 20))
                    {
                        redSpot.trackingMarker.x += (float)j;
                        redSpot.trackingMarker.y += (float)i;
                        redSpot.trackedPixels++;
                    }
                }
                else if (blue >=  yellowSpot.lowerBound[2] && blue <=  yellowSpot.upperBound[2] &&
                         green >= yellowSpot.lowerBound[1] && green <= yellowSpot.upperBound[1] &&
                         red >=   yellowSpot.lowerBound[0] && red <=   yellowSpot.upperBound[0])
                {
                    // Now, check to make sure the pixel has the right dominant color
                    if (red > (blue + 20) && green > (blue + 20))
                    {
                        yellowSpot.trackingMarker.x += (float)j;
                        yellowSpot.trackingMarker.y += (float)i;
                        yellowSpot.trackedPixels++;
                    }
                }
                else if (blue >=  blueSpot.lowerBound[2] && blue <=  blueSpot.upperBound[2] &&
                         green >= blueSpot.lowerBound[1] && green <= blueSpot.upperBound[1] &&
                         red >=   blueSpot.lowerBound[0] && red <=   blueSpot.upperBound[0])
                {
                    // Now, check to make sure the pixel has the right dominant color
                    if (blue > (red + 20) && blue > (green + 20))
                    {
                        blueSpot.trackingMarker.x += (float)j;
                        blueSpot.trackingMarker.y += (float)i;
                        blueSpot.trackedPixels++;
                    }
                }
            }
        }
        // Now, compute the average x and y values for the tracked marker
        greenSpot.trackingMarker.x = greenSpot.trackingMarker.x / (float)greenSpot.trackedPixels;
        greenSpot.trackingMarker.y = greenSpot.trackingMarker.y / (float)greenSpot.trackedPixels;
        ROS_INFO("Green tracking marker found with center %f (x) %f (y)", greenSpot.trackingMarker.x, greenSpot.trackingMarker.y);

        redSpot.trackingMarker.x = redSpot.trackingMarker.x / (float)redSpot.trackedPixels;
        redSpot.trackingMarker.y = redSpot.trackingMarker.y / (float)redSpot.trackedPixels;
        ROS_INFO("Red tracking marker found with center %f (x) %f (y)", redSpot.trackingMarker.x, redSpot.trackingMarker.y);

        yellowSpot.trackingMarker.x = yellowSpot.trackingMarker.x / (float)yellowSpot.trackedPixels;
        yellowSpot.trackingMarker.y = yellowSpot.trackingMarker.y / (float)yellowSpot.trackedPixels;
        ROS_INFO("Yellow tracking marker found with center %f (x) %f (y)", yellowSpot.trackingMarker.x, yellowSpot.trackingMarker.y);

        blueSpot.trackingMarker.x = blueSpot.trackingMarker.x / (float)blueSpot.trackedPixels;
        blueSpot.trackingMarker.y = blueSpot.trackingMarker.y / (float)blueSpot.trackedPixels;
        ROS_INFO("Blue tracking marker found with center %f (x) %f (y)", blueSpot.trackingMarker.x, blueSpot.trackingMarker.y);

        /////////////////////////////////////////////////////
        ///// Publish the debug image /////
        /////////////////////////////////////////////////////
        // Make the debug image with checkboard centers, actuator base, and actuator tip drawn in
        cv::circle(masked_resized, cv::Point2i(snap(greenSpot.trackingMarker.x), snap(greenSpot.trackingMarker.y)), 5, cv::Scalar(0xff, 0x00, 0xff), -1);
        cv::circle(masked_resized, cv::Point2i(snap(redSpot.trackingMarker.x), snap(redSpot.trackingMarker.y)), 5, cv::Scalar(0x00, 0xff, 0xff), -1);
        cv::circle(masked_resized, cv::Point2i(snap(yellowSpot.trackingMarker.x), snap(yellowSpot.trackingMarker.y)), 5, cv::Scalar(0x00, 0x00, 0xff), -1);
        cv::circle(masked_resized, cv::Point2i(snap(blueSpot.trackingMarker.x), snap(blueSpot.trackingMarker.y)), 5, cv::Scalar(0xff, 0xff, 0x00), -1);
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
