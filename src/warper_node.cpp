
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

bool message = false;
bool first = true;
sensor_msgs::Image image_in;
sensor_msgs::Image image_out;

ros::Publisher image_pub;

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage out;


cv::Mat map_x, map_y;
cv::Mat dst;

static const std::string OPENCV_WINDOW = "Image window";
static constexpr unsigned short maxR = 360, maxT = 1500; 
static constexpr unsigned short AngleOffset = 150;
static constexpr float PI = 3.14159265; 


void imageCallback (const sensor_msgs::Image::ConstPtr& image)
{
  image_in.header = image->header;

  ROS_INFO("image_in size  %d  image size %d",image_in.data.size(),image->data.size());
  image_in.encoding = image->encoding;
  image_in.is_bigendian = image->is_bigendian;
  image_in.step = image->step;
  image_in.height = image->height;
  image_in.width = image->width;
  message = true;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    ROS_INFO("success");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv) {
  double midX,midY;
  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node("~");
  ROS_INFO("starting");
  node.getParam("center_X",midX);
  node.getParam("center_Y",midY);
  ROS_INFO("X %f Y %f",midX,midY);

  
  image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  image_out.header = image_in.header;
  image_out.encoding = image_in.encoding;
  image_out.is_bigendian = image_in.is_bigendian;
  image_out.step = (maxT-2*AngleOffset) * 3 ;
  image_out.height = maxR;
  image_out.width = maxT-2*AngleOffset;
  float widthPerDegree= maxT/360.0;
  while (ros::ok()) {
    if(message) {
      if(first){
        out.image.create(maxR,maxT-2*AngleOffset, cv_ptr->image.type() );
        map_x.create(maxR,maxT-2*AngleOffset, CV_32FC1 );
        map_y.create(maxR,maxT-2*AngleOffset, CV_32FC1 );

        for (int x = AngleOffset; x < maxT-AngleOffset; x++) {
          for (int y = 0; y <maxR; y++) {
            const double radians = (PI/180.0)*(x/widthPerDegree);
            map_y.at<float>(maxR-y -1,x-AngleOffset) = midY - y*cos(radians);
            map_x.at<float>(maxR-y -1,x-AngleOffset) = midX + y*sin(radians);
          }
        }
        first = false;
      } 

      ROS_INFO("encoder in %s",cv_ptr->encoding);

      out.header = cv_ptr->header;
      out.encoding = cv_ptr->encoding;
      ROS_INFO("encoder out %s",out.encoding);

      cv::remap(cv_ptr->image, out.image, map_x, map_y, CV_INTER_LINEAR);

      //uncomment to visualize data
      //cv::imshow(OPENCV_WINDOW, out.image);
      image_pub.publish(out.toImageMsg());

      cv::waitKey(3);


      message = false;

    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
