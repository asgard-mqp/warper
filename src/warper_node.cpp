
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "warper/GPU_Warper.h"

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



//uint8_t* imageLocation;
static const std::string OPENCV_WINDOW = "Image window";


//unsigned short remap[maxT][maxR][12];
//4 directions
//absolute X cord, absolute Y cord, distance it was away in new image

//unsigned short preInterpImage[maxT][maxR][2];
//absolute X cord, absolute Y cord

static constexpr float PI = 3.14159265;



void imageCallback (const sensor_msgs::Image::ConstPtr& image)
{

  image_in.header = image->header;
  //memcpy(imageLocation, &image->data.at(0),image->data.size());

  ROS_INFO("image_in size  %d  image size %d",image_in.data.size(),image->data.size());
  //image_in.data.assign(image->data.begin()+1,image->data.end()-1);
  image_in.encoding = image->encoding;
  image_in.is_bigendian = image->is_bigendian;
  image_in.step = image->step;
  image_in.height = image->height;
  image_in.width = image->width;
  message = true;
  //image_in = *image;
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



  //image_out.data.assign(maxT * maxR * 3 ,0);

  //image_in.data.assign(1920 * 1080 *3,5);

  unsigned short* map;
  //uint8_t* GPULocation;

  //GPU_Warper totes_gpu(&imageLocation,&map,&GPULocation);


  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node;
  ROS_INFO("starting");

  //memset(preInterpImage,-1,sizeof(unsigned short)*maxT*maxR*2);
  //generate();
 // memcpy(map, &remap[0][0][0], maxT * maxR * 12 * sizeof(short));//to lazy to make this cleaner
  
  image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  while (ros::ok()) {
    if(message) {

      ROS_INFO("cpu started");

      //process();
      ROS_INFO("gpu started");


      //totes_gpu.process();
      ROS_INFO("gpu ended");

      if(first){
        out.image.create(maxR,maxT, cv_ptr->image.type() );
        map_x.create(maxR,maxT, CV_32FC1 );
        map_y.create(maxR,maxT, CV_32FC1 );
        for (int x = 0; x < maxT; x++) {
          for (int y = 0; y < maxR; y++) {
            const double radians = (PI/180.0)*(x/12.5);
            map_y.at<float>(maxR-y,x) = midY + y*sin(radians);
            map_x.at<float>(maxR-y,x) = midX + y*cos(radians);
          }
        }
        first = false;
      } 
        // Draw an example circle on the video stream
      ROS_INFO("encoder in %s",cv_ptr->encoding);

      out.header = cv_ptr->header;
      out.encoding = cv_ptr->encoding;
      ROS_INFO("encoder out %s",out.encoding);
      cv::remap(cv_ptr->image, out.image, map_x, map_y, CV_INTER_LINEAR);

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
