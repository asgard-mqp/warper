
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

cv_bridge::CvImagePtr cv_ptr;


cv::Mat map_x, map_y;
cv::Mat dst;



uint8_t* imageLocation;
static const std::string OPENCV_WINDOW = "Image window";


unsigned short remap[maxT][maxR][12];
//4 directions
//absolute X cord, absolute Y cord, distance it was away in new image

unsigned short preInterpImage[maxT][maxR][2];
//absolute X cord, absolute Y cord

static constexpr float PI = 3.14159265;

__attribute__((always_inline))
uint8_t getO(const unsigned short x, const unsigned short y, const unsigned short z) {
  return image_in.data[image_in.step * y + 3 * x + z];
}

__attribute__((always_inline))
void getN(const unsigned short x, const unsigned short y, const unsigned short z, const float in) {
  image_out.data[image_out.step * y + 3 * x + z] = in;
}

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
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if(first){
    dst.create(maxR,maxT, cv_ptr->image.type() );
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

  }else{    
        // Draw an example circle on the video stream
    cv::remap(cv_ptr->image, dst, map_x, map_y, CV_INTER_LINEAR );

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, dst);

    cv::waitKey(3);
  }
}

void generate()
{
  //X in original image
  for (int x = 0; x < 1920; x++) {
    for (int y = 0; y < 1080; y++) {
      const double Angle = atan2(x - midX, y - midY) * (180.0 / PI) + 180 ; //to degrees
      const double Distance = sqrt(pow((x - midX), 2) + pow((y - midY), 2));
      const unsigned short newX = round(Angle*12.5);//to 0.08 degrees
      const unsigned short newY = round(Distance); // pixel radius


      if(newY < maxR && newX < maxT) {
        preInterpImage[newX][newY][0] = x ;
        preInterpImage[newX][newY][1] = y ;
        
      }

      if(y==1079)
        ROS_INFO("X %d Y %d",x,y);
    }
  }
  ROS_INFO("trig");

  //X in new image
  /*
  static constexpr int shifts[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
  for (unsigned short x = 0; x < maxT; x++) {
    for (unsigned short y = 0; y < maxR; y++) {
      //for every
      for (int direction = 0; direction < 4; direction++) {
        //check each direction
        unsigned short searchX = x;
        unsigned short searchY = maxR -y;
        int distance = 1;
        //ROS_INFO("first %d second %d",preInterpImage[searchX][searchY][0],preInterpImage[searchX][searchY][1]);

        while (distance > 0 && preInterpImage[searchX][searchY][0] == 65535 && preInterpImage[searchX][searchY][1] == 65535) {
          //while preInterpImage point is -1,-1, ie not known
          searchX += shifts[direction][0];
          searchY += shifts[direction][1];
          distance++;
          if (searchX >= maxT || searchX < 0 || searchY >= maxR || searchY < 0 || distance > 10 ){
            distance = 0;
            remap[x][y][direction * 3] = 0 ; // point to origin
            remap[x][y][direction * 3 + 1] = 0;
            remap[x][y][direction * 3 + 2] = 65000; //weight 65000 to have little effect
          }
        }
          //get original image pixel that created pixel at searchX,searchY
        if(distance>0){
          remap[x][y][direction * 3] = preInterpImage[searchX][searchY][0] ;
          remap[x][y][direction * 3 + 1] = preInterpImage[searchX][searchY][1];
          remap[x][y][direction * 3 + 2] = distance;
        }
          //if(distance==0)
            //ROS_INFO("pixel %d %d",preInterpImage[searchX][searchY][0],preInterpImage[searchX][searchY][1]);
        
      }
      //ROS_INFO("point");


    }
  }
  */
}

void process()
{
  for (int x = 0; x < maxT-1; x++) {
    for (int y = 0; y < maxR; y++) {
      //for every
      double red = 0;
      double green = 0;
      double blue = 0;
      double total_weight = 0;
      if(x==maxT-1)
        ROS_INFO("x %d y %d ",x,y);


      for (int direction = 0; direction < 4; direction++) {
        //check each direction
        const int searchX = remap[x][y][direction * 3];
        const int searchY = remap[x][y][direction * 3 + 1];
        const int distance = remap[x][y][direction * 3 + 2];
        const float distInv = 1.0 / distance;

        red += distInv * getO(searchX, searchY, 0);
        green += distInv * getO(searchX, searchY, 1);
        blue += distInv * getO(searchX, searchY, 2);
        total_weight += distInv;
        
      }

      getN(x, y, 0, round(red / total_weight));
      getN(x, y, 1, round(green / total_weight));
      getN(x, y, 2, round(blue / total_weight));
    }
  }
}

int main(int argc, char **argv) {



  //image_out.data.assign(maxT * maxR * 3 ,0);

  image_in.data.assign(1920 * 1080 *3,5);

  unsigned short* map;
  //uint8_t* GPULocation;

  //GPU_Warper totes_gpu(&imageLocation,&map,&GPULocation);


  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node;
  ROS_INFO("starting");

  memset(preInterpImage,-1,sizeof(unsigned short)*maxT*maxR*2);
  //generate();
 // memcpy(map, &remap[0][0][0], maxT * maxR * 12 * sizeof(short));//to lazy to make this cleaner


  ros::Publisher image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  while (ros::ok()) {
    if(message) {

      ROS_INFO("cpu started");

      //process();
      ROS_INFO("gpu started");


      //totes_gpu.process();
      ROS_INFO("gpu ended");

      /*image_out.header = image_in.header;
      image_out.encoding = image_in.encoding;
      image_out.is_bigendian = image_in.is_bigendian;
      image_out.step = maxT * 3;
      image_out.height = maxR;
      image_out.width = maxT;*/
      //memcpy(&image_out.data.at(0), GPULocation, maxT * maxR * 3);
      //memcpy(&(image_in.data.at(0)), imageLocation, 1920 * 1080 * 3);

      //image_pub.publish(image_out);
      message = false;

    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
