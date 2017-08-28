
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>

bool message = false;
sensor_msgs::Image image_in;
sensor_msgs::Image image_out;


#define maxR 800
#define maxT 9000
unsigned short remap[maxT][maxR][12]; 
//4 directions
//absolute X cord, absolute Y cord, distance it was away in new image

unsigned short preInterpImage[maxT][maxR][2];
//absolute X cord, absolute Y cord 

#define PI 3.14159265
unsigned short midX = 960;
unsigned short midY = 540;
#define getO(x,y,z) image_in.data[image_in.step*y + 3*x + z]
#define getN(x,y,z) image_out.data[image_out.step*y + 3*x + z]

void imageCallback (const sensor_msgs::Image::ConstPtr& image)
{
  image_in = *image;
  message = true;
}

void generate()
{
  //X in original image

  for(int x =0;x<1920;x++){
    for(int y=0; y < 1080; y++){

      double Angle = atan2(x-midX,y-midY) * (180.0/PI) + 180 ; //to degrees
      unsigned short newX = round(Angle*25);//to 0.04 degrees
      unsigned short newY = round(sqrt(pow((x-midX),2) + pow((y-midY),2))); // pixel radius
      //ROS_INFO("X %d  Y %d",newX,newY);

      if(newY < maxR){
        preInterpImage[newX][newY][0] = x+1;//just need them to be non 0, will correct later
        preInterpImage[newX][newY][1] = y+1;
      }
    }
  }

  ROS_INFO("finished Trig");



  //X in new image
  int shifts[4][2]={{0,1},{1,0},{0,-1},{-1,0}};
  for(unsigned short x=0; x<maxT; x++){
    for(unsigned short y=0; y<maxR; y++){//for every 
      unsigned short searchX = x;
      unsigned short searchY = y;
      for(int direction=0; direction<4; direction++){//check each direction
        int distance = 1;
        while(distance > 0 && preInterpImage[searchX][searchY][0] ==0 && preInterpImage[searchX][searchY][1] ==0){//while preInterpImage point is 0,0, ie not known
          searchX += shifts[direction][0];
          searchY += shifts[direction][1];
          distance ++;
          if( searchX >= maxT || searchX < 0 || searchY >= maxR || searchY < 0 ||distance > 20 ){
            searchX -= shifts[direction][0];
            searchY -= shifts[direction][1];
            distance = 0;
          }

        }
        //found position
        remap[x][y][direction*3] = preInterpImage[searchX][searchY][0] -1;//get original image pixel that created pixel at searchX,searchY 
        remap[x][y][direction*3 + 1] = preInterpImage[searchX][searchY][1]-1;
        remap[x][y][direction*3 + 2] = distance;
      }
    }
  }



}

void process()
{

  image_out.header = image_in.header;
  image_out.encoding = image_in.encoding;
  image_out.is_bigendian = image_in.is_bigendian;
  image_out.step = maxT * 3;
  image_out.height = maxR;
  image_out.width = maxT;
  for(int x=0; x<maxT; x++){
    for(int y=0; y<maxR; y++){//for every 
      double red = 0;
      double green = 0;
      double blue = 0;
      double total_weight = 0;

      for(int direction=0; direction<4; direction++){//check each direction
        int searchX = (int) remap[x][y][direction*3];
        int searchY = (int) remap[x][y][direction*3 + 1];
        int distance = remap[x][y][direction*3 + 2];
        //ROS_INFO("x %d y %d distance %d",searchX,searchY,distance);
        //ROS_INFO("getO %d",getO(searchX,searchY,0));

        red += (1.0/distance)*getO(searchX,searchY,0);
        blue += (1.0/distance)*getO(searchX,searchY,1);
        green += (1.0/distance)*getO(searchX,searchY,2);
        total_weight += 1.0/distance;
      }
      if(y>300 && y<600){
        getN(x,y,0)=255;
      }
    }
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node;
  ROS_INFO("starting");

  generate();

  image_out = sensor_msgs::Image();
  image_out.data.assign(21600000,0);
  ROS_INFO("started");

  ros::Publisher image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  while (ros::ok())
  {
    if(message)
    {

      message = false;
      //ROS_INFO("step %d height %d width %d encoding %s",image_in.step,image_in.height,image_in.width,image_in.encoding);
      process();
      image_pub.publish(image_out);

    }

    ros::spinOnce();
    rate.sleep();




  }

  return 0;

}
