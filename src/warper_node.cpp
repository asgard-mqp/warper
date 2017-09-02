
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>

bool message = false;
sensor_msgs::Image image_in;
sensor_msgs::Image image_out;

static constexpr unsigned short maxR = 800, maxT = 4500;
unsigned short remap[maxT][maxR][12];
//4 directions
//absolute X cord, absolute Y cord, distance it was away in new image

unsigned short preInterpImage[maxT][maxR][2];
//absolute X cord, absolute Y cord

static constexpr float PI = 3.14159265;
static constexpr unsigned short midX = 960, midY = 540;

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
  image_in = *image;
  message = true;
}
unsigned short FillerPixelX;
unsigned short FillerPixelY;
void generate()
{
  //X in original image
  for (int x = 0; x < 1920; x++) {
    for (int y = 0; y < 1080; y++) {
      const double Angle = atan2(x - midX, y - midY) * (180.0 / PI) + 180 ; //to degrees
      const unsigned short newX = round(Angle*12.5);//to 0.08 degrees
      const unsigned short newY = round(sqrt(pow((x - midX), 2) + pow((y - midY), 2))); // pixel radius
      if(x==0 && y==0){
        FillerPixelX = newX;
        FillerPixelY = newY;
      }

      if(newY < maxR && newX < maxT) {
        preInterpImage[newX][newY][0] = x ;
        preInterpImage[newX][newY][1] = y ;
      }


    }
  }
  ROS_INFO("trig");

  //X in new image
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
}

void process()
{
  image_out.header = image_in.header;
  image_out.encoding = image_in.encoding;
  image_out.is_bigendian = image_in.is_bigendian;
  image_out.step = maxT * 3;
  image_out.height = maxR;
  image_out.width = maxT;

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
        // if (searchX >= 1920 || searchY >= 1080) {
        //ROS_INFO("x %d y %d distance %d",searchX,searchY,distance);
        // }
        //ROS_INFO("get %d",getO(searchX,searchY,0));
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
  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node;
  ROS_INFO("starting");

  memset(preInterpImage,-1,sizeof(unsigned short)*maxT*maxR*2);
  ROS_INFO("started %d %d",preInterpImage[5][5][0],preInterpImage[5][5][1]);


  generate();

  image_out = sensor_msgs::Image();
  image_out.data.assign(maxT * maxR * 3 ,0);
  ROS_INFO("started");

  ros::Publisher image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  while (ros::ok()) {
    if(message) {
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
