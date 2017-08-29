
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>

bool message = false;
sensor_msgs::Image image_in;
sensor_msgs::Image image_out;

static constexpr unsigned short maxR = 800, maxT = 9000;
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

void generate()
{
  //X in original image
  for (int x = 0; x < 1920; x++) {
    for (int y = 0; y < 1080; y++) {
      const double Angle = atan2(x - midX, y - midY) * (180.0 / PI) + 180 ; //to degrees
      const unsigned short newX = round(Angle*25);//to 0.04 degrees
      const unsigned short newY = round(sqrt(pow((x - midX), 2) + pow((y - midY), 2))); // pixel radius
      //ROS_INFO("X %d  Y %d",newX,newY);

      if(newY < maxR) {
        preInterpImage[newX][newY][0] = x + 1;//just need them to be non 0, will correct later
        preInterpImage[newX][newY][1] = y + 1;
      }
    }
  }

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
        while (distance > 0 && preInterpImage[searchX][searchY][0] == 0 && preInterpImage[searchX][searchY][1] == 0) {
          //while preInterpImage point is 0,0, ie not known
          searchX += shifts[direction][0];
          searchY += shifts[direction][1];
          distance++;
          if (searchX >= maxT || searchX < 0 || searchY >= maxR || searchY < 0 || distance > 20 )
            distance = 0;
        }
        if (distance > 0) {
          //get original image pixel that created pixel at searchX,searchY
          remap[x][y][direction * 3] = preInterpImage[searchX][searchY][0] -1;
          remap[x][y][direction * 3 + 1] = preInterpImage[searchX][searchY][1]-1;
          remap[x][y][direction * 3 + 2] = distance;
        }
      }
    }
  }
}

__global__ void process(const float* __restrict__ const in, float* __restrict__ const out) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < maxT && y < maxR) {
    float red = 0, green = 0, blue = 0, total_weight = 0;

    for (int direction = 0; direction < 4; direction++) {
      //check each direction
      const int searchX = remap[x][y][direction * 3];
      const int searchY = remap[x][y][direction * 3 + 1];
      const int distance = remap[x][y][direction * 3 + 2];
      const float distInv = 1.0 / distance;

      if (distance > 0) { //TODO: Use in and out instead of image_in and image_out
        red += distInv * image_in.data[image_in.step * searchY + 3 * searchX + 0]
        green += distInv * image_in.data[image_in.step * searchY + 3 * searchX + 1]
        blue += distInv * image_in.data[image_in.step * searchY + 3 * searchX + 2]
        total_weight += distInv;
      }
    }

    image_out.data[image_out.step * y + 3 * x + 0] = round(red / total_weight);
    image_out.data[image_out.step * y + 3 * x + 1] = round(green / total_weight);
    image_out.data[image_out.step * y + 3 * x + 2] = round(blue / total_weight);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "warper_node");
  ros::NodeHandle node;
  ROS_INFO("starting");

  const int n = maxT * maxR, blocksize = 512, nthreads = 1024;
  int nblocks = n / nthreads;

  generate();

  image_out = sensor_msgs::Image();
  image_out.data.assign(21600000,0);
  ROS_INFO("started");

  ros::Publisher image_pub = node.advertise<sensor_msgs::Image>("de_warped_image", 100);
  ros::Subscriber sub = node.subscribe("/usb_cam/image_raw", 10, imageCallback);
  ros::Rate rate(100.0);

  image_out.header = image_in.header;
  image_out.encoding = image_in.encoding;
  image_out.is_bigendian = image_in.is_bigendian;
  image_out.step = maxT * 3;
  image_out.height = maxR;
  image_out.width = maxT;

  while (ros::ok()) {
    if(message) {
      message = false;
      //ROS_INFO("step %d height %d width %d encoding %s",image_in.step,image_in.height,image_in.width,image_in.encoding);
      process<<<nblocks, blocksize>>>(in, out); //TODO: Make in and out shared memory
      image_pub.publish(image_out);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
