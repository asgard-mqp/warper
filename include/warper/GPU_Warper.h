static constexpr unsigned short maxR = 800, maxT = 4500;
static constexpr unsigned short midX = 960, midY = 540;
static constexpr unsigned short outStep = maxT*3, inStep = 1920 *3;//maxT *3 and 1920*3 

#include <sensor_msgs/Image.h>


class GPU_Warper
{
public:
  GPU_Warper();
 
  void process(const sensor_msgs::Image::ConstPtr image_in, sensor_msgs::Image &image_out, const unsigned short* (map));
};