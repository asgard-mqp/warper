static constexpr unsigned short maxR = 800, maxT = 4500;
static constexpr unsigned short midX = 960, midY = 540;
static constexpr unsigned short outStep = maxT*3, inStep = 1920 *3;//maxT *3 and 1920*3 

#include <sensor_msgs/Image.h>


class GPU_Warper
{
public:
  GPU_Warper(uint8_t** input,const unsigned short* (map), uint8_t** output);
 
  void process();
private:
	const uint8_t* d_input;
	uint8_t* h_output;
	uint8_t* d_output;
	const unsigned short* d_map;
};