//const int N = 16; 
//const int blocksize = 16; 

#include <ros/ros.h>
#include "warper/GPU_Warper.h"


__global__ 
void hello(char *a, int *b) 
{
	a[threadIdx.x] += b[threadIdx.x];
}

__global__
void gpu_process(const uint8_t* __restrict__ const in,const unsigned short* const remapArray, uint8_t* __restrict__ out) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < maxT && y < maxR) {
    float red = 0, green = 0, blue = 0, total_weight = 0;

    for (int direction = 0; direction < 4; direction++) {
      //check each direction
      const int searchX = *(remapArray + ((x * maxR) + y)*12 + direction * 3);
      const int searchY = *(remapArray + ((x * maxR) + y)*12 + direction * 3 + 1);
      const int distance = *(remapArray + ((x * maxR) + y)*12 + direction * 3 + 2);
      const float distInv = 1.0 / distance;

      red += distInv * in[inStep * searchY + 3 * searchX + 0];
      green += distInv * in[inStep * searchY + 3 * searchX + 1];
      blue += distInv * in[inStep * searchY + 3 * searchX + 2];
      total_weight += distInv;
    }
    //ROS_INFO("x %d y %d",x,y);

    //out[5] = 255;
    out[outStep * y + 3 * x + 0] = 255;//round(red / total_weight);
    out[outStep * y + 3 * x + 1] = 0;//round(red / total_weight);
    out[outStep * y + 3 * x + 2] = 0;//round(red / total_weight);

    //*(out + outStep * y + 3 * x + 0) = 255;//round(red / total_weight);
    //*(out + outStep * y + 3 * x + 1) = 0;//round(green / total_weight);
    //*(out + outStep * y + 3 * x + 2) = 0;//round(blue / total_weight);
  }
}

GPU_Warper::GPU_Warper(const sensor_msgs::Image &image_in, sensor_msgs::Image &image_out, const unsigned short* (map)){
	cudaSetDeviceFlags(cudaDeviceMapHost);
  const uint8_t* input = image_in.data.data();
  uint8_t* output = image_out.data.data();

  //gpu_process(input,map,output);
  //device arrays
  const uint8_t* d_input;
  uint8_t* d_output;
  const unsigned short* d_map;

  ROS_INFO("input size %d output size %d",image_in.data.size(),image_out.data.size());

  ROS_INFO("host input return  %d, good value would be %d",cudaHostAlloc(&input, image_in.data.size(), cudaHostAllocMapped),cudaSuccess);
  ROS_INFO("host output return  %d, good value would be %d",cudaHostAlloc(&output, image_out.data.size(), cudaHostAllocMapped),cudaSuccess);
  ROS_INFO("host map return  %d, good value would be %d",cudaHostAlloc(&map, 172800000, cudaHostAllocMapped),cudaSuccess);




  ROS_INFO("input return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_input,  (void *) input , 0),cudaSuccess);
  ROS_INFO("output return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_output,  (void *) output , 0),cudaSuccess);
  ROS_INFO("map return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_map,  (void *) map , 0),cudaSuccess);
}


void GPU_Warper::process(){
	gpu_process<<<10,10>>>(d_input,d_map,d_output);
	ROS_INFO("sync return %d, good value would be %d",cudaThreadSynchronize(),cudaSuccess);
  ROS_INFO("gpu ended");

}