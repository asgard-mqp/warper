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
void gpu_process(const uint8_t* __restrict__ const in,const unsigned short* __restrict__ const remapArray, uint8_t* __restrict__ out) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  out[outStep * y + 3 * x + 0] = 255;//round(red / total_weight);
  out[outStep * y + 3 * x + 1] = 0;//round(red / total_weight);
  out[outStep * y + 3 * x + 2] = 0;//round(red / total_weight);
  /*
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
    //out[5] = 255;
    out[outStep * y + 3 * x + 0] = 255;//round(red / total_weight);
    out[outStep * y + 3 * x + 1] = 0;//round(red / total_weight);
    out[outStep * y + 3 * x + 2] = 0;//round(red / total_weight);

    //*(out + outStep * y + 3 * x + 0) = 255;//round(red / total_weight);
    //*(out + outStep * y + 3 * x + 1) = 0;//round(green / total_weight);
    //*(out + outStep * y + 3 * x + 2) = 0;//round(blue / total_weight);
  }
  */
}

GPU_Warper::GPU_Warper(uint8_t** input,const unsigned short* (map), uint8_t** output){
	cudaSetDeviceFlags(cudaDeviceMapHost);

  uint8_t* h_input;
  //gpu_process(input,map,output);
  //device arrays

  //ROS_INFO("input size %d output size %d",image_in.data.size(),image_out.data.size());
  ROS_INFO("pre pointer %d",h_output);

  ROS_INFO("host input return  %d, good value would be %d",cudaHostAlloc(&h_input, 1920*1080*3, cudaHostAllocMapped),cudaSuccess);
  ROS_INFO("host output return  %d, good value would be %d",cudaHostAlloc(&h_output, maxR*maxT*3, cudaHostAllocMapped),cudaSuccess);
  ROS_INFO("host map return  %d, good value would be %d",cudaHostAlloc(&map, 172800000, cudaHostAllocMapped),cudaSuccess);
  ROS_INFO("during pointer %d",h_output);
  *output = h_output;
  *input = h_input;

  ROS_INFO("input return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_input,  (void *) h_input , 0),cudaSuccess);
  ROS_INFO("output return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_output,  (void *) h_output , 0),cudaSuccess);
  ROS_INFO("map return %d, good value would be %d",cudaHostGetDevicePointer((void **)&d_map,  (void *) map , 0),cudaSuccess);
  ROS_INFO(" post pointer %d",h_output);

}


void GPU_Warper::process(){
  dim3 grid(maxT/16,maxR/16);
  dim3 threadPerBlock(16,16);
	gpu_process<<<grid,threadPerBlock>>>(d_input,d_map,d_output);
	ROS_INFO("sync return %d, good value would be %d",cudaThreadSynchronize(),cudaSuccess);
  ROS_INFO("gpu ended");
  ROS_INFO("pointer %d",h_output);
/*
  for(int i=0; i < maxR*maxT*3;i+=3){
    h_output[i] = 255;
  }*/
}