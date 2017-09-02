const int N = 16; 
const int blocksize = 16; 

#include <ros/ros.h>
#include "warper/GPU_Warper.h"



__global__ 
void hello(char *a, int *b) 
{
	a[threadIdx.x] += b[threadIdx.x];
}
__global__
void gpu_process(const unsigned short* __restrict__ const in,const unsigned short*** __restrict__ const remap, unsigned short* __restrict__ const out) {
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

      //TODO: Use in and out instead of image_in and image_out
      red += distInv * in[inStep * searchY + 3 * searchX + 0];
      green += distInv * in[inStep * searchY + 3 * searchX + 1];
      blue += distInv * in[inStep * searchY + 3 * searchX + 2];
      total_weight += distInv;
    }

    out[outStep * y + 3 * x + 0] = round(red / total_weight);
    out[outStep * y + 3 * x + 1] = round(green / total_weight);
    out[outStep * y + 3 * x + 2] = round(blue / total_weight);
  }
}

GPU_Warper::GPU_Warper(){}


void GPU_Warper::process(){
	char a[N] = "Hello \0\0\0\0\0\0";
	int b[N] = {15, 10, 6, 0, -11, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	char *ad;
	int *bd;
	const int csize = N*sizeof(char);
	const int isize = N*sizeof(int);

	ROS_INFO("%s", a);

	cudaMalloc( (void**)&ad, csize ); 
	cudaMalloc( (void**)&bd, isize ); 
	cudaMemcpy( ad, a, csize, cudaMemcpyHostToDevice ); 
	cudaMemcpy( bd, b, isize, cudaMemcpyHostToDevice ); 
	
	dim3 dimBlock( blocksize, 1 );
	dim3 dimGrid( 1, 1 );
	hello<<<dimGrid, dimBlock>>>(ad, bd);
	cudaMemcpy( a, ad, csize, cudaMemcpyDeviceToHost ); 
	cudaFree( ad );
	cudaFree( bd );
	
	ROS_INFO("%s\n", a);

}