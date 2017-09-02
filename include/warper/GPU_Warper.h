static constexpr unsigned short maxR = 800, maxT = 4500;
static constexpr unsigned short midX = 960, midY = 540;
static constexpr unsigned short outStep = maxT*3, inStep = 1920 *3;//maxT *3 and 1920*3 



class GPU_Warper
{
public:
  GPU_Warper();
 
  void process();
};