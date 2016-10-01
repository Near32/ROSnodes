#include "LFExtractor.h"

//#define PFEXTRACTOR_USE
#ifndef PFEXTRACTOR_USE
int main(int argc, char* argv[])
{
	ros::init(argc, argv,"LFExtractor");
	
	std::cout << " Usage : path2img [string] // typeOfTracker [int] // tresholdDetectionCorner [int]  // scalingFactor [int] ." << std::endl;
	
	std::string path2img("/camera/image_raw");
	if(argc>1)
	{
		path2img = std::string(argv[1]);
	}
	
	int typeTracker = 1;	//FAST OPENCV
	if( argc>2)
	{
		typeTracker = atoi(argv[2]);
	}
	
	int tresh = 100;
	if(argc>3)
	{
		tresh = atoi(argv[3]);
	}
	
	unsigned int scaler = 1;
	if(argc>4)
	{
		scaler = atoi(argv[4]);
	}
	
	LFExtractor lfe(path2img,typeTracker,tresh,scaler);
	
	ros::spin();
	
	return 0;
	
}

#else


int main(int argc, char* argv[])
{
	ros::init(argc, argv,"PFExtractor");
	
	std::cout << " Usage : path2img [string]  // tresholdDetectionCorner [int]  // scalingFactor [int] ." << std::endl;
	
	
	std::string path2img("/camera/image_raw");
	if(argc>1)
	{
		path2img = std::string(argv[1]);
	}
	
	
	int tresh = 100;
	if(argc>3)
	{
		tresh = atoi(argv[3]);
	}
	
	unsigned int scaler = 1;
	if(argc>4)
	{
		scaler = atoi(argv[4]);
	}
	
	PFExtractor pfe(path2img,tresh,scaler);
	
	ros::spin();
	
	return 0;
	
}


#endif

