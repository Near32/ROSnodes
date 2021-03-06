#include "VO/VO.h"

#include <iostream>
#include <thread>
#include <mutex>

extern mutex rMutex;

#define test1

int main( int argc, char* argv[])
{
	 float gradIntTreshold = 30;
    if(argc>1)
        gradIntTreshold = atoi(argv[1]);
        

    int factor = 1;
    
    if(argc>2)
        factor = atoi(argv[2]);
        
    int pyramidDepth = 1;
    
    if(argc>3)
    	pyramidDepth = atoi(argv[3]);
    	
    int nbrHypSearch = 10;
    
    if(argc>4)
    	nbrHypSearch = atoi(argv[4]);
    	
    int count = 1;
    		
    cout << "Usage : " << argv[0] << " gradTresh=50 invfactor=4 pyramidDepth=3 nbrHypSearch=10 meanDepth=0.9 initVarDepth=0.5 TUMBenchmark=0:false-->LiveCamera;1:true useExtendedKalmanFilterWithVO=1:true;0:false" << endl;		
    
    cv::Mat debugIm;    
    cv::namedWindow("Entry");
    
    //containers
    bool tum = true;

    bool continuer = true;
    
    cv::Mat frame, frame1,frame2;
    
    
    float meandepth = 1.0;
    float initVariance = 0.9;
    
    if(argc>5)
    	meandepth = atof(argv[5]);
    if(argc>6)
    	initVariance = atof(argv[6]);


	
    //cv::VideoCapture cap(1);
    cv::VideoCapture cap;
    if( argc>7)
    	tum = (atoi(argv[7])>=1? true:false);
    	
    if(tum)
    {
#ifdef test1
	    frame1 = cv::imread("../data/Images/LSD-SLAM_room/00001.png", CV_LOAD_IMAGE_COLOR);
	    frame2 = cv::imread("../data/Images/LSD-SLAM_room/00002.png", CV_LOAD_IMAGE_COLOR);
#else
	    frame1 = cv::imread("../data/Images/LSD-SLAM_room1/00501.png", CV_LOAD_IMAGE_COLOR);
	    frame2 = cv::imread("../data/Images/LSD-SLAM_room1/00502.png", CV_LOAD_IMAGE_COLOR);
#endif	    
    }
    else
    {
    	cap.open(1);
    	if(!cap.isOpened())
		{
		    cerr << "Erreur : Impossible de démarrer la capture video sur 1." << endl;
		    cap.open(0);
		    if(!cap.isOpened())
		    {
		    	cerr << "Erreur : Impossible de démarrer la capture video sur 0." << endl;
		        return -1;
		    }
		}
		
		cap >> frame;
		cap >> frame;
    }
#ifdef test1    
    string path("../data/Images/LSD-SLAM_room/");
#else    
    string path("../data/Images/LSD-SLAM_room1/");
#endif    
    int fcount = 2;
    
	
	//float w = 640.0f/factor;
	float w = 640.0f;
	//float h = 480.0f/factor;
	float h = 480.0f;
	
    Mat<float> K((float)0.0,3,3);
    /*
    K.set((float)-0.000876,1,1); //fx
    K.set((float)-0.000018459,2,2); //fy
    K.set((float)-0.4445,1,3); //cx
    K.set((float)-0.900336,2,3); //cy
    */
    
    K.set((float)723.546970/factor,1,1); //fx
    K.set((float)729.176276/factor,2,2); //fy
    K.set((float)-h/2,1,3); //cx
    K.set((float)-w/2,2,3); //cy
    
    /*
    K.set((float)0.39738586545*h,1,1); //fx
    K.set((float)0.78319662809*w,2,2); //fy
    */
    /*
    K.set((float)0.41778421402*h,1,3); //cx
    K.set((float)0.48249810536*w,2,3); //cy    
    */
    K.set((float)1.0,3,3);
    
    //Gestion ecriture dans un fichier :
    /*
    string filepath("./log.txt");
    FILE* log = fopen(filepath.c_str(), "w+");
    if(log == NULL)
    {
    	cout << "ERROR : cannot open the file." << endl;
    	exit(1);
    }
    else
    	cout << "File opened." << endl;
    */
    //------------------------------------
    
    
    VO<float> odometry( gradIntTreshold, nbrHypSearch, factor, K, pyramidDepth, meandepth, initVariance);
    
    std::thread tloop( &VO<float>::loop, std::ref(odometry) );
    
    while(continuer)
    {        
        count = 0;
        
        
        if(tum)
        {
			if(fcount == 100)
				fcount = 1;
			 
			if(fcount <10)
#ifdef test1		
				path+= string("0000");
#else			
				path+= string("0050");
#endif			
			else if(fcount < 100)
#ifdef test1		
				path+= string("000");
#else			
				path+= string("005");
#endif			
		
		
		
			stringstream ss;
			ss << fcount;	
			path = path+ ss.str()+string(".png");
			//cout << path << endl;
			frame = cv::imread(path.c_str(), CV_LOAD_IMAGE_COLOR);
#ifdef test1		
			path = string("../data/Images/LSD-SLAM_room/");
#else		
			path = string("../data/Images/LSD-SLAM_room1/");
#endif		
			fcount++;
			debugIm = frame;                     

        }
        else
        {
        	//while( !odometry.isReady());
        	//waiting for the odometry to be ready to grab new frame.
        	
        	
        	cap >> frame;
	        cap >> debugIm;        
		}
        
        odometry << frame;
        
        cv::imshow("Entry",frame);
       
       
       	
        rMutex.lock();
        if(cv::waitKey(30)>=0)
        {
            continuer = false;
     		odometry.continuer = false;
     	}
     	       
        if( odometry.continuer == false )
        {
        	continuer = false;
        }
        rMutex.unlock();
    }
    	
   	/*
   	rMutex.lock();
   	odometry.continuer = false;
   	rMutex.unlock();
   	*/
   	
    if(tloop.joinable())
    {
    	tloop.join();
    }
    
	return 0;
}

