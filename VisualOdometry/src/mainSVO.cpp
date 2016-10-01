#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define OPENCV_USE
#include "./StructureFromMotion/SFM.h"

void seekMatchs(const vector<vector<cv::Mat> > patchs_ct, Mat<int> *matchs);

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Visual Odometry-Image Processed";


#define debugOptim
class SVO
{
	private :
	
	//-------------------------
	//		EKF
	//-------------------------
	bool cbVOuse;
	int nbrstate;
	int nbrcontrol;
	int nbrobs;
	float dt;
	float stdnoiseobs;
	float stdnoiseobs_lin;
	float stdnoiseobs_ang;
	float stdnoisepro;
	float varxrot;
	float varyrot;
	float varzrot;
	float varx;
	float vary;
	float varz;
	bool ext;
	bool filteron;
	bool noise;

	Mat<float> EKFPose;  
	Mat<float> EKFPoseCovar;  
	EEKF<float>* instanceEKF;
	
	bool EKFuse;
	//------------------------
	//------------------------
	
	//------------------------
	//	STATS
	//------------------------
	Mat<float> statsmean;
	Mat<float> statsvar;
	int nbrIteration;
	//-------------------------
	//-------------------------
	
	
	//------------------------
	//		PARAMETERS
	//------------------------
	int gradIntTreshold;
	clock_t timertotal;
	clock_t timerdepth;
	float factor;
	int pyramidDepth;
	int nbrHypSearch;
	int count;
	float coeff;
	bool debug;
	bool withMultipleKF;
	//------------------------
	//------------------------
	
	Mat<float> khi;
	Mat<float> deltaPose;
	Mat<float> khiPose;
	Mat<float> lastKhiPose;
	vector<Mat<float> > pose;
	Mat<float> globalPose;
	Mat<float> globalKhiPose;
	
	cv::Mat frame;
	vector<cv::Mat> frames;
	int nbrFrame;
	vector<cv::Mat> KFlist;
	int nbrKF;
	int fcount;


	cv::Scalar colorbleue;
	cv::Scalar colorvert;
	cv::Mat debugIm;
	
	bool newImage;
	bool initKF;
	
	Mat<float> ResidualImage;
	Frame<float> f1;
	Frame<float> f1GradX, f1GradY, f1Grad;
	cv::Mat f1gradX;
	cv::Mat f1gradY;
	Mat<float> invDepthMap1;
	Mat<float> var_invDepthMap1;
	    	    
	Frame<float> f2;
	Frame<float> f2GradX, f2GradY, f2Grad;
	cv::Mat f2gradX;
	cv::Mat f2gradY;
	Mat<float> invDepthMap2;
	Mat<float> var_invDepthMap2;
	    
	float depthVar_init;
	int h;
	int w;


	float meandepth;
	float initVariance;
	
	image_transport::Publisher pub;

	
	public :
	
	SVO(int argc, char* argv[])
	{
	    //EKF : usual ...
	    cbVOuse = false;
	    
	    nbrstate = 12;
	    nbrcontrol = 0;
	    nbrobs = 6;
	    dt = 1;
	#define var_huberK
	#ifndef var_huberK    
	    stdnoiseobs = 1e-6;
	    stdnoiseobs_lin = 1e-4;
	    stdnoiseobs_ang = 1e-4;
	    stdnoisepro = 1e-4;
	    varxrot = stdnoiseobs_ang;
	    varyrot = stdnoiseobs_ang;
	    varzrot = stdnoiseobs_ang;
	    varx = stdnoiseobs_lin;
	    vary = stdnoiseobs_lin;
	    varz = stdnoiseobs_lin*1e1;
	#else    
	    stdnoiseobs = 1e-5;
	    stdnoiseobs_lin = 1e-4;
	    stdnoiseobs_ang = 1e-4;
	    stdnoisepro = 1e-3;
	    varxrot = 3.3e-5;
	    varyrot = 1e-5;
	    varzrot = 3.4e-7;
	    varx = 1.6e-4;
	    vary = 1.9e-4;
	    varz = 7.3e-4;
	#endif    
	    ext = false;
	    filteron = true;
	    noise = false;

	    EKFPose = Mat<float>((float)0,nbrstate,1);  
	    EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
	    instanceEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dt,stdnoiseobs,EKFPose,ext,filteron,noise,cbVOuse);
	    
	    Mat<float> A((float)0,nbrstate,nbrstate);
	    for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
	    //unstable if we propagate the velocities...
	    for(int i=1;i<=nbrstate/2;i++)	A.set((float)dt,i,nbrstate/2+i);
	    A.afficher();
	    instanceEKF->initA(A);
	    Mat<float> C((float)0,nbrobs,nbrstate);
	    for(int i=nbrstate/2+1;i<=nbrstate;i++)	C.set((float)1,i-nbrstate/2,i);
	    C.afficher();
	    instanceEKF->initC(C);
	    //B=0;

	    Mat<float> Q((float)0,nbrstate,nbrstate);
	    for(int i=nbrstate;i--;)	Q.set( (i+1>6 ? stdnoisepro : stdnoisepro), i+1, i+1);
	    instanceEKF->initQ(Q);
	    
	    Mat<float> R((float)0,nbrobs,nbrobs);
	    //for(int i=nbrobs;i--;)	R.set( (i+1>3 ? stdnoiseobs_lin : stdnoiseobs_ang), i+1, i+1);
	    //R.set( stdnoiseobs_lin*1e1, 3,3);
	    R.set( varxrot, 1,1);
	    R.set( varyrot, 2,2);
	    R.set( varzrot, 3,3);
	    R.set( varx, 4,4);
	    R.set( vary, 5,5);
	    R.set( varz, 6,6);
	    
	    instanceEKF->initR(R);
	    
	    EKFuse = false;
	    if(argc > 13)
	       	if(atoi(argv[13]))
	       		EKFuse = true;
	       		
	    statsmean = Mat<float>(0.0f, 6,1);
	    statsvar = Mat<float>(0.0f,6,1);
	    nbrIteration = 0;
	    //--------------------------------------------------------
	    //--------------------------------------------------------
	    
	    
	    
	    gradIntTreshold = 50;
	    if(argc>1)
		gradIntTreshold = atoi(argv[1]);
		
	    timertotal = clock();
	    timerdepth = clock();
	    
	    factor = (float)1.0/4;
	    if(argc>2)
		factor = (float)1.0/atoi(argv[2]);
		
	    pyramidDepth = 3;
	    if(argc>3)
	    	pyramidDepth = atoi(argv[3]);
	    	
	    nbrHypSearch = 10;
	    if(argc>4)
	    	nbrHypSearch = atoi(argv[4]);
	    	
	    count = 1;
	    coeff = (float)1.0/factor/2;
	    debug =false;
	    if(argc > 8)
	    	if(atoi(argv[8]))
	    		debug = true;
	    	else
	    		debug = false;
	    
	    //Mat<float> khi(numeric_limits<float>::epsilon(),6,1);
	    //Mat<float> khi(numeric_limits<float>::epsilon(),7,1);
	    khi = Mat<float>((float)0,7,1);
	    khi.set((float)1,7,1);

	    deltaPose = expMSIM3(khi);
	    khiPose = Mat<float>(6,1);
	    lastKhiPose = Mat<float>(0.0f,6,1);
	    globalPose = Mat<float>(deltaPose);
	    globalKhiPose = Mat<float>(khiPose);

	    nbrFrames = 0;
	    nbrKF = 0;
	    
	    withMultipleKF = false;
	    if(argc>5)
	    	if(atoi(argv[5]))
	    		withMultipleKF = true;
	    	else
	    		withMultipleKF = false;
	    		
	    cout << "Usage : " << argv[0] << " mode=12 gradTresh=50 invfactor=4 pyramidDepth=3 nbrHypSearch=10 withMultipleKF=0:false;1:true meanDepth=0.9 initVarDepth=0.5 debug=0:false;1=true TUMBenchmark=0:false-->LiveCamera;1:true DEBUGMODE:FramePerFrame=1:false-->Live;0:true DEBUGMODE:IrrlichtVisualisationOfDepthMAP=1:true;0:false useExtendedKalmanFilterWithVO=1:true;0:false" << endl;		
	    
	    colorbleue = cv::Scalar(255,0,0);
	    colorvert = cv::Scalar(0,255,0);

	    
	    cv::namedWindow("Entry");

	    fcount = 0;
	    initKF = false;
	    newImage = false;
	    
	    meandepth = 0.9;
	    initVariance = 1.0;
	    if(argc>7)
	    	meandepth = atof(argv[7]);
	    if(argc>8)
	    	initVariance = atof(argv[8]);
	    
	    initFrames();
	f1 = cv2Matp<float>( frame1 );
	f1Grad = cv2Matp<float>( computeGradientXY( &frame1, &f1gradX, &f1gradY) );
	f1GradX = cv2Matp<float>( f1gradX);
	f1GradY = cv2Matp<float>( f1gradY);

	int h = f1.getLine();
	int w = f1.getColumn();


	    
	    	
	invDepthMap1 = Mat<float>((float)0,h,w);
	invDepthMap1 =  initializeInvDM(&f1Grad, gradIntTreshold, initVariance, meandepth);
	    	
	    invDepthMap2 = invDepthMap1;
	    depthVar_init = (float)1.0;
	    var_invDepthMap1 = Mat<float>(depthVar_init*initVariance,h,w);
	    var_invDepthMap2 = Mat<float>(depthVar_init*initVariance,h,w);

	    Mat<float> x1( (float)1.0,3,1);
	    Mat<float> x2(x1);
	    Mat<float> line2;
	    Mat<float> line22;
	    float numlim = (float)1.0/pow(numeric_limits<float>::epsilon(),10);


	    Mat<float> K((float)0.0,3,3);
	    //K.set((float)-0.000876,1,1); //fx
	    //K.set((float)-0.000018459,2,2); //fy
	    //K.set((float)-0.4445,1,3); //cx
	    //K.set((float)-0.900336,2,3); //cy
	    K.set((float)723.546970*factor,1,1); //fx
	    K.set((float)729.176276*factor,2,2); //fy
	    K.set((float)-h/2,1,3); //cx
	    K.set((float)-w/2,2,3); //cy
	    
	    //OFFSET :
	    int offsetx = K.get(1,3);
	    int offsety = K.get(2,3);
	    Mat<float> offset(3,1);
	    offset.set( (float)offsetx, 1,1);
	    offset.set( (float)offsety, 2,1);
	    offset.set( (float)0, 3,1);
	    
	    K.set((float)1.0,3,3);
	    Mat<float> invK(invGJ(K));
	    invK.afficher();
	    Mat<float> rot(1,1);
	    Mat<float> invRot(1,1);
	    Mat<float> t((float)0,3,1);

	    float dist = (float)0;
	    float variance = (float)0;
	    Mat<float> disparityMap((float)1,h,w);
	    
	    
	    //Gestion ecriture dans un fichier :
	    string filepath("./log.txt");
	    FILE* log = fopen(filepath.c_str(), "w+");
	    if(log == NULL)
	    {
	    	cout << "ERROR : cannot open the file." << endl;
	    	exit(1);
	    }
	    else
	    	cout << "File opened." << endl;
	    //------------------------------------
	    
	    
	    
	    
	    while(continuer)
	    {        
		count = 0;        
		nbrFrame++;
		
		
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
			path = string("./data/Images/LSD-SLAM_room/");
	#else		
			path = string("./data/Images/LSD-SLAM_room1/");
	#endif		
			fcount++;
			debugIm = frame;                     
			//debugIm = cv::imread("./data/Images/LSD-SLAM_room/00001.png", CV_LOAD_IMAGE_COLOR);
			//debugIm = cv::imread("./data/Images/LSD-SLAM_room1/00501.png", CV_LOAD_IMAGE_COLOR);
		}
		else
		{
			cap >> frame;
			cap >> debugIm;        
		}
		
		cv::resize(frame,frame2,cv::Size(0,0),factor,factor);
		framelist.insert(framelist.end(), frame2);
		
		if(grad)
		{
		    f2 = cv2Matp<float>( frame2 );
		    f2Grad = cv2Matp<float>( computeGradientXY( &frame2, &f2gradX, &f2gradY) );
		    f2GradX = cv2Matp<float>( f2gradX);
		    f2GradY = cv2Matp<float>( f2gradY);
		}
		else
		{
		    f2 = cv2Matp<float>( computeGradient( &frame2) );
		}


		//deltaPose = poseEstimation(&f1, &invDepthMap1,&f2, &K, &invK);
	
	       	//khiPose = poseEstimationLSGRAD(&f1, &invDepthMap1,&f2, &f2GradX, &f2GradY, &K, &invK, &var_invDepthMap1, &ResidualImage, pyramidDepth);
	       	khiPose = poseEstimationLSGRAD(lastKhiPose, &f1, &invDepthMap1,&f2, &f2GradX, &f2GradY, &K, &invK, &var_invDepthMap1, &ResidualImage, pyramidDepth);
	       	
	       	statsmean += khiPose;
	       	statsvar += khiPose%khiPose;
	       	
	       	nbrIteration++;
	       	cout << "MEAN = " << endl;
	       	transpose( (1.0f/nbrIteration)*statsmean).afficher();
	       	cout << "VAR = " << endl;
	       	transpose( (1.0f/(nbrIteration-1))*statsvar).afficher();
	       	//--------------------------------------------------------
    		//--------------------------------------------------------
    		//EKF :
    		if(EKFuse)
    		{
    			instanceEKF.measurement_Callback(khiPose-lastKhiPose);
    			lastKhiPose = khiPose;
    			instanceEKF.state_Callback();
    			EKFPose = instanceEKF.getX();
    			EKFPoseCovar = instanceEKF.getSigma();
    			
    			transpose(EKFPose).afficher();
    			EKFPoseCovar.afficher();
    			//updating of the velocities value :
    			globalKhiPose = extract(&EKFPose,1,1,nbrstate/2,1);
    			khiPose = extract(&EKFPose,nbrstate/2+1,1,nbrstate,1);
    			
    			Mat<float> pose(khiPose+globalKhiPose);
		       	stringstream s;
		       	for(int i=0;i<=11;i++)	
		       	{
		       		s << EKFPose.get(i+1,1) << ",";
		       	}
		       	
		       	s << " | " ;
		       	for(int i=0;i<=pose.getLine()-1;i++)	
		       	{
		       		s << khiPose.get(i+1,1) << ",";
		       	}
		       	s << endl;
			cout << s.str();
			expM(pose).afficher();
		       	fputs( s.str().c_str(), log);
	    	}
    		else
    		{
	    		//--------------------------------------------------------
			//--------------------------------------------------------
			lastKhiPose = khiPose;
		       	Mat<float> pose(khiPose+globalKhiPose);
		       	stringstream s;
		       	for(int i=0;i<=pose.getLine()-1;i++)	
		       	{
		       		s << pose.get(i+1,1) << ",";
		       	}
		       	s << " | " ;
		       	for(int i=0;i<=pose.getLine()-1;i++)	
		       	{
		       		s << khiPose.get(i+1,1) << ",";
		       	}
		       	s << endl;
			cout << s.str();
		       	fputs( s.str().c_str(), log);
		       	//cout << s.str() << endl;
		}
	       		
	       	deltaPose = expM(khiPose);
	       	cout << "ESTIMATED MOVEMENT : " << endl;
	       	deltaPose.afficher();

		pose.insert(pose.end(),deltaPose);        
		
		rot = extract(deltaPose,1,1,3,3);
		invRot = transpose(rot);
		// rot € SO(3) !!!
		t = extract(deltaPose,1,4,3,4);
		
		
		timerdepth = clock();

		count = DepthUpdate21(nbrHypSearch, &f1, &f2, &invDepthMap1, &invDepthMap2, &rot, &t, &var_invDepthMap1, &var_invDepthMap2, &K, &invK, 1.0/factor);
		
		
#ifdef debugOptim
		cout << "L'execution depth-map a prise : " << (float)(clock()-timerdepth)/CLOCKS_PER_SEC << " secondes." << endl;
#endif
		

		Mat<float> ResidualAfterAlignement(w,h);
	
		if(debug)
		{
			ResidualAfterAlignement = debugPose( &f1, &f2, invDepthMap2/*depthmap assigned to f1..*/, rot, t, K, invK, coeff);
		}

		if(cv::waitKey(30)>=0)
		    continuer = false;

	
		ostringstream tfps, distance,compteur,var, gradient;
		fps = (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC));
		tfps << fps << " FPS.";
		distance << "distance = " << dist;
		var << "variance = " << variance;
		compteur << "nombre de pixels : " << count;
		gradient << "Valeur du treshold gradient : " << gradIntTreshold;
	#ifdef debugOptim        
		cout << "l'execution a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
	#endif        
		cv::putText(frame, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
		cv::putText(frame, distance.str(), cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
		cv::putText(frame, var.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
		cv::putText(frame, compteur.str(), cv::Point(10,120), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
		cv::putText(frame, gradient.str(), cv::Point(10,150), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
		//cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
		
		timertotal=clock();

		cv::imshow("Entry", frame);
	
	
		displayInverseCodedFrame(string("DEPTH MAP"), (Mat<float>*)&f1, &invDepthMap2, true, coeff*2);
	
		bool displaycontinuer = true;
		if(argc>11)
			displaycontinuer = ( atoi(argv[11]) == 1 ? true : false);
	
		displayInverseCodedFrame(string("VARIANCE OF DEPTH MAP"), (Mat<float>*)&f1, &var_invDepthMap2, displaycontinuer, coeff);

		//Depth Map update :
		invDepthMap1 = invDepthMap2;
		var_invDepthMap1 = var_invDepthMap2;

		//f1 = f2;
		
		
		//KF Handling :        
		Mat<float> W(0.0f,6,6);
		float wrotx = 1e4;
		float wroty = 1e4;
		float wrotz = 1e5;
		float wtx = 1e2;
		float wty = 1e2;
		float wtz = 1e2;
		W.set(wrotx, 1,1);
		W.set(wroty, 2,2);
		W.set(wrotz, 3,3);
		W.set(wtx, 4,4);
		W.set(wty, 5,5);
		W.set(wtz, 6,6);
		float val_tresh_move = 1;
		float val = ( transpose(khiPose) *(W * khiPose) ).get(1,1);
		
		cout << "MOVEMENT : tresh = "<< val_tresh_move << " ; val = " << val << endl;
		if( withMultipleKF &&  val >= val_tresh_move)
		{
			cout << "POSE GLOBAL : " << endl;
			//(operatorC(globalPose,deltaPose)).afficher();
			lastKhiPose *= 0.0f;
			globalKhiPose += khiPose;
			globalPose = deltaPose*globalPose;
			globalPose.afficher();
			cv::Mat newKF = framelist[nbrFrame-1];
			KFlist.insert(KFlist.end(), newKF);
			nbrKF++;
			
			
			f1 = f2;
			//invDepthMap2 = initializeInvDM(&f2Grad, gradIntTreshold, initVariance, meandepth) + propagateDepthMap( &K, &invK, &deltaPose, &invDepthMap2);
			invDepthMap2 = /*initializeInvDM(&f2Grad, gradIntTreshold, initVariance, meandepth)*/ propagateDepthMap( &f2Grad, gradIntTreshold, initVariance, meandepth, &K, &invK, &deltaPose, &invDepthMap2);
			
			regularizePropagatedDepthMap( &f2Grad, gradIntTreshold, &invDepthMap2, &var_invDepthMap2, depthVar_init);
			//invDepthMap1 = propagateDepthMap( &K, &invK, &deltaPose,&invDepthMap1);
			
			//var_invDepthMap2 += Mat<float>(depthVar_init,h,w); 
			
			invDepthMap1 = invDepthMap2;
			var_invDepthMap1 = var_invDepthMap2;       	
		}
		
		
		
		

	    }
	    
	    cv::destroyAllWindows();

#ifdef LOGDEPTH
	    //Gestion ecriture dans un fichier :
	    
	    string filepathDEPTH("./logDEPTH.txt");
	    FILE* logDEPTH = fopen(filepathDEPTH.c_str(), "w+");
	    if(logDEPTH == NULL)
	    {
	    	cout << "ERROR : cannot open the file DEPTH." << endl;
	    	exit(1);
	    }
	    else
	    	cout << "File opened DEPTH." << endl;
	    //------------------------------------
	    
	    //Ecriture de la depth map :
		strinstream ss;
		for(int i=0;i<invDepthMap1.getLine();i++)	
		{
			stringstream s;
			for(int j=0;j<invDepthMap1.getColumn();j++)
			{
	
				ss << (float)(1.0/invDepthMap1.get(i+1,j+1)) << ",";
			}

			ss << endl;
			fputs( s.str().c_str(), logDEPTH);
		}
	
	    
	    //------------------------------------
	    //Fermeture du fichier :
	    if(fclose(logDEPTH) == EOF)
	    {
	    	cout << "ERROR : cannot close the file DEPTH." << endl;
	    	exit(1);
	    }
	    else
	    	cout << "File closed DEPTH." << endl;
#endif

			    	
	    //Fermeture du fichier :
	    if(fclose(log) == EOF)
	    {
	    	cout << "ERROR : cannot close the file." << endl;
	    	exit(1);
	    }
	    else
	    	cout << "File closed." << endl;

	}
	
	
	~SVO()
	{
		delete instanceEKF;
	}
	
	
	//This function is called every time a new image is published
	void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
		
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("VisualOdometry::main.cpp::cv_bridge exception : %s", e.what());
			return ;
		}
	
		//-------------------------------------------------
		//
		//		Image Processing 
		//
		//-------------------------------------------------
		//Inversion
		/*
		for(int i=0;i<cv_ptr->image.rows;i++)
		{
			for(int j=0;j<cv_ptr->image.cols;j++)
			{
				for(int k=0;k<cv_ptr->image.channels();k++)
				{
					cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3+k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3+k];
				}
			}
		}
		*/
		cv::Mat Im(cv_ptr->image);
		//------------------------------------------------
	
		//------------------------------------------------
		//
		//		Stack of Frames
		//
		//-----------------------------------------------
		cv::resize(Im,Im,cv::Size(0,0),factor,factor);

	    	framelist.insert(framelist.end(),Im);
	    	nbrFrames++;
	    	
	    	if(!initKF)
	    	{
	    		initKF = true;
	    		KFlist.insert(KFlist.end(),Im);
	    		nbrKF++;
	    	}
	    	
		if(nbrFrames)
			newImage = true;
	
	
		//---------------------------------------------
		//
		//	End Image Processing
		//
		//---------------------------------------------
	
	
		//--------------------------------------------
		//
		//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
		//
		//--------------------------------------------
		/*
		if(nbrFramesProcessed)
		{	
			cv_bridge::CvImagePtr cv_ptrProcessed(cv_ptr);
			cv_ptrProcessed->image = framesProcessed[0];
			pub.publish(cv_ptrProcessed->toImageMsg());
			//pub.publish(cv_ptr->toImageMsg());
			if(nbrFramesProcessed>=10)
			{
				framesProcessed.pop_back();
			}
		}
		*/
	}

	
	
};



int main(int argc, char* argv[])
{
	bool fast = true;
	
	if(argc<2)
	{
		ROS_INFO("VisualOdometry::Argument needed : 1->SIFT / 0->FAST[DEFAULT].");
	}
	else
		fast = (atoi(argv[1]) ? true:false);
	
	ros::init(argc,argv,"VisualOdometry");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	cv::namedWindow(WINDOW,CV_WINDOW_AUTOSIZE);
	
	image_transport::Subscriber sub = it.subscribe("imageLOOP",1,imageCallback);
	
	
	if(fast)
		pub = it.advertise("VisualOdometry/imageFAST",1);
	else
		pub = it.advertise("VisualOdometry/imageSIFT",1);
	
	//ros::spin();
	LiveFASTorSIFT(fast);
	
	cv::destroyWindow(WINDOW);
	
	ROS_INFO("VisualOdometry::main.cpp::No error.");
	
}



void seekMatchs(const vector<vector<cv::Mat> > patchs_ct, Mat<int> *matchs)
{
    //only between the last two frames.
    int nbr_patchF1 = patchs_ct[0].size();
    int nbr_patchF2 = patchs_ct[1].size();

    Mat<float> patch_matching_error((float)0, nbr_patchF1, nbr_patchF2);

    for(int i=0;i<nbr_patchF1;i++)
    {
        for(int j=0;j<nbr_patchF2;j++)
        {            
            patch_matching_error.set( (float)sum( sum( absM(patchs_ct[0][i]-patchs_ct[1][j]) ) ).at<float>(1,1), i+1,j+1);            
        }

        int idmin = 0;
        int errmin = patch_matching_error.get(i,1);
        for(int j=0;j<nbr_patchF2;j++)
        {
            if(patch_matching_error.get(i,j) < errmin)
            {
                idmin = j;
                errmin = patch_matching_error.get(i,j);
            }
        }

        matchs->set(i,1,i+1);
        matchs->set(idmin, 2,i+1);
        //attention, décalage entre indice de colonne et indice de patch.
    }

    //patch_matching_error.afficher();
    //patch_matching_error = (255.0/max(patch_matching_error) )*patch_matching_error;
    //cv::Mat show = Mat2cvp( patch_matching_error, patch_matching_error, patch_matching_error);
    //afficher(&show,NULL,NULL,true,(float)10);

    //matchs->afficher();


}


