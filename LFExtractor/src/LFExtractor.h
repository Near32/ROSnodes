#ifndef LFEXTRACTOR_H


#include "./LF/LF.h"
#include "./RunningStats/RunningStats.h"

#include <iostream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

namespace enc = sensor_msgs::image_encodings;

#define features_use
//#define haar_use

#define restrictFEATURES

int count_info = 51;
#define debug_v0	

//#include <MAT/MatOpenCV.h>


void sorting(std::vector<cv::DMatch>& m, Mat<float>& sortingVal)
{
	int nbrdata = sortingVal.getLine();
	int idxColumn = 1;
	cv::DMatch dataOnLine;
	
	for(int i=0;i<nbrdata;i++)
	{
		float val = sortingVal.get(i+1,idxColumn);
		//let us save that line of data :
		dataOnLine = m[i];		
		
		int j = i;
		//let us move the previous lines over this one or...
		while( j > 0 && sortingVal.get(j,idxColumn) > val)
		{
			
			m[j] = m[j-1];
			sortingVal.set( sortingVal.get(j,idxColumn), j+1,idxColumn);
			
			j = j-1;
		}
		
		//... let us reassign the saved line :
		m[j] = dataOnLine;
		sortingVal.set( val, j+1,idxColumn);
	}

}



mat::Mat<float> Mouse((float)0,2,1);
bool update = false;
int idx = 1;
void CallBackMouseFunc(int event, int x, int y, int flag, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        Mouse.set( (float)x, 1,1);
        Mouse.set( (float)y, 2,1);
        update = true;
        cout << "Position de la souris : " << endl;
        Mouse.afficher();
        idx = (idx+1)%2;
    }
}

class FeatureRegister
{
	private :
	
	std::mutex mutexRES;
	
	std::vector<cv::Mat> features;
	std::vector<cv::Mat> imgs;
	std::vector<std::vector<cv::KeyPoint> > kps;
	
	std::vector<cv::Mat> featuresToProcess;
	cv::Mat imgToProcess;
	std::vector<cv::KeyPoint> kpToProcess;
	
	bool registering;
	
	
	std::thread* t;
	bool continuer;
	bool binaryFeatures;
	float lastmeandist;
	
	//delimiter of the selected keypoints :
	int xs1;
	int ys1;
	int xs2;
	int ys2;
	
	public :
	
	
	
	ros::NodeHandle nh;
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	
	
	FeatureRegister(bool bf = true)	: registering(true), continuer(true), binaryFeatures(bf)
	{
		lastmeandist = 100.0f;
		
		xs1 = 100;
		xs2 = 200;
		ys1 = 100;
		ys2 = 200;
		
		it = new image_transport::ImageTransport(nh);		
		img_pub = it->advertise("/LFExtractor/MATCHING", 1);
		
		t = new std::thread(&FeatureRegister::loop, this);
		
		
	}
	
	~FeatureRegister()
	{
		this->setContinuer(false);
		
		if(this->t->joinable())
		{
			this->t->join();
		}
		
		delete this->t;
		delete this->it;
	}
	
	void loop()
	{	
	    cv::namedWindow("DELIMITER FEATURES",CV_WINDOW_AUTOSIZE);
	    cv::setMouseCallback("DELIMITER FEATURES",CallBackMouseFunc);
	    	
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			//clock_t timer = clock();
			
			if(this->registering)
			{
				mutexRES.lock();
				cv::Mat img;
				std::vector<cv::KeyPoint> kp;
				imgToProcess.copyTo(img);
				kp = kpToProcess;
				mutexRES.unlock();
				
				//clock_t time = clock();
				
				for(unsigned int k=0;k<this->featuresToProcess.size();k++)
				{
					mutexRES.lock();
					cv::Mat f;
					featuresToProcess[k].copyTo(f);
					mutexRES.unlock();
					
					bool add = decideAddition(f,img,kp);
					
					if( add)
					{
						features.push_back(f);
					}
					
				}
				
				/*
				mutexRES.lock();
				cv::Mat f;
				featuresToProcess[0].copyTo(f);
				mutexRES.unlock();
				
				bool add = decideAddition(f,img,kp);
				
				if( add)
				{
					features.push_back(f);
				}
				*/
				
				//ROS_INFO("NBR FEATURES : %d ; TOOK : %f seconds.", this->featuresToProcess.size(), (float)((float)(clock()-time)/CLOCKS_PER_SEC) );
			}
			
			
			
			
			//-------------------------------------------
			
			
			if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				break; 
			}


			if(update)
			{
				update = false;
				switch(idx)
				{
					case 0:
					{
						xs1 = Mouse(1,1);
						ys1 = Mouse(2,1);
						std::cout << " UPDATE X,Y 1 : " << xs1 << " / " << ys1 <<  std::endl;
					}
					break;
	
					case 1:
					{
						xs2 = Mouse(1,1);
						ys2 = Mouse(2,1);
						std::cout << " UPDATE X,Y 2 : " << xs2 << " / " << ys2 <<  std::endl;
					}
					break;
				}
			}
			
			mutexRES.lock();
			cv::Mat dummy;
			if(imgs.size() < 1)
			{
				imgToProcess.copyTo(dummy);
			}
			else
			{
				imgs[0].copyTo(dummy);
			}
			mutexRES.unlock();
			
			try{
				if( kps.size()>0)
				{
					cv::drawKeypoints( dummy, kps[0], dummy, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
				}
				
				cv::Point p1(xs1,ys1);
				cv::Point p2(xs2,ys2);
				cv::Scalar color(255,0,0);
				int thickness = 1;
				int lineType = 8;
				int shift = 0;			
				cv::rectangle(dummy,p1, p2, color, thickness, lineType, shift);
				cv::imshow("DELIMITER FEATURES",dummy);
				
			}
			catch(const cv::Exception& e)
			{
			
			}
			
			//ROS_INFO("FEATURES REGISTER : FPS :: %f.", (float)((float)CLOCKS_PER_SEC/((float)(clock()-timer))) );
			
			
			mutexRES.lock();
		}
		mutexRES.unlock();
		
	}
	
	bool decideAddition(const cv::Mat& f,const cv::Mat& img, std::vector<cv::KeyPoint>& kp)
	{
		//TODO
		int normType = cv::NORM_HAMMING;
		if(!binaryFeatures)
		{
			normType = cv::NORM_L2;
		}
		bool crossCheck = true;
		cv::BFMatcher bfm(normType, crossCheck);
		
		if(features.size() == 0)
		{
			features.push_back(f);
			imgs.push_back(img);
			kps.push_back(kp);
		}
		
		
		std::vector<cv::DMatch> matches;
		if(binaryFeatures)
		{
			#ifdef restrictFEATURES
			//cv::Mat featuresRestricted = restrictFeatures(features[0], xs1,ys1,xs2,ys2,kps[0]);
			cv::Mat featuresRestricted;
			std::vector<cv::KeyPoint> kpsRestricted;
			restrictFeatures(features[0], featuresRestricted, xs1,ys1,xs2,ys2,kps[0], kpsRestricted);
			#else
			cv::Mat featuresRestricted = features[0];
			std::vector<cv::KeyPoint> kpsRestricted = kps[0];
			#endif
			//std::cout << " DIMS : f : " << f.rows << " x " << f.cols << " ; featurest : " << featuresRestricted.rows << " x " << featuresRestricted.cols << std::endl;
			
			//bfm.match(f,features[0],matches);
			//clock_t time = clock();
			bfm.match(f,featuresRestricted,matches);
	
			cv::Mat dim;
			//ROS_INFO("MATCHES : %d. TOOK : %f seconds.",matches.size(), (float)((float)(clock()-time)/CLOCKS_PER_SEC) );
			
			//SORTING :
			Mat<float> val(matches.size(),1);
			for(int k=0;k<matches.size();k++)
			{
				val.set( matches[k].distance, k+1,1);
			}
			sorting(matches,val);
			//--------------------------
			
			
			std::vector<cv::DMatch> GoodMatches;
			/*
			float meandist = 0.0f;
			float mindist = 1e3f;
			for(int k=1;k<matches.size();k++)
			{
				meandist += matches[k].distance;
				if(matches[k].distance <= lastmeandist)
				{
					GoodMatches.push_back(matches[k]);
				}
				
				if(matches[k].distance <= mindist)
				{
					mindist = matches[k].distance;
				}
			}
			meandist /= matches.size();
			lastmeandist = mindist/2+meandist/4;
			*/
			int maxitem = 20;
			if(maxitem > matches.size())	maxitem = matches.size()/2;
			maxitem = matches.size();
			for(int k=0;k<maxitem;k++)	GoodMatches.push_back( matches[k] );
			 
			//ROS_INFO("MEAN DISTANCE MATCH = %f // MIN DISTANCE = %f",meandist,mindist);
			
			try
			{
				cv::drawMatches(imgs[0],kpsRestricted,img,kp,GoodMatches,dim);
				//cv::drawMatches(imgs[0],kps[0],img,kp,matches,dim);
			}
			catch(const cv::Exception& e)
			{
				//std::cout << " OPENCV EXCETION // " << std::endl;
				//ROS_INFO("OPENCV EXCEPTION : %s",e.what());
			}
			
			try
			{
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dim).toImageMsg();
				this->img_pub.publish(msg);
			}
			catch(const cv_bridge::Exception& e)
			{
				ROS_INFO("EXCEPTION : %s",e.what());
			}
			
		}
		else	
		{
			//TODO
		}
		
		
		
		return false;
	}
	
	static cv::Mat restrictFeatures(const cv::Mat& features, const int& xs1, const int& ys1, const int& xs2, const int& ys2, const std::vector<cv::KeyPoint>& kp)
	{
		//ROS_INFO("RESTRICTING FEATURES : ...");
		
		std::vector<int> indexes;
		for(int k=0;k<kp.size();k++)
		{
			if( kp[k].pt.x >= xs1 && kp[k].pt.y >= ys1 && kp[k].pt.x <= xs2 && kp[k].pt.y <= ys2)
			{
				indexes.push_back(k);
			}
		}
		
		//ROS_INFO("RESTRICTING FEATURES : %d / %d ...", indexes.size(), features.rows);
		
		if(indexes.size() > 0)
		{
			cv::Mat ret(indexes.size(), features.cols, features.type());
	
			for(int k=0;k<indexes.size();k++)
			{
				for(int j=0;j<features.cols;j++)
				{
					ret.at<char>(k,j) = features.at<char>(indexes[k],j);
				}
			}	
		
			//ROS_INFO("RESTRICTING FEATURES : DONE.");
		
			return ret;
		}
		else
		{
			ROS_INFO("LFEXTRACTOR :: RESTRICTING FEATURES :: NO FEATURES !!!!!...");
			return features;
		}
	}
	
	static void restrictFeatures(const cv::Mat& features, cv::Mat& fout, const int& xs1, const int& ys1, const int& xs2, const int& ys2, const std::vector<cv::KeyPoint>& kp, std::vector<cv::KeyPoint>& kpout)
	{
		//ROS_INFO("RESTRICTING FEATURES : ...");
		
		std::vector<int> indexes;
		for(int k=0;k<kp.size();k++)
		{
			if( kp[k].pt.x >= xs1 && kp[k].pt.y >= ys1 && kp[k].pt.x <= xs2 && kp[k].pt.y <= ys2)
			{
				indexes.push_back(k);
			}
		}
		
		//ROS_INFO("RESTRICTING FEATURES : %d / %d ...", indexes.size(), features.rows);
		
		if(indexes.size() > 0)
		{
			fout = cv::Mat(indexes.size(), features.cols, features.type());
			kpout.clear();
	
			for(int k=0;k<indexes.size();k++)
			{
				for(int j=0;j<features.cols;j++)
				{
					fout.at<char>(k,j) = features.at<char>(indexes[k],j);
				}
				
				kpout.push_back( kp[ indexes[k] ] );
			}	
		
			//ROS_INFO("RESTRICTING FEATURES : DONE.");
		}
		else
		{
			ROS_INFO("LFEXTRACTOR :: RESTRICTING FEATURES :: NO FEATURES !!!!!...");
			fout = features;
			kpout = kp;
		}
	}
	
	void setRegistering(bool rg = true)
	{
		mutexRES.lock();
		this->registering = rg;
		mutexRES.unlock();
	}
	
	void setContinuer(bool c=true)
	{
		mutexRES.lock();
		this->continuer = c;
		mutexRES.unlock();
	}
	
	void compute(const std::vector<cv::Mat>& f, const cv::Mat& img,std::vector<cv::KeyPoint>& kp)
	{
		mutexRES.lock();
		featuresToProcess = f;
		imgToProcess = img;
		kpToProcess = kp;
		mutexRES.unlock();
	}
	
	

};


class LFExtractor
{
	protected :
	
	unsigned int scaler;
	std::mutex mutexRES;
	bool continuer;
	//DATA :
	int treshold;
	int count;
	Mat<int> features;
	std::vector<cv::Mat> descriptors;
	//SIFT/FAST DIY :
	std::vector<Mat<int> > binStrings;
	std::vector<cv::Mat> patchs;
	bool rotationInvariance;
	
	//FAST OPENCV :
	std::vector<cv::KeyPoint> kp;
	
	
	std::vector<cv::Mat> frames;
	cv::Mat frameProcessed;
	
	std::thread* t;
	
	FeatureRegister* fr;
	
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	
	#ifdef haar_use
	image_transport::ImageTransport* itHAAR;
	image_transport::Publisher imgHAAR_pub;
	#endif
	
	image_transport::Subscriber img_sub;
	
	#ifdef haar_use
	cv::Mat detectAndDisplay(const cv::Mat& frame )
	{
		std::vector<cv::Rect> faces;
		cv::Mat frame_gray;
		cv::Mat ret;
		frame.copyTo(ret);

		
		if(frame.channels() == 3)
		{
			try
			{
			cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
			}
			catch(const cv::Exception& e)
			{
				frame.copyTo(frame_gray);
			}
		}
		else
		{
			frame.copyTo(frame_gray);
			
		}
		//cv::equalizeHist( frame_gray, frame_gray );

		//-- Detect faces
		// 1-3 FPS : SLOW BUT SOME DETECTIONS...
		//face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
		
		//10 FPS FAST AND ACCURATE SOMEHOW...
		//face_cascade.detectMultiScale( frame_gray, faces, 1.3, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(60, 60),cv::Size(350,350) );
		
		//15 FPS : some detections...
		face_cascade.detectMultiScale(frame_gray, faces, 2, 0, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(60, 60));
		// 4 FPS : a lot of false detections...
		//face_cascade.detectMultiScale(frame_gray, faces, 2, 0, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(20, 20));
		
		//face_cascade.detectMultiScale( frame_gray, faces);

		for( size_t i = 0; i < faces.size(); i++ )
		{
			cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
			//cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
			cv::ellipse( ret, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

			
			cv::Mat faceROI = frame_gray( faces[i] );
			std::vector<cv::Rect> eyes;

			//-- In each face, detect eyes
			eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
			//eyes_cascade.detectMultiScale( faceROI, eyes, 1.3, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(60, 60),cv::Size(350,350) );

			for( size_t j = 0; j < eyes.size(); j++ )
			{
				cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
				int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
			    //cv::circle( frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
			    cv::circle( ret, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
			}
			
			
		}
		//-- Show what you got
		/*
		try
		{
			cv::imshow( window_name,ret );
		}
		catch(const cv::Exception& e)
		{
		
		}
		*/
		
		return ret;
	}
	

	/** Global variables */
	std::string face_cascade_name;// = "haarcascade_frontalface_alt.xml";
	std::string eyes_cascade_name;// = "haarcascade_eye_tree_eyeglasses.xml";
	cv::CascadeClassifier face_cascade;
	cv::CascadeClassifier eyes_cascade;
	std::string window_name;// = "Capture - Face detection";
	//RNG rng(12345);
	#endif
	
	std::string path2img;
	int typeTracker;
	
	
	LFExtractor(const std::string& path2img_, const int typeTracker_, const int& treshold_ = 100, const unsigned int& scaler_ = 1) : continuer(true), path2img(path2img_), typeTracker(typeTracker_), scaler(scaler_)
	{			
		//DATA :	
		treshold = treshold_;
		count = 0;
		rotationInvariance = false;
		
		it = new image_transport::ImageTransport(nh);		
		#ifdef haar_use
		itHAAR = new image_transport::ImageTransport(nh);
		#endif
		
		img_sub = it->subscribe(path2img.c_str(), 1, &LFExtractor::callback,this);
		#ifdef haar_use
		imgHAAR_pub = itHAAR->advertise("/LFExtractor/HAAR_test", 1);
		#endif
		
		switch(typeTracker)
		{
			case 0:/*SIFT*/
			{
			img_pub = it->advertise("/LFExtractor/SIFT_Features", 1);
			}
			break;
			
			case 1:/*FAST*/
			{
			img_pub = it->advertise("/LFExtractor/FAST_Features", 1);
			}
			break;
			
			default:
			{
			img_pub = it->advertise("/LFExtractor/Features", 1);
			}
			break;
			
		}
		
		#ifdef features_use
		fr = new FeatureRegister(true);
		#else
		fr = NULL;
		#endif
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		#ifdef haar_use
		face_cascade_name = "haarcascade_frontalface_default.xml";
		eyes_cascade_name = "haarcascade_eye.xml";
		window_name = "Capture - Face detection";
		
		if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); throw; };
		if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); throw; };
		#endif
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&LFExtractor::loop, this);
		
		ROS_INFO("LFExtractor::Initialization : OK.");
	}
	
	~LFExtractor()
	{
		
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		
		#ifdef haar_use
		delete itHAAR;
		#endif
		
		if(fr != NULL)	delete fr;
		
		ROS_INFO("LFExtractor::Exiting.");
	}
	
	void callback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("LFExtractor::main.cpp::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		mutexRES.lock();
		//frames.insert(frames.begin(), cv_ptr->image);	
		frames.insert(frames.begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
		
	void loop()
	{
		clock_t timer = clock();
		
		#ifdef haar_use
		cv::namedWindow(window_name,CV_WINDOW_AUTOSIZE);
		#endif
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
					
			if(frames.size())
			{
				//let us dewarped the current frame :
				cv::Mat frameToProcess;
				
				mutexRES.lock();
				frames[frames.size()-1].copyTo(frameToProcess);
				mutexRES.unlock();
				
				
				initDATAS();
				//ROS_INFO("LFExtractor::PROCESS : ...");
				
				mutexRES.lock();
				if(this->typeTracker == 0)
				{
					try
					{
						processFAST(&frameToProcess, &features, &binStrings, &patchs, &treshold, &count,rotationInvariance).copyTo(frameProcessed);
					}
					catch(const cv::Exception& e)
					{
						ROS_INFO("EXCEPTION : %s",e.what());
					}
					
					if(fr != NULL)	fr->compute( descriptors, frameToProcess, kp);
				}
				else if(this->typeTracker == 1)
				{
					try
					{
						processOPENCV(&frameToProcess, &features, &kp, &patchs, &treshold, &count).copyTo(frameProcessed);
					}
					catch(const cv::Exception& e)
					{
						ROS_INFO("EXCEPTION : %s",e.what());
					}
					
					if(fr != NULL)	fr->compute( descriptors, frameToProcess, kp);
				}
				else if(this->typeTracker == 2)
				{
					//ORB :
					try
					{
						processORB(&frameToProcess, &features, &kp, &patchs, &treshold, &count, &descriptors).copyTo(frameProcessed);
					}
					catch(const cv::Exception& e)
					{
						ROS_INFO("EXCEPTION : %s",e.what());
					}
					
					//std::vector<cv::Mat> feats;
					//feats.push_back(descriptors);
					if(fr != NULL)	fr->compute( descriptors, frameToProcess, kp);
				}
				mutexRES.unlock();
				
				//ROS_INFO("LFExtractor::PROCESS : %d features.",count);
				
				mutexRES.lock();
				frames.clear();
				mutexRES.unlock();
				
#ifdef debug_v0		
				count_info++;
		
				if(count_info>10)
				{
					ROS_INFO("LFExtractor::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
					count_info = 0;
				}
				timer = clock();
#endif 
			}
			
			//----------------------------
			//		Publisher
			//----------------------------
		
			//--------------------------------------------
			//
			//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
			//
			//--------------------------------------------
			
			
			#ifdef haar_use
			cv::Mat haarimg;
			//ROS_INFO("LFEXTRACTOR :: HAAR CASCADE : processing ...");
			detectAndDisplay(frameProcessed).copyTo(haarimg);
			//ROS_INFO("LFEXTRACTOR :: HAAR CASCADE : DONE.");
			
			sensor_msgs::ImagePtr msghaar = cv_bridge::CvImage(std_msgs::Header(), "bgr8", haarimg).toImageMsg();
			imgHAAR_pub.publish(msghaar);
			#endif
			
			
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameProcessed).toImageMsg();
			img_pub.publish(msg);
			
			
			mutexRES.lock();
		}
		mutexRES.unlock();

	}
	
	inline void setContinuer(bool c)
	{
		mutexRES.lock();
		continuer = c;
		mutexRES.unlock();	
	}
	
	void initDATAS()
	{
		mutexRES.lock();
		features *= 0;
		binStrings.clear();
		patchs.clear();
		mutexRES.unlock();
	}
	
	std::vector<cv::Mat> getPatchs()
	{
		mutexRES.lock();
		std::vector<cv::Mat> ret = patchs;
		mutexRES.unlock();
		
		return ret;
	}
	
	
	std::vector<cv::Mat> getDescriptors()
	{
		mutexRES.lock();
		std::vector<cv::Mat> ret = descriptors;
		mutexRES.unlock();
		
		return ret;
	}
	
	static cv::Mat processFAST(cv::Mat* im_, Mat<int> *features_, vector<Mat<int> > *binStrings_, vector<cv::Mat> *patchs_, int *treshold_, int *count_, bool rotationInvariance)
	{
		//float factor = 2;
		float factor = 2;
		//float vfactor = 2;
		float vfactor = 1;
		int treshold = (treshold_ != NULL ? *treshold_:20);
		cv::Mat im,imr,rim;
		im = *im_; //cv::imread("../SFM_OCV/data1.pgm");
		//cv::cvtColor(im1,im1,CV_BGR2GRAY);
		//cv::cvtColor(im2,im2,CV_BGR2GRAY);
		cv::resize(im,imr,cv::Size(0,0),1.0/factor,1.0/factor);


		    /*------------------------------*/
		bool show = true;
		int nbrFeatures = 100;
		int patchSize = 11;
		int binLength = 32;
		    BRIEFFAST detector(imr,im,binLength,1,patchSize, nbrFeatures, show);
		    int mode = 0; /* not equal ; 1 =*/
		    detector.run(treshold, mode, rotationInvariance);
		    Mat<int> features(detector.getFeatures());
		    Mat<int> allFeatures(detector.getAllFeatures());
		    *patchs_ = detector.getPatchs();
		    vector<Mat<int> > bstr = detector.getBinaryString();

		    // Add results to image and save.
		    cv::resize(im,rim,cv::Size(0,0),(float)vfactor,(float)vfactor);

		    /*----------------------------*/

		    int count = 0;
		    for(int i=features.getLine();i--;)
		    {
		        for(int j=features.getColumn();j--;)
		        {
		            if(features.get(i+1,j+1) != 0)
		            {
		                count++;

		                circle(rim, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 5, cv::Scalar(255,0,0), 1, 8, 0);
		                ostringstream nbr;
		                nbr << count;
		                //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
		                //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);

		            }

		            if(allFeatures.get(i+1,j+1) != 0)
		            {
		                circle(rim, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 2, cv::Scalar(0,255,0), 1, 8, 0);
		            }

		        }
		    }


		    *im_ = im;


		    if(count_ != NULL)
		        *count_ = detector.getNbrLandmarksFound();

		    *binStrings_ = bstr;
		    *features_ = features;

		    return rim;

	}
	
	cv::Mat processOPENCV(cv::Mat* img, Mat<int>* features, std::vector<cv::KeyPoint>* kp, std::vector<cv::Mat>* patchs, int* treshold, int* count)
	{
		//-- Step 1: Detect the keypoints using SURF Detector
		//int minHessian = 400;
		int fasttreshold = *treshold;//default : 10

		//SurfFeatureDetector detector( minHessian );
		cv::FastFeatureDetector detector( fasttreshold );
		//cv::GoodFeaturesToTrackDetector detector();

		detector.detect( *img, *kp );
		*count = kp->size();

		//-- Draw keypoints
		cv::Mat imgkp;
		cv::drawKeypoints( *img, *kp, imgkp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
		
		//-- Register datas in :
		//features :
		//patchs :
		patchs->clear();
		for(unsigned int k=0;k<kp->size();k++)
		{
			features->set( 1, (*kp)[k].pt.x+1, (*kp)[k].pt.y+1);
			
			int sizepatch = 10;
			cv::Mat currentpatch = extractCV( img, (*kp)[k].pt.x, (*kp)[k].pt.y, sizepatch);
			
			patchs->push_back(currentpatch);
		}
		
		
	
		return imgkp;
	}
	
	cv::Mat processORB(cv::Mat* img, Mat<int>* features, std::vector<cv::KeyPoint>* kp, std::vector<cv::Mat>* patchs, int* treshold, int* count, std::vector<cv::Mat>* descriptors)
	{
		//-- Step 1: Detect the keypoints using SURF Detector
		//int minHessian = 400;

		//cv::FREAK extractor;
		
		//#define BRISK_USE
		#define ORB_USE
		
		#ifdef ORB_USE
		int nfeatures = 1000;
		float scaleFactor = 1.1f;
		int level = 16;
		int edgetreshold = *treshold;
		int firstlevel = 0;
		int wtak = 2;
		int scoretype = cv::ORB::kBytes;//FAST_SCORE;//HARRIS_SCORE;
		int patchsize = 10;
		cv::OrbFeatureDetector detector(nfeatures,scaleFactor,level,edgetreshold,firstlevel,wtak,scoretype,patchsize);
		#endif
		
		#ifdef BRISK_USE
		int tresh=*treshold;
		int octave=8;
		float patternScale=1.0f;
		cv::BRISK detector(tresh,octave,patternScale);
		#endif
		
		cv::Mat desc;
		
		//detector( *img, cv::Mat::ones( cv::Size(img->rows, img->cols),CV_8UC1), *kp, *descriptors, false );
		
		detector.detect(*img,*kp);
		/**/
		detector.compute(*img, *kp, desc);
		//extractor.compute(*img, *kp, desc);
		/**/
		*count = kp->size();
		descriptors->clear();
		descriptors->push_back(desc);

		//-- Draw keypoints
		cv::Mat imgkp;
		cv::drawKeypoints( *img, *kp, imgkp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
		
		//-- Register datas in :
		//features :
		//patchs :
		patchs->clear();
		for(unsigned int k=0;k<kp->size();k++)
		{
			features->set( 1, (*kp)[k].pt.x+1, (*kp)[k].pt.y+1);
			
			int sizepatch = 20;
			cv::Mat currentpatch = extractCV( img, (*kp)[k].pt.x, (*kp)[k].pt.y, sizepatch);
			
			patchs->push_back(currentpatch);
		}
		
		
	
		return imgkp;
	}

	
};





























class PlanarTarget
{
	public :
	
	cv::Mat image;
	Mat<float> rect;
	std::vector<cv::KeyPoint> points;
	std::vector<cv::Mat> descrs;
	Mat<float> data;
	
	PlanarTarget( const cv::Mat& im, const Mat<float>& r, const std::vector<cv::KeyPoint>& p, const std::vector<cv::Mat>& desc, Mat<float>* d = NULL) : rect(r), points(p), descrs(desc), data(Mat<float>(1,1))
	{
		im.copyTo(image);
		
		if(d != NULL)
		{
			data = *d;
		}
	}
	
	
};

class TrackedTarget
{
	public :
	
	PlanarTarget target;
	std::vector<cv::Point> p0;	//Point from the planartarget.
	std::vector<cv::Point> p1;	//Point from the current frame.
	cv::Mat H;
	cv::Mat quad;
	
	TrackedTarget( const PlanarTarget& t, const std::vector<cv::Point>& pt0, const std::vector<cv::Point>& pt1, const cv::Mat& h, const cv::Mat& q ) : target(t), p0(pt0), p1(pt1), H(h)
	{
		q.copyTo(quad);
	}
	
	
};

#define MIN_MATCH_COUNT 4

class PlaneTracker
{
	protected :
	
	cv::OrbFeatureDetector* detector;
	cv::FlannBasedMatcher* matcher;
	
	std::vector<PlanarTarget> targets;
	
	std::vector<cv::KeyPoint> frame_points;
	cv::Mat frame_descrs;
	
	std::vector<Mat<float> > meanTargets;
	
    public :
    
    PlaneTracker( const int& tresh = 100) : frame_descrs(cv::Mat(1,1,CV_8UC1))
    {
        int nfeatures = 1000;
		float scaleFactor = 1.1f;
		int level = 8;
		int	edgetreshold = tresh;
		int firstlevel = 0;
		int wtak = 2;
		int scoretype = /*cv::ORB::kBytes;*//*cv::ORB::FAST_SCORE;*/cv::ORB::HARRIS_SCORE;
		int patchsize = 5;
		
		detector = new cv::OrbFeatureDetector(nfeatures,scaleFactor,level,edgetreshold,firstlevel,wtak,scoretype,patchsize);
        
        //matcher = new cv::FlannBasedMatcher( new cv::flann::LshIndexParams(1, 24, 2) );
        //matcher = new cv::FlannBasedMatcher( new cv::flann::LshIndexParams(30, 16, 2) );
        matcher = new cv::FlannBasedMatcher();
        
	}
	
	~PlaneTracker()
	{
		delete detector;
		delete matcher;
	}

    void add_target(const cv::Mat& image, const Mat<float>& rect, const Mat<float>* data = NULL)
    {
    
        float x0 = rect.get(1,1);
        float y0 = rect.get(2,1);
        float x1 = rect.get(1,2);
        float y1 = rect.get(2,2);
        
        std::vector<cv::KeyPoint> raw_points;
        cv::Mat raw_descrs;
        detect_features(image,raw_points, raw_descrs);
        
        std::vector<cv::KeyPoint> points;
        cv::Mat desc;
        
        FeatureRegister::restrictFeatures(raw_descrs, desc, x0, y0, x1, y1, raw_points, points);
        
        std::vector<cv::Mat> descs;
        if(desc.type()!=CV_32F) 
    	{
			desc.convertTo(desc, CV_32F);
		}
        descs.push_back(desc);
        
        try
        {
        	this->matcher->add(descs);
        	this->matcher->train();
        }
        catch( cv::Exception& e)
        {
        	std::cout << "ADD TARGET : " << e.what() << std::endl;
        }
        
        PlanarTarget target(image, rect, points, descs);
        this->targets.push_back(target);
    }

    void clear()
    {
        this->targets.clear();
        this->matcher->clear();
    }

    std::vector<TrackedTarget> track(const cv::Mat& frame)
    {
        detect_features(frame, this->frame_points, this->frame_descrs);
        
        if( this->frame_points.size() < MIN_MATCH_COUNT)
        {
            std::vector<TrackedTarget> retnone;
        	return retnone;
        }
        
        std::vector<std::vector<cv::DMatch> > matches;
        int knn = 2;
        
        try
        {
        	if(this->frame_descrs.type()!=CV_32F) 
        	{
			    this->frame_descrs.convertTo(this->frame_descrs, CV_32F);
			}
			
        	this->matcher->knnMatch(this->frame_descrs, matches, knn);
        }
        catch(cv::Exception& e)
        {
        	std::cout << "MATCHER KNN : " << e.what() << std::endl;
        	
        }
        
        std::vector<cv::DMatch> finalmatches;
        for(int k=0;k<matches.size();k++)
        {
        	//matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.75]
        	if( matches[k].size() == 2)
        	{
        		if( matches[k][0].distance < matches[k][1].distance * 0.75 )
        		{
        			finalmatches.push_back( matches[k][0] );
        		}
        	}
        }
        
        
        if( finalmatches.size()  < MIN_MATCH_COUNT)
        {
        	std::vector<TrackedTarget> retnone;
        	return retnone;
       	}
       	else
       	{
		    std::map<int, std::vector<cv::DMatch> > matches_by_id;
		    for( int k=0;k<finalmatches.size();k++)
		    {
		        matches_by_id[ finalmatches[k].imgIdx ].push_back( finalmatches[k] );
		    }
		    
		    std::vector<TrackedTarget> tracked;
		    std::map<int, std::vector<cv::DMatch> >::iterator it = matches_by_id.begin();
		    
		    for(;it!=matches_by_id.end();it++)
		    {
		    	int imgIdx = it->first;
		    	std::vector<cv::DMatch> matches = it->second;
	    		
	    		
	    		double max_dist = 0; double min_dist = 1000;
				//-- Quick calculation of max and min distances between keypoints
				for( int i=0;i<matches.size();i++ )
				{ 
					double dist = matches[i].distance;
					if( dist < min_dist )	min_dist = dist;
					if( dist > max_dist )	max_dist = dist;
				}
	    		
			    PlanarTarget target = this->targets[imgIdx];
			    std::vector<cv::Point> p0;
			    std::vector<cv::Point> p1;
			    
			    for(int kk=0;kk<matches.size();kk++)
			    {
			    	//SELECT ONLY THE GOOD MATCHES :
			    	if( matches[kk].distance < max( 2*min_dist, 0.02) )
			    	{
			    		p0.push_back( target.points[ matches[kk].trainIdx].pt );
				    	p1.push_back( this->frame_points[ matches[kk].queryIdx].pt );
				    }
			    }
			    
			    if( p0.size() < MIN_MATCH_COUNT)
			    {
			    	std::vector<TrackedTarget> retnone;
		        	return retnone;
			    }
			    
			    cv::Mat H;
			    try
			    {
			    	cv::Mat p0M(p0.size(),2,CV_32F);
			    	cv::Mat p1M(p0.size(),2,CV_32F);
			    	
			    	for(int k=0;k<p0.size();k++)
			    	{
			    		p0M.at<float>(k,0) = p0[k].x;
			    		p0M.at<float>(k,1) = p0[k].y;
			    		
			    		p1M.at<float>(k,0) = p1[k].x;
			    		p1M.at<float>(k,1) = p1[k].y;
			    	}
			    	
			    	H = cv::findHomography( p0M, p1M, cv::RANSAC, 5.0);
			    }
			    catch( const cv::Exception& e)
			    {
			    	std::cout << " TRACK : FIND HOMO : " << e.what() << std::endl;
			    	std::cout << " P0 : P1 : " << p0.size() << " ; " << p1.size() << std::endl;
			    }
			    
			    std::cout << " H : " << H << std::endl;
			    
			
			    Mat<float>  rect( target.rect);
			    //cv::Mat quadin(2,4,CV_32FC1)
#define point2fUseHomography			    
#ifndef point2fUseHomography			    
			    cv::Mat quadin(4,1,CV_32FC2);
			    cv::Mat quadout(4,1,CV_32FC2);
			    
			    /*
			    quadin.ptr<float>(0)[0] = rect(1,1);
			    quadin.ptr<float>(0)[1] = rect(1,2);
			    
			    quadin.ptr<float>(1)[0] = rect(2,1);
			    quadin.ptr<float>(1)[1] = rect(1,2);
			    
			    quadin.ptr<float>(2)[0] = rect(2,1);
			    quadin.ptr<float>(2)[1] = rect(2,2);
			    
			    quadin.ptr<float>(3)[0] = rect(1,1);
			    quadin.ptr<float>(3)[1] = rect(2,2);
			    */
			    
			    /**/
			    quadin.ptr<float>(0)[0] = rect(1,1);
			    quadin.ptr<float>(0)[1] = rect(1,2);
			    
			    quadin.ptr<float>(1)[0] = rect(1,1);
			    quadin.ptr<float>(1)[1] = rect(2,2);
			    
			    quadin.ptr<float>(2)[0] = rect(2,1);
			    quadin.ptr<float>(2)[1] = rect(2,2);
			    
			    quadin.ptr<float>(3)[0] = rect(2,1);
			    quadin.ptr<float>(3)[1] = rect(1,2);
			    /**/
			    
			    /*
			    quadin.ptr<float>(0)[0] = rect(1,1);
			    quadin.ptr<float>(1)[0] = rect(1,2);
			    
			    quadin.ptr<float>(0)[1] = rect(2,1);
			    quadin.ptr<float>(1)[1] = rect(1,2);
			    
			    quadin.ptr<float>(0)[2] = rect(2,1);
			    quadin.ptr<float>(1)[2] = rect(2,2);
			    
			    quadin.ptr<float>(0)[3] = rect(1,1);
			    quadin.ptr<float>(1)[3] = rect(2,2);
			    */
			    
			    /*
			    quadin(0,0) = rect(1,1);
			    quadin(1,0) = rect(1,2);
			    
			    quadin(0,1) = rect(2,1);
			    quadin(1,1) = rect(1,2);
			    
			    quadin(0,2) = rect(2,1);
			    quadin(1,2) = rect(2,2);
			    
			    quadin(0,3) = rect(1,1);
			    quadin(1,3) = rect(2,2);
			    */
			    
			    //quad = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
			    //quad = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2)
#else
				std::vector<cv::Point2f> quadin;
				int cols = abs(rect(2,2) - rect(1,2));
				int rows = abs(rect(2,1) - rect(1,1 ));
				/*
				quadin.push_back( cv::Point(0,0) ); quadin.push_back( cv::Point( cols, 0 ) );
				quadin.push_back( cv::Point( cols, rows ) ); quadin.push_back( cv::Point( 0, rows ) );
				*/
				quadin.push_back( cv::Point(rect(1,1),rect(2,1) ) ); quadin.push_back( cv::Point( rect(1,1), rect(2,2) ) );
				quadin.push_back( cv::Point( rect(1,2), rect(2,2) ) ); quadin.push_back( cv::Point( rect(1,2), rect(2,1) ) );
				
				std::vector<cv::Point2f> quadout(4);			    
#endif
			    
			    try
			    {
			    	//cv::perspectiveTransform( cv::Mat(quadin), cv::Mat(quadout), H);
			    	cv::perspectiveTransform( quadin, quadout, H);
			    	
			    	std::cout << " QUAD IN QUAD OUT : " << std::endl;
			    	std::cout << quadin << std::endl;
			    	std::cout << quadout << std::endl;
			    	
			    }
			    catch( const cv::Exception& e)
			    {
			    	std::cout << " TRACK : PERSP : " << e.what() << std::endl;
			    	std::cout << " QUADIN : " << quadin << std::endl;
			    	std::cout << " QUADOUT : " << quadout << std::endl;
			    }

			    #ifndef point2fUseHomography
			    tracked.push_back( TrackedTarget( target, p0, p1, H, quadout ) );
			    #else
			    
			    // INFORMATION LOGGING :
			    Mat<float> meanTarget(0.0f,2,1);
			    meanTarget(1,1) = ((float)(quadout[0].x+quadout[2].x))/2;
			    meanTarget(2,1) = ((float)(quadout[0].y+quadout[2].y))/2;
			    meanTargets.push_back( transpose(meanTarget) );
			    
			    writeInFile(std::string("./meanTargets.txt"), meanTargets);
			    
			    cv::Mat quadoutMat = cv::Mat(quadout);
			    std::cout << " PT :: track :: quadout Mat " << quadoutMat << std::endl;
			    
			    tracked.push_back( TrackedTarget( target, p0, p1, H, quadoutMat ) );
			    #endif
			    
			    
		    }
		    
		    //tracked.sort(key = lambda t: len(t.p0), reverse=True)
		    
		    return tracked;
		}
		
	}
	
	
    void detect_features(const cv::Mat& frame, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descrs)
    {
        this->detector->detect(frame, keypoints);
        this->detector->compute(frame, keypoints, descrs);
    }
    
};






class PFExtractor
{
	protected :
	
	unsigned int scaler;
	std::mutex mutexRES;
	bool continuer;
	//DATA :
	PlaneTracker* pt;
	int treshold;
	int count;
	
	cv::Mat frameProcessed;
	std::vector<cv::Mat> frames;
	
	std::thread* t;
	
	int xs1;
	int xs2;
	int ys1;
	int ys2;
	
	
	public :
	
	ros::NodeHandle nh;
	
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	
	
	image_transport::Subscriber img_sub;
	
	std::string path2img;
	
	
	PFExtractor(const std::string& path2img_, const int& treshold_ = 100, const unsigned int& scaler_ = 1) : continuer(true), path2img(path2img_), scaler(scaler_), xs1(100),ys1(100),xs2(200),ys2(200)
	{			
		//DATA :	
		pt = new PlaneTracker(treshold);
		treshold = treshold_;
		count = 0;
		
		it = new image_transport::ImageTransport(nh);
		
		img_sub = it->subscribe(path2img.c_str(), 1, &PFExtractor::callback,this);
		
		img_pub = it->advertise("/PFExtractor/Features", 1);
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		/*-------------------------------------------*/
		
		
		t = new std::thread(&PFExtractor::loop, this);
		
		ROS_INFO("PFExtractor::Initialization : OK.");
	}
	
	~PFExtractor()
	{
		
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete it;
		delete pt;
		
		ROS_INFO("PFExtractor::Exiting.");
	}
	
	void callback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
	
		try
		{
			cv_ptr = cv_bridge::toCvCopy(original_image,enc::BGR8);
		}
		catch( cv_bridge::Exception e)
		{
			ROS_ERROR("PFExtractor::main.cpp::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		cv::Mat frameDownSample;
		int h = cv_ptr->image.rows/scaler;
		//h*=2;
		//int h = 480;
		int w = cv_ptr->image.cols/scaler;
		//w*=2;
		//int w = 640;
		cv::resize(cv_ptr->image,frameDownSample,cv::Size(w,h));
		
		mutexRES.lock();
		frames.insert(frames.begin(), frameDownSample);	
		mutexRES.unlock();
		
	}
		
	void loop()
	{
		clock_t timer = clock();
		
		cv::namedWindow("PLANAR FEATURES",CV_WINDOW_AUTOSIZE);
	    cv::setMouseCallback("PLANAR FEATURES",CallBackMouseFunc);
		
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			cv::Mat frameToProcess;		
			bool frameHasBeenProcessed = false;
			bool initNewPlane = false;
			
			if(frames.size())
			{
				
				mutexRES.lock();
				frames[frames.size()-1].copyTo(frameToProcess);
				mutexRES.unlock();
				
				mutexRES.lock();
				std::vector<TrackedTarget> tt = this->pt->track(frameToProcess);
				mutexRES.unlock();
				
				if(tt.size() > 0)
				{
					//TODO : let us draw those on frameProcessed :
					frameToProcess.copyTo(frameProcessed);
					
					try
					{
						for(auto& tr : tt)
						{
		                	for(int k=0;k<tr.p1.size();k++)
		                	{
		                		int radius = 3;
		                		int thicknessC = 2;
		                    	cv::circle(frameProcessed, tr.p1[k], radius, cv::Scalar(255, 255, 255), thicknessC);
		                    }
		                    
		                    int thickness = 2;
		                    std::vector<cv::Point> quad;
		                    quad.push_back( cv::Point( tr.quad.at<float>(0,0), tr.quad.at<float>(0,1)  ) );
		                    quad.push_back( cv::Point( tr.quad.at<float>(1,0), tr.quad.at<float>(1,1)  ) );
		                    quad.push_back( cv::Point( tr.quad.at<float>(2,0), tr.quad.at<float>(2,1)  ) );
		                    quad.push_back( cv::Point( tr.quad.at<float>(3,0), tr.quad.at<float>(3,1)  ) );
		                    
		                    std::cout << " PFEXTRACTOR :: quadout = " << tr.quad << std::endl;
		                	//cv::polylines(frameProcessed, tr.quad, true, cv::Scalar(255, 255, 255), thickness);
		                	cv::polylines(frameProcessed, quad, true, cv::Scalar(255, 255, 255), thickness);
		                }
		            }
		            catch(const cv::Exception& e)
		            {
		            	std::cout << "DRAWING : " << std::endl;//e.what() << std::endl;
		            	std::cout << " Target : " << tt.size() << std::endl;
		            }
		            
		            frameHasBeenProcessed = true;
				}
				
				mutexRES.lock();
				frames.clear();
				mutexRES.unlock();
				
#ifdef debug_v0		
				count_info++;
		
				if(count_info>10)
				{
					ROS_INFO("PFExtractor::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
					count_info = 0;
				}
				timer = clock();
#endif 
			}
			
			
			//----------------------------
			//		PLANAR FEATURES 
			//----------------------------
			
			if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				break; 
			}


			if(update)
			{
				update = false;
				switch(idx)
				{
					case 0:
					{
						xs1 = Mouse(1,1);
						ys1 = Mouse(2,1);
						std::cout << " UPDATE X,Y 1 : " << xs1 << " / " << ys1 <<  std::endl;
					}
					break;
	
					case 1:
					{
						xs2 = Mouse(1,1);
						ys2 = Mouse(2,1);
						std::cout << " UPDATE X,Y 2 : " << xs2 << " / " << ys2 <<  std::endl;
						
						//TODO : set the planar tracker features
						initNewPlane = true;
					}
					break;
				}
			}
			
			mutexRES.lock();
			cv::Mat dummy(10,10,CV_32FC1);
			
			if( frames.size() > 0)
			{
				frames[frames.size()-1].copyTo(dummy);
			}
			else if( frameHasBeenProcessed )
			{
				frameToProcess.copyTo(dummy);
			}
			else
			{
				frameToProcess.copyTo(dummy);
				frameToProcess.copyTo(frameProcessed);
			}
			mutexRES.unlock();
			
			
			if(initNewPlane)
			{
				Mat<float> rect(2,2);
				rect(1,1) = xs1;
				rect(2,1) = ys1;
				
				rect(1,2) = xs2;
				rect(2,2) = ys2;
				
				this->pt->add_target( dummy, rect);
			}
			
			
			try
			{
				cv::Point p1(xs1,ys1);
				cv::Point p2(xs2,ys2);
				cv::Scalar color(255,0,0);
				int thickness = 3;
				int lineType = 8;
				int shift = 0;			
				cv::rectangle(dummy,p1, p2, color, thickness, lineType, shift);
				cv::imshow("PLANAR FEATURES",dummy);
				
			}
			catch(const cv::Exception& e)
			{
			
			}
			
			
			//----------------------------
			//		Publisher
			//----------------------------
		
			//--------------------------------------------
			//
			//	Convert CvImage to ROS image message and publish it to camera/image_processed topic
			//
			//--------------------------------------------
					
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameProcessed).toImageMsg();
			img_pub.publish(msg);
			
			
			mutexRES.lock();
		}
		mutexRES.unlock();

	}
	
	inline void setContinuer(bool c)
	{
		mutexRES.lock();
		continuer = c;
		mutexRES.unlock();	
	}
	
	

};



#endif
