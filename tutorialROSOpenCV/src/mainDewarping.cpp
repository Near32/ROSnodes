#include <iostream>
#include <string>
#include <thread>
#include <mutex>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "./DewarpNode/DewarpNode.h"

namespace enc = sensor_msgs::image_encodings;


int count_info = 51;
#define debug_v0		

class imgEditor 
{
	protected :
	
	std::mutex mutexRES;
	bool continuer;
	//DATA :
	std::vector<cv::Mat> frames;
	cv::Mat frameProcessed;
	
	std::thread* t;
	
	DewarpNode* dn;
	
	
	public :
	
	ros::NodeHandle nh;
	
	image_transport::ImageTransport* it;
	image_transport::Publisher img_pub;
	image_transport::Subscriber img_sub;
	std::string path2img;
	
	imgEditor(const std::string& path2img_) : continuer(true), path2img(path2img_)
	{				
		it = new image_transport::ImageTransport(nh);		
		img_sub = it->subscribe(path2img.c_str(), 1, &imgEditor::callback,this);
		img_pub = it->advertise("/camera/imageDewarped", 1);
		
		dn = new DewarpNode();
		t = new std::thread(&imgEditor::loop, this);
		
		ROS_INFO("imgEditor::Initialization : OK.");
	}
	
	~imgEditor()
	{
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
		delete dn;
		delete it;
		
		ROS_INFO("imgEditor::Exiting.");
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
			ROS_ERROR("imageDewarper::main.cpp::cv_bridge exception : %s", e.what());
			return ;
		}
		//------------------------------------------------
		//		Stack of Frames
		//-----------------------------------------------
		mutexRES.lock();
		frames.insert(frames.begin(), cv_ptr->image);	
		mutexRES.unlock();
		
	}
		
	void loop()
	{
		clock_t timer = clock();
		
		mutexRES.lock();
		while(continuer)
		{
			mutexRES.unlock();
			
			
			
			
			
			if(frames.size())
			{
				//let us dewarped the current frame :
				
				mutexRES.lock();
				frames[frames.size()-1].copyTo(frameProcessed);
				mutexRES.unlock();
				
				
				*dn << frameProcessed;
				*dn >> frameProcessed;
			
				mutexRES.lock();
				frames.clear();
				mutexRES.unlock();
				
#ifdef debug_v0		
				count_info++;
		
				if(count_info>50)
				{
					ROS_INFO("imgEditor::FPS : %f.", CLOCKS_PER_SEC/((float)(clock()-timer)) );
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
			
			/*
			cv_bridge::CvImage imgProcessed;
			imgProcessed.image = frameProcessed;
			img_pub.publish(imgProcessed.toImageMsg());
			*/
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


int main(int argc, char* argv[])
{
	ros::init(argc, argv,"imgEditor");
	
	std::string path2img("/camera/image_raw");
	if(argc>1)
	{
		path2img = std::string(argv[1]);
	}
	
	imgEditor ie(path2img);
	
	ros::spin();
	
	return 0;
	
}
