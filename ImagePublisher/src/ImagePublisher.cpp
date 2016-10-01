#include <iostream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ImagePublisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/imageLOOP", 1);
  
  string path = string("/home/kevin/rosbuild_ws/package_dir/ImagePublisher/src");
  string ImagePath = path+string("/LSD_room/images/00001.png");
  
  cv::Mat image = cv::imread(ImagePath.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  int freq = 10;
  int nbrIm = 2100;
  int count = 0;
  cout << "Usage : " << argv[0] << " [frequency = "<<freq<<" Hz] [number of image = "<<nbrIm<<"]." << endl;
  
  if(argc > 1)
  	freq = atoi(argv[1]);
  if(argc > 2)
  	nbrIm = atoi(argv[2]);
   
  ros::Rate loop_rate(freq);
  
  
  while (nh.ok()) 
  {
  
    if(count < 10)
    	ImagePath = path + string("/LSD_room/images/0000");
    else if(count < 100 )
    	ImagePath = path + string("/LSD_room/images/000");
    else if(count < 1000)
    	ImagePath = path + string("/LSD_room/images/00");
    else if(count < 10000)
    	ImagePath = path + string("/LSD_room/images/0");
    
    stringstream ss;
    ss << count;	
    ImagePath = ImagePath + ss.str()+string(".png");
    
    image = cv::imread(ImagePath.c_str(), CV_LOAD_IMAGE_COLOR);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    
    pub.publish(msg);
    
    count++;
    if(count > nbrIm)
    	count = 1;
    	
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
}
