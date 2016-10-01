#ifndef SCAN_EDITOR
#define SCAN_EDITOR

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class scanEditor 
{
	protected :
	
	ros::NodeHandle* nh_;
	ros::Publisher scan_pub;
	
	//DATA :
	sensor_msgs::LaserScan scan_out;
		
	
	public :
	
	scanEditor(ros::NodeHandle *nh)
	{
		nh_ = nh;
		scan_pub = nh->advertise<sensor_msgs::LaserScan>("scan_edited",10);
	}
	
	void callback( const sensor_msgs::LaserScan& scan_in)
	{
		scan_out = scan_in;
		
		//--------------------------
		//		Editor
		//--------------------------
		
		
		
		
		
		//----------------------------
		//----------------------------
		
		
		//----------------------------
		//		Publisher
		//----------------------------
		scan_pub.publish(scan_out);
		//----------------------------
		//----------------------------
	}
	
};


#endif
