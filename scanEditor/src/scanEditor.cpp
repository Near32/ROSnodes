#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class ScanEditor 
{
	protected :
	
	ros::NodeHandle nh_;
	
	ros::Subscriber scan_sub;
	
	ros::Publisher scan_pub;
	
	//DATA :
	sensor_msgs::LaserScan scan_out;
		
	
	public :
	
	ScanEditor() : scan_sub(nh,"scan",10, &ScanEditor::callback)
	{
		scan_pub = nh.advertise<sensor_msgs::LaserScan>("scanEditor/scan_edited",100);	
	}
	
	void callback( const sensor_msgs::LaserScanConstPtr& scan_in)
	{
		scan_out = *scan_in;
		
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

