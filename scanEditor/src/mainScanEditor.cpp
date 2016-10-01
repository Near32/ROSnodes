#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define debug_v0
//#define debug_v1
#define upsampling
#define medianfilter
#define extmedianfilter
#define sharpenfilter
#define gradientfilter
#define smoothfilter


int count_info = 51;

class scanEditor 
{
	protected :
	
	//DATA :
	sensor_msgs::LaserScan scan_out;
	int factor;
	int med_size;
	float deltaDiff;
	float offset_grad;
	
	float angle_min_true;
	float angle_max_true;
		
	
	public :
	
	ros::NodeHandle nh;
	
	ros::Publisher scan_pub;
#ifdef upsampling
	ros::Publisher scan_ups_pub;
#endif

#ifdef medianfilter
	ros::Publisher scan_med_pub;
#endif

#ifdef extmedianfilter
	ros::Publisher scan_extmed_pub;
#endif

#ifdef sharpenfilter
	ros::Publisher scan_sharp_pub;
#endif
	
#ifdef gradientfilter
	ros::Publisher scan_grad_pub;
#endif

#ifdef smoothfilter
	ros::Publisher scan_smooth_pub;
#endif

	ros::Subscriber scan_sub;
	
	scanEditor(int factor_=100, int med_size_ = 10, float deltaDiff_ = 0.2f, float offset_grad_ = 2.0f, float a_min = -90.0f, float a_max = 90.0f) : factor(factor_), med_size(med_size_), deltaDiff(deltaDiff_), offset_grad(offset_grad_), angle_min_true(a_min), angle_max_true(a_max)
	{		
		scan_sub = nh.subscribe("scan",10, &scanEditor::callback, this);
		scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_edited",10);
#ifdef upsampling
		scan_ups_pub = nh.advertise<sensor_msgs::LaserScan>("scan_upsampled",10);
#endif

#ifdef medianfilter
		scan_med_pub= nh.advertise<sensor_msgs::LaserScan>("scan_median_filter",10);
#endif

#ifdef extmedianfilter
		scan_extmed_pub= nh.advertise<sensor_msgs::LaserScan>("scan_extmedian_filter",10);
#endif

#ifdef sharpenfilter
		scan_sharp_pub= nh.advertise<sensor_msgs::LaserScan>("scan_sharp_filter",10);
#endif

#ifdef gradientfilter
		scan_grad_pub= nh.advertise<sensor_msgs::LaserScan>("scan_gradient_filter",10);
#endif

#ifdef smoothfilter
		scan_smooth_pub= nh.advertise<sensor_msgs::LaserScan>("scan_smooth_filter",10);
#endif

	}
	
	void callback( const sensor_msgs::LaserScan& scan_in)
	{
		scan_out = scan_in;
		//----------------------
		//----------------------
		//	ANGLE SETTINGS : 
		//	values returned by the hokuyo node aren't the real ones...
		//-------------------------------------------
		//scan_out.angle_min = angle_min_true;
		//scan_out.angle_max = angle_max_true;	
		//scan_out.angle_increment *= (angle_max_true-angle_min_true)/180.0f;
		//----------------------
		//----------------------
		
		
		clock_t timer = clock();
		
		
		scan_out.intensities = scan_out.ranges;
		float r_min = scan_out.range_min;
		float r_max = scan_out.range_max;

		float a_min = scan_out.angle_min;
		float a_max = scan_out.angle_max;
		float a_inc = scan_out.angle_increment;
		
		float t_inc = scan_out.time_increment;
		//time between measurements in seconds, 
		//necessary in order to interpolate the position.
		
		float r_off = r_max-0.1;
		int nOri = (a_max-a_min)/a_inc+1;
		int n = nOri;
		//int n = scan_out.ranges.size();
		//ROS_INFO("nbr scan points : %d or %d.", n, nOri);
		//--------------------------
		//		Editor:Normalization
		//--------------------------
		for(int i=n;i--;)
		{
			if(isnan(scan_out.ranges[i]) )
			 	scan_out.ranges[i] = r_off;
			 	
			if(scan_out.ranges[i] > r_max || scan_out.ranges[i] < r_min)
			{
				//we discard/filter the value if it is not within the possible ranges value of the LRF.
#ifdef debug_v1				
				ROS_INFO("range value off : %f ; at angle : %f.", scan_out.ranges[i],a_min+n*a_inc*180.0f/3.14159f);
#endif				
				scan_out.ranges[i] = r_off;
				
				
			}
			
			scan_out.intensities[i] = scan_out.ranges[i]/(r_max-r_min);
			
		}
		
#ifdef upsampling
		//int factor = 100;
		sensor_msgs::LaserScan temp_scan_ups = scan_out;
		temp_scan_ups.angle_min = a_min;
		temp_scan_ups.angle_max = a_max;
		
		temp_scan_ups.angle_increment = a_inc/factor;
		temp_scan_ups.time_increment = t_inc/factor;
		
		temp_scan_ups.scan_time = scan_out.scan_time;
		
		temp_scan_ups.range_min = r_min;
		temp_scan_ups.range_max = r_max;
		
		//temp_scan_ups.ranges.reserve(scan_out.ranges.size()*factor);
		//temp_scan_ups.intensities.reserve(scan_out.intensities.size()*factor);
		temp_scan_ups.ranges = std::vector<float>(scan_out.ranges.size()*factor);
		temp_scan_ups.intensities = std::vector<float>(scan_out.intensities.size()*factor);
		
		int idx_ups = factor*n-1;
		for(int i=n-1;i--;)
		{
				float diff = scan_out.ranges[i+1]-scan_out.ranges[i];
				float inc_diff = diff/(float)factor;
				
				for(int j=factor;j--;)
				{
					temp_scan_ups.ranges[idx_ups] = scan_out.ranges[i]+j*inc_diff;
					temp_scan_ups.intensities[idx_ups] = scan_out.intensities[i];
					idx_ups--;
				}
		}
		//scan_out = temp_scan_ups;
#endif
		

		
		
#ifdef medianfilter
		#ifdef upsampling
		sensor_msgs::LaserScan temp_scan_med = temp_scan_ups;		
		sensor_msgs::LaserScan med_temp = temp_scan_ups;
		n=factor*nOri;
		#else
		sensor_msgs::LaserScan temp_scan_med = scan_out;		
		sensor_msgs::LaserScan med_temp = scan_out;
		#endif
		
		//int med_size = 20;
		float med_mean = 0.0f;
		for(int i=n;i--;)
		{
			if(i>med_size/2 || i<n-med_size/2)
			{
				float val = 0.0f;
				for(int j=med_size/2;j>0;j--)
				{
					val+= med_temp.ranges[i-(j+1)]+med_temp.ranges[i+(j+1)];
				}
				val /= (float)(med_size+1);				
				
				temp_scan_med.ranges[i] = med_temp.ranges[i]/(med_size+1) + val;
				
				med_mean += temp_scan_med.ranges[i];
			}
			else
				temp_scan_med.ranges[i] = r_off;
			
		}
		
		med_mean /= (n-med_size);
		
		//REGULARIZATION :
		for(int i=0;i<=med_size/2;i++)
		{
			temp_scan_med.ranges[i] = med_mean;
			temp_scan_med.ranges[n-(i+1)] = med_mean;
		}
		
		//scan_out = temp_scan_med;
#endif
		
#ifdef extmedianfilter
		#ifdef upsampling
		sensor_msgs::LaserScan temp_scan_extmed = temp_scan_ups;		
		n=factor*nOri;
		#else
		sensor_msgs::LaserScan temp_scan_extmed = scan_out;
		#endif
		
		//int med_size = 20;
		//float deltaDiff = 0.2; //200cm.
		for(int i=n;i--;)
		{
			if(abs(temp_scan_extmed.ranges[i]-temp_scan_med.ranges[i]) > deltaDiff)
			{	
				temp_scan_extmed.ranges[i] = temp_scan_med.ranges[i];
			}
			
		}
		
		//scan_out = temp_scan_extmed;
#endif
	

#ifdef sharpenfilter
		#ifdef extmedianfilter
		sensor_msgs::LaserScan temp_scan_sharp = temp_scan_extmed;		
		sensor_msgs::LaserScan sharp_temp = temp_scan_extmed;
		n=factor*nOri;
		#elif defined medianfilter
		sensor_msgs::LaserScan temp_scan_sharp = temp_scan_med;		
		sensor_msgs::LaserScan sharp_temp = temp_scan_med;
		n=factor*nOri;
		#elif defined upsampling
		sensor_msgs::LaserScan temp_scan_sharp = temp_scan_ups;		
		sensor_msgs::LaserScan sharp_temp = temp_scan_ups;
		n=factor*nOri;
		#else
		sensor_msgs::LaserScan temp_scan_sharp = scan_out;		
		sensor_msgs::LaserScan sharp_temp = scan_out;
		#endif
		
		float sharp_phi1 = -1.0f;
		float sharp_phi2 = 4.0f;
		float sharp_sumphii = 1.0f/(2*sharp_phi1+sharp_phi2);
		for(int i=n;i--;)
		{
			if(i>0 || i<n-1)
			{
				float val = sharp_phi1*(sharp_temp.ranges[i-1]+sharp_temp.ranges[i+1]);
				temp_scan_sharp.ranges[i] = sharp_sumphii*(sharp_phi2*sharp_temp.ranges[i] + val);
			}
			
			//ROS_INFO("sharp filter : %d : val = %f.", i, temp_scan_sharp.ranges[i]);
		}
		
		//scan_out = temp_scan_sharp;
#endif
	
#ifdef gradientfilter	
		#ifdef sharpenfilter
		sensor_msgs::LaserScan temp_scan_grad = temp_scan_sharp;		
		sensor_msgs::LaserScan grad_temp = temp_scan_sharp;
		n=factor*nOri;
		#elif defined extmedianfilter
		sensor_msgs::LaserScan temp_scan_grad = temp_scan_extmed;		
		sensor_msgs::LaserScan grad_temp = temp_scan_extmed;
		n=factor*nOri;
		#elif defined  medianfilter
		sensor_msgs::LaserScan temp_scan_grad = temp_scan_med;		
		sensor_msgs::LaserScan grad_temp = temp_scan_med;
		n=factor*nOri;
		#elif defined upsampling
		sensor_msgs::LaserScan temp_scan_grad = temp_scan_ups;		
		sensor_msgs::LaserScan grad_temp = temp_scan_ups;
		n=factor*nOri;
		#else
		sensor_msgs::LaserScan temp_scan_grad = scan_out;		
		sensor_msgs::LaserScan grad_temp = scan_out;
		#endif
		
		for(int i=n;i--;)
		{
			//gradient :
			if(i>0 || i<n-1)
			{
				//float offset_grad = 2.0f;
				temp_scan_grad.ranges[i] = offset_grad + 0.5*(grad_temp.ranges[i-1]-grad_temp.ranges[i+1]);
			}
			else
				temp_scan_grad.ranges[i] = offset_grad;
		}
		//scan_out = temp_scan_grad;
#endif


#ifdef smoothfilter	
		#ifdef gradientfilter
		sensor_msgs::LaserScan temp_scan_smooth = temp_scan_grad;		
		sensor_msgs::LaserScan smooth_temp = temp_scan_grad;
		n=factor*nOri;
		#elif defined sharpenfilter
		sensor_msgs::LaserScan temp_scan_smooth = temp_scan_sharp;		
		sensor_msgs::LaserScan smooth_temp = temp_scan_sharp;
		n=factor*nOri;
		#elif defined extmedianfilter
		sensor_msgs::LaserScan temp_scan_smooth = temp_scan_extmed;		
		sensor_msgs::LaserScan smooth_temp = temp_scan_extmed;
		n=factor*nOri;
		#elif defined  medianfilter
		sensor_msgs::LaserScan temp_scan_smooth = temp_scan_med;		
		sensor_msgs::LaserScan smooth_temp = temp_scan_med;
		n=factor*nOri;
		#elif defined upsampling
		sensor_msgs::LaserScan temp_scan_smooth = temp_scan_ups;		
		sensor_msgs::LaserScan smooth_temp = temp_scan_ups;
		n=factor*nOri;
		#else
		sensor_msgs::LaserScan temp_scan_smooth = scan_out;		
		sensor_msgs::LaserScan smooth_temp = scan_out;
		#endif
		
		int smooth_size = 3;
		float smooth_phi = 1.0f;
		float smooth_phi1[smooth_size];
		for(int k=smooth_size;k--;)	smooth_phi1[k] = 1.0f;
		
		float smooth_mean = 0.0f;
		for(int i=n;i--;)
		{
			if(i>smooth_size || i<n-smooth_size)
			{
				float val = 0.0f;
				for(int k=smooth_size;k--;)
					val += smooth_phi1[k]*(smooth_temp.ranges[i-(k+1)]+smooth_temp.ranges[i+(k+1)]);
					
				temp_scan_smooth.ranges[i] = (smooth_phi*smooth_temp.ranges[i] + val)/(2*smooth_size+1);
				
				if(i>med_size/2 || i<n-med_size/2)
					smooth_mean+= temp_scan_smooth.ranges[i];
				// the correct mean which do not comprise the extra values that cannot be filtered...
			}
			
			//ROS_INFO("sharp filter : %d : val = %f.", i, temp_scan_sharp.ranges[i]);
		}
		//scan_out = temp_scan_smooth;
		smooth_mean /= n;
		
		for(int i=smooth_size+1;i--;)	
		{
			temp_scan_smooth.ranges[i] = smooth_mean;
			temp_scan_smooth.ranges[n-i] = smooth_mean;
		}
		
		temp_scan_smooth.intensities[0] = smooth_mean;
		temp_scan_smooth.intensities[1] = med_size;
		
#endif
		//----------------------------
		//----------------------------
		
		
		//----------------------------
		//		Publisher
		//----------------------------
		scan_pub.publish(scan_out);
#ifdef upsampling
		scan_ups_pub.publish(temp_scan_ups);
#endif

#ifdef medianfilter
		scan_med_pub.publish(temp_scan_med);
#endif
	
#ifdef extmedianfilter
		scan_extmed_pub.publish(temp_scan_extmed);
#endif

#ifdef sharpenfilter
		scan_sharp_pub.publish(temp_scan_sharp);
#endif
		
#ifdef gradientfilter
		scan_grad_pub.publish(temp_scan_grad);
#endif

#ifdef smoothfilter
		scan_smooth_pub.publish(temp_scan_smooth);
#endif
		//----------------------------
		//----------------------------
		
#ifdef debug_v0		
		count_info++;
		
		if(count_info>50)
		{
			ROS_INFO("scanEditor::frequency : %f Hz.", (float)(1.0f/((float)(clock()-timer)/CLOCKS_PER_SEC)) );
			count_info = 0;
		}
#endif 

	}
	
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv,"scanEditor");
	
	ros::NodeHandle nh;
	
	int factor = 20;
	int med_size = 10;
	float deltaDiff = 0.2f;
	float offset_grad = 1.0f;
	
	float a_min_true = -90.0f;
	float a_max_true = 90.0f;
	
	if(argc>1)
		factor = atoi(argv[1]);
	if(argc>2)
		med_size = atoi(argv[2]);
	if(argc>3)
		deltaDiff = atof(argv[3]);
	if(argc>4)
		offset_grad = atof(argv[4]);
	if(argc>5)
		a_min_true = atof(argv[5]);
	if(argc>6)
		a_max_true = atof(argv[6]);
		
	scanEditor se(factor,med_size,deltaDiff,offset_grad, a_min_true, a_max_true);
	
	//se.scan_sub = se.nh.subscribe("scan",10, &scanEditor::callback, &se);
	
	ROS_INFO("scanEditor::Streaming data : no error.");
	ROS_INFO("scanEditor::Factor upsampling = %d.",factor);
	ROS_INFO("scanEditor::Median filter size = %d.", med_size);
	ROS_INFO("scanEditor::ExtendedMedian filter DeltaDiff = %f.", deltaDiff); 
	ROS_INFO("scanEditor::Gradient offset = %f.", offset_grad);
	ROS_INFO("scanEditor::Angle MIN = %f.", a_min_true);
	ROS_INFO("scanEditor::Angle MAX = %f.", a_max_true);	
	
	ros::spin();
	
	return 0;
	
}
