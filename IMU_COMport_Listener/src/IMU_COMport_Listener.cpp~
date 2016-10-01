#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "comport.h"
#include "Quaternion.h"
#include "EKF/EKF.h"

#include <iostream>
#include <string>
using namespace std;


#define debug_lvl1
//#define debug_lvl2
//#define debug_lvl3

#define Gacc 9.81f
//#define COMPL1		//COMPLEMENTARY FILTER 1st ORDER
#define EKF_USE		// KALMAN FILTER
#define EKF_ACCGYRO

void Euler2Quaternion(geometry_msgs::Quaternion* q, float roll, float pith, float yaw);


class IMUListener
{

	public :
	
	IMUListener(int rate_ = 10, float regul_ = 1.0f, int idxCOM = 0 /*ttyACM0*/, int baudrate_ = 9600) : rate(rate_), regul(regul_), codeCOMPORT(24+idxCOM), baudrate(baudrate_)
	{
	
#ifdef debug_lvl1
		//Gestion ecriture dans un fichier :
		filepath = string("/home/kevidena/ROS/sandbox/IMU_COMport_Listener/src/log.txt");
		log = fopen(filepath.c_str(), "w+");
		if(log == NULL)
		{
			cout << "ERROR : cannot open the file LOG." << endl;
			exit(1);
		}
		else
			cout << "File opened LOG." << endl;
#endif

				
		imu_pub = nh.advertise<sensor_msgs::Imu>("IMUListener/imu",10);
		
		path_pub = nh.advertise<nav_msgs::Path>("IMUListener/Path",10);
		pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("IMUListener/POSESTAMPED",10);
		
		const char* filepath("/home/kevidena/ROS/sandbox/IMU_COMport_Listener/src/resultat.txt");
		instanceCOMPORT = new Comport(codeCOMPORT /*25 ttyACM1 24 for ttyACM0*/, baudrate, "8N1", filepath);
		imu_data = sensor_msgs::Imu();
		
		rangeAcc = 2.0f*Gacc;
		precisionAcc = pow(2.0f, 16-1);
		rangeGyro = 1.0f*250.0f;
		precisionGyro = pow(2.0f, 16-1);
		sgyro = rangeGyro/precisionGyro;
		sacc = rangeAcc/precisionAcc;
		
		sacx = atoi("AcX");
		sacy = atoi("AcY");
		sacz = atoi("AcZ");
		sgyx = atoi("GyX");
		sgyy = atoi("GyY");
		sgyz = atoi("GyZ");
		
		acx = 0;
		acy = 0;
		acz = 0;
		gyx = 0;
		gyy = 0;
		gyz = 0;
		
		angleX = 0.0f;
		angleY = 0.0f;
		angleZ = 0.0f;
		dt = 1.0f/rate;
		
		pose = new Mat<float>(0.0f, 3,1);
		biasAcc = new Mat<float>(0.0f,3,1);
		initBias = false;
		initBiasAccX = false;
		initBiasAccY = false;
		initBiasAccZ = false;
		
		info_count = 1001;
		
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		//EKF :
		#ifndef EKF_ACCGYRO
		nbrstate = 12;
		nbrcontrol = 0;
		nbrobs = 6;
		dtEKF = dt;
		
		stdnoise = 1;
		stdnoise_obs = 1;

		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dtEKF,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
		//unstable if the velocity is propagated...
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)dtEKF,i,nbrstate/2+i);
		A.afficher();
		instanceEEKF->initA(A);
		
		Mat<float> C((float)0,nbrobs,nbrstate);
		for(int i=1;i<=nbrobs;i++)	C.set((float)1,i,nbrobs+i);
		C.afficher();
		instanceEEKF->initC(C);
		
		/*
		Mat<float> B((float)0,nbrstate,nbrcontrol);
		for(int i=1;i<=nbrcontrol;i++)	B.set((float)1,nbrcontrol+i,i);
		B.afficher();
		instanceEEKF->initB(B);
		*/
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	Q.set( stdnoise, i,i);
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		for(int i=1;i<=nbrobs;i++)	R.set( stdnoise_obs, i,i);
		R.afficher();
		instanceEEKF->initR(R);
		
		#else
		
		nbrstate = 6;
		nbrcontrol = 3;
		nbrobs = 2;
		dtEKF = dt;
		
		stdnoise = 3e-1;
		stdnoise_obs = 1e-5;

		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dtEKF,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	A.set((float)1,i,i);
		//unstable if the velocity is propagated...
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)-dtEKF,i,nbrstate/2+i);
		A.afficher();
		instanceEEKF->initA(A);
		
		Mat<float> C((float)0,nbrobs,nbrstate);
		for(int i=1;i<=nbrobs;i++)	C.set((float)1,i,i);
		C.afficher();
		instanceEEKF->initC(C);
		
		
		Mat<float> B((float)0,nbrstate,nbrcontrol);
		for(int i=1;i<=nbrcontrol;i++)	B.set((float)1,i,i);
		B.afficher();
		instanceEEKF->initB(B);
		
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate/2;i++)	Q.set( stdnoise, i,i);
		//bias :
		for(int i=nbrstate/2+1;i<=nbrstate;i++)	Q.set( stdnoise*1e-5, i,i);
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		for(int i=1;i<=nbrobs;i++)	R.set( stdnoise_obs, i,i);
		R.afficher();
		instanceEEKF->initR(R);
		
		#endif
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		
		this->mainLoop();	
	}
	
	~IMUListener()
	{
		delete instanceCOMPORT;
		delete pose;
		delete biasAcc;
		delete instanceEEKF;
		
#ifdef debug_lvl1
		//------------------------------------
		//Fermeture du fichier :
		if(fclose(log) == EOF)
		{
			cout << "ERROR : cannot close the file." << endl;
			exit(1);
		}
		else
			cout << "File closed." << endl;
			
		//--------------------------------
		//--------------------------------
#endif				
	}
	
	void mainLoop()
	{
		ros::Rate r(rate);
		
		while(ros::ok())
		{
			clock_t timer = clock();
			
			if(instanceCOMPORT->listen() != 0)
			{
				//at this stage, we have characters waiting for us in the buffer.
				string currentDATA(instanceCOMPORT->buffer);
				instanceCOMPORT->clear();
				//data += currentDATA;

#ifdef debug_lvl2	
				ROS_INFO( (string( "RECEIVED : ")+currentDATA).c_str());
				//watch out cause if the rate is too high, then the value changed isn't filling up all the buffer.
#endif				
				
				//-------------------------
				//		PARSING :
				//------------------------
				int length = currentDATA.length();
				
				for(int k=0;k<length;k++)
				{
					if(currentDATA[k] == '=')
					{
						//then before that we have a name+' ' and after that, a ' '+value+" |" :
						if(k>4 && k<length-4)
						{
							
							/*
							int l = 0;
							char buffer[10];
							while(k+l <length-4 && l<10 && currentDATA[k+2+l] != ' ')
							{
								char lettre = currentDATA[k
							}
							*/
							string name;
							char namechar[4];
							currentDATA.copy(namechar, 3, k-4);
							namechar[3] ='\0';
							name = string(namechar);
							
							/*
							char lettre = currentDATA[k-4];
							string name =  string((const char*)&lettre);
							lettre = currentDATA[k-3];
							name += string((const char*)&lettre);
							lettre = currentDATA[k-2];
							name += string((const char*)&lettre);
							*/
							
							
							string value;
							int nbrdigit = 0;
							while(currentDATA[k+2+nbrdigit] != '\0' && currentDATA[k+2+nbrdigit] != ' ')
							{
								nbrdigit++;
							}
							char valuechar[nbrdigit];
							currentDATA.copy(valuechar, nbrdigit-1, k+2);
							valuechar[nbrdigit-1] ='\0';
							value = string(valuechar);
							
							
#ifdef debug_lvl2						
							string temp = " name = "+ name ;//+ " ; value = " + value;
							ROS_INFO( temp.c_str() );
							temp = " value = " + value;
							ROS_INFO( temp.c_str() );							
#endif
							
							if( name.compare(string("AcX")) == 0 )
							{
#ifdef debug_lvl3							
								ROS_INFO("ACX SET.");
#endif								
								acx = ((float)atoi(value.c_str()));// * rangeAcc)/precisionAcc;
								
								if(!initBiasAccX)
								{
									initBiasAccX = true;
									biasAcc->set(acx,1,1);
									
									if(initBiasAccY && initBiasAccZ)
										initBias = true;
								}
							}
							else if( name.compare(string("AcY")) == 0 )
							{
								acy = ((float)atoi(value.c_str()));// * rangeAcc)/precisionAcc;
								
								if(!initBiasAccY)
								{
									initBiasAccY = true;
									biasAcc->set(acy,2,1);
									
									if(initBiasAccX && initBiasAccZ)
										initBias = true;
								}
							}
							else if( name.compare(string("AcZ")) == 0 )
							{
								acz = ((float)atoi(value.c_str()));// * rangeAcc)/precisionAcc;
								
								if(!initBiasAccZ)
								{
									initBiasAccZ = true;
									biasAcc->set(acz,3,1);
									
									if(initBiasAccY && initBiasAccX)
										initBias = true;
								}	
							}
							else if( name.compare(string("GyX")) == 0 )
								gyx = atoi(value.c_str());// * (rangeGyro/precisionGyro);	//in degrees.
							else if( name.compare(string("GyY")) == 0 )
								gyy = atoi(value.c_str());// * (rangeGyro/precisionGyro);
							else if( name.compare(string("GyZ")) == 0 )
								gyz = atoi(value.c_str());// * (rangeGyro/precisionGyro);
						
						}
					
					}
					else if(currentDATA[k] == '\0')
						k = length;
				}
				
				//-------------------------
				//-------------------------
				
				
			}
			
			angular_vel.x = (float)gyx/regul;
			angular_vel.y = (float)gyy/regul;
			angular_vel.z = (float)gyz/regul;
			
			linear_acc.x = (float)acx/regul;
			linear_acc.y = (float)acy/regul;
			linear_acc.z = (float)acz/regul;
			
#ifdef debug_lvl1
			ROS_INFO(" READ : acc x : %f ; y : %f ; z : %f ;; vel x : %f ; y : %f ; z : %f", linear_acc.x, linear_acc.y, linear_acc.z, angular_vel.x, angular_vel.y, angular_vel.z);
			ROS_INFO(" CONV : acc x : %f ; y : %f ; z : %f ;; vel x : %f ; y : %f ; z : %f", linear_acc.x*sacc, linear_acc.y*sacc, linear_acc.z*sacc, angular_vel.x*sgyro, angular_vel.y*sgyro, angular_vel.z*sgyro);
#endif		


			//--------------------------
			//		SLAM
			//-------------------------
			Mat<float> rotAG2Gl(transpose( Euler2Rot( angleX,angleY,angleZ) ) );
			Mat<float> acc(3,1);
			acc.set( acx*sacc,1,1);
			acc.set( acy*sacc,2,1);
			acc.set( acz*sacc,3,1);
			
			acc = rotAG2Gl * acc;
			//instantaneous felt accelerations in the world frame.
			if(initBias)
			{
				initBias = false;
				*biasAcc = rotAG2Gl * ((sacc)*(*biasAcc));
				// bias in the world frame, which takes into account the gravity :
				// this initialization assume that the IMU is resting its frame being equal to the worl frame.
				
			}
			
			//acc -= (*biasAcc);
			//instantaneous accelerations in the world frame ( especially no longer any effect from gravity effect).
			
#ifdef EKF_USE	
	#ifndef EKF_ACCGYRO	
			//------------------------
			//		EKF
			//--------------------
			Mat<float> measure(6,1);
			measure.set( acc.get(1,1)*dt,1,1);
			measure.set( acc.get(2,1)*dt,2,1);
			measure.set( acc.get(3,1)*dt,3,1);
			measure.set( gyx*sgyro,4,1);
			measure.set( gyy*sgyro,5,1);
			measure.set( gyz*sgyro,6,1);
			
			instanceEEKF->measurement_Callback(measure);
			//instanceEEKF->setCommand(updatePose);
			instanceEEKF->state_Callback();
			EKFPose = instanceEEKF->getX();
			EKFPoseCovar = instanceEEKF->getSigma();
			
			ROS_INFO("EKF GLOBAL POSE : ");
			transpose(EKFPose).afficher();
			EKFPoseCovar.afficher();
			
			//---------------------------	
			*pose = extract(EKFPose, 1,1, 3,1);
			
	#else
			//------------------------
			//		EKF_ACCGYRO
			//--------------------
			Mat<float> input(3,1);
			Mat<float> measure(2,1);
			
			input.set( gyx*sgyro,1,1);
			input.set( gyy*sgyro,2,1);
			input.set( gyz*sgyro,3,1);
			
			float accRoll = atan2( acy*sacc, sqrt(acx*acx*sacc*sacc+acz*acz*sacc*sacc));
			float accPitch = atan2( acy*sacc*cos(accRoll*PI/180.0f), acz*sacc);
			measure.set( accRoll, 1,1);
			measure.set( accPitch, 2,1);
			
			
			ROS_INFO("MEASURE : ACC");
			transpose(measure).afficher();
			ROS_INFO("INPUT : GYRO");
			transpose(input).afficher();
			instanceEEKF->setCommand(input);
			instanceEEKF->measurement_Callback(measure);
			//instanceEEKF->setCommand(updatePose);
			instanceEEKF->state_Callback();
			EKFPose = instanceEEKF->getX();
			EKFPoseCovar = instanceEEKF->getSigma();
			
			ROS_INFO("EKF ANGLE BIAS : ");
			transpose(EKFPose).afficher();
			EKFPoseCovar.afficher();
			
			//---------------------------	
			*pose = extract(EKFPose, 1,1, 3,1);
	
	
	#endif
#else
			
			*pose += acc;	
	#ifdef debug_lvl1			
			ROS_INFO("POSE : x = %f ; y = %f ; z = %f", pose->get(1,1),	pose->get(2,1), pose->get(3,1));
	#endif				
			
#endif			
			//-------------------------
			//		FILTERING :
			//-------------------------
#ifdef COMPL1			
			//Complementary Filter :
			float lambda1 = 0.02;
			angleX = (1.0f-lambda1)*(angleX + gyx*sgyro*dt)+lambda1*acx*sacc;
			angleY = (1.0f-lambda1)*(angleY + gyy*sgyro*dt)+lambda1*acy*sacc;
			angleZ = (1.0f-lambda1)*(angleZ + gyz*sgyro*dt)+lambda1*acz*sacc;
#endif

#ifndef COMPL1
	#ifdef EKF_USE
		#ifndef EKF_ACCGYRO
			// Kalman Filter :
			angleX = EKFPose.get(4,1);			
			angleY = EKFPose.get(5,1);
			angleZ = EKFPose.get(6,1);
		#else
			// Kalman Filter ACCGYRO:
			angleX = EKFPose.get(1,1);			
			angleY = EKFPose.get(2,1);
			angleZ = EKFPose.get(3,1);
		#endif
	#endif
#endif

#ifndef COMPL1
	#ifndef EKF_USE
			//No Filter at all :
			angleX = angleX + gyx*sgyro*dt;
			angleY = angleY + gyy*sgyro*dt;
			angleZ = angleZ + gyz*sgyro*dt;
	#endif
#endif			

			Euler2Quaternion(&q, angleX,angleY,angleZ);
			ROS_INFO("ANGLE : roll : %f ; pitch : %f ; yaw : %f", angleX*3.1415f/180.0f,angleY*3.1415f/180.0f,angleZ*3.1415f/180.0f);
			
			
			//--------------------------
			//		PUBLISHER
			//-------------------------
			
			imu_data.header.stamp = ros::Time(0);
			imu_data.header.frame_id = "map";
			
			imu_data.orientation = q;
			imu_data.angular_velocity = angular_vel;
			imu_data.linear_acceleration = linear_acc;
			
			imu_pub.publish(imu_data);
			
			// SLAM PUBLISHER :
			std_msgs::Header head_path;
			head_path.frame_id = "map";
			
			geometry_msgs::Pose path_pose;
			path_pose.position.x = EKFPose.get(1,1);
			path_pose.position.y = EKFPose.get(2,1);
			path_pose.position.z = EKFPose.get(3,1);
			
			Quat q = Euler2Qt( EKFPose.get(4,1), EKFPose.get(5,1), EKFPose.get(6,1) );
			path_pose.orientation.x = q.x;
			path_pose.orientation.y = q.y;
			path_pose.orientation.z = q.z;
			path_pose.orientation.w = q.w;
			
			
			geometry_msgs::PoseStamped path_poseStamped;
			path_poseStamped.header.stamp = ros::Time(0);
			path_poseStamped.header.frame_id = "map";
			path_poseStamped.pose = path_pose;
			
			path.poses.push_back(path_poseStamped);
			path.header.stamp = ros::Time(0);
			path.header.frame_id = "map";
			
			//--------------------------------
			//--------------------------------
			
			
			path_pub.publish(path);
			pose_stamped_pub.publish(path_poseStamped);
				
			//-------------------------
			//-------------------------
			
			
#ifdef debug_lvl1			
			if(info_count >= 1000)
			{
				info_count = 0;
				ROS_INFO("IMU_COMport_Listener :: EXECUTION : %f Hz.", (float)(CLOCKS_PER_SEC/(clock()-timer)) );
				//ROS_INFO(" ACC x : %f ; y : %f ; z : %f ;; VEL x : %f ; y : %f ; z : %f", linear_acc.x, linear_acc.y, linear_acc.z, angular_vel.x, angular_vel.y, angular_vel.z);				
			}
			else
				info_count++;
#endif



#ifdef debug_lvl1
		//------------------------------------------
		//------------------------------------------
		//Ecriture  :
    	stringstream s;
		s << angleX << " | " << angleY << " | " << angleZ << " | " << acx  << " | " << acy << " | " << acz << " | " << gyx << " | " << gyy << " | " << gyz ;
		s << endl;
		fputs( s.str().c_str(), log);
#endif		
			r.sleep();
			
		}
		
	}		
	
	private :
	
	
	string filepath;
	FILE* log;
	
	int rate;	
	float regul;
	float dt;
	
	ros::NodeHandle nh;
	ros::Publisher imu_pub;
	ros::Publisher path_pub;
	ros::Publisher pose_stamped_pub;
	
	nav_msgs::Path path;

		
	int codeCOMPORT;
	int baudrate;
	Comport* instanceCOMPORT;
	
	float rangeAcc;
	float precisionAcc;
	float rangeGyro;
	float precisionGyro;
	
	string data;
	float sacc;
	float sgyro;
	
	int acx;
	int acy;
	int acz;
	int gyx;
	int gyy;
	int gyz;
	
	int sacx;
	int sacy;
	int sacz;
	int sgyx;
	int sgyy;
	int sgyz;
	
	float angleX;
	float angleY;
	float angleZ;
	
	Mat<float>* pose;
	Mat<float>* biasAcc;
	bool initBias;
	bool initBiasAccX;
	bool initBiasAccY;
	bool initBiasAccZ;
	
	geometry_msgs::Quaternion q;
	geometry_msgs::Vector3 angular_vel;
	geometry_msgs::Vector3 linear_acc;
	sensor_msgs::Imu imu_data;
	
	int info_count;
	
	//------------------------------------------
	//-----------------------------------------
	//EEKF :
	int nbrstate;
	int nbrcontrol;
	int nbrobs;
	float dtEKF;
	float stdnoise;
	float stdnoise_obs;
	bool ext;
	bool filteron;
	bool noise;

	Mat<float> EKFPose;  
	Mat<float> EKFPoseCovar;  
	EEKF<float>* instanceEEKF;
	Mat<float> Q;
	Mat<float> R;
	
	//------------------------------------------
	//-----------------------------------------
};


void Euler2Quaternion(geometry_msgs::Quaternion* q, float roll, float pitch, float yaw)
{
	Quat qq = Euler2Qt(roll,pitch,yaw);
	q->x = qq.x;
	q->y = qq.y;
	q->z = qq.z;
	q->w = qq.w;
}

int main(int argc, char* argv[])
{

	ros::init(argc,argv,"IMU_COMport_Listener");
	
	int idxCOM = 0;
	
	if(argc>1)
		idxCOM = atoi(argv[1]);
		
		
	int rate = 10;
	if(argc>2)
		rate = atoi(argv[2]);
	
	float regul = 1.0f;
		
	ROS_INFO("IMUListener:: rate : %d Hz ; regul_factor : %f, COMPORT : %d", rate, regul, idxCOM);
		
	IMUListener instanceIMUL(rate,regul,idxCOM);
	
	
	
	return 0;
}

