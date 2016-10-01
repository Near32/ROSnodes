#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <MAT/MatOpenCV.h>



mat::Mat<float> Mouse((float)0,2,1);
bool update = false;
int idx = 2;
void CallBackMouseFunc(int event, int x, int y, int flag, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        Mouse.set( (float)x, 1,1);
        Mouse.set( (float)y, 2,1);
        update = true;
        cout << "Position de la souris : " << endl;
        Mouse.afficher();
        idx = (idx+1)%3;
    }
}

cv::Mat dewarp(const cv::Mat& img, int R1, int R2, int cx, int cy)
{
	int Hs = abs(R2-R1);
	int Hd = 2*Hs;
	int Wd = 2*PI*Hs;
	
	int nbrchannel = img.channels();
	cv::Mat ret(Hd,Wd, CV_8UC3);
	//cv::Mat ret(Hd,Wd, img.type());
	
	
	for(int i=Hd;i--;)
	{
		uchar* pr = ret.ptr<uchar>(i);
		
		for(int j=Wd;j--;)
		{
			/*
			float r = ((float)j)/((float)Hd)*((float)(Hs))+(float)R1;
			float theta = ((float)i)/((float)Wd)*2*PI;
			float xs = (float)cx+r*cos(theta*180.0f/PI);
			float ys = (float)cy+r*sin(theta*180.0f/PI);
			*/
			
			float r = ((float)i)/((float)Hd)*((float)(Hs))+(float)R1;
			float theta = ((float)j)/((float)Wd)*2*PI;
			//float xs = (float)cx+r*cos(theta*180.0f/PI);
			float xs = (float)cx+r*cos(theta);
			//float ys = (float)cy+r*sin(theta*180.0f/PI);
			float ys = (float)cy+r*sin(theta);
			
			//std::cout << "r = " << r << " theta = " << theta << std::endl;
			
			/*
			for(int c=nbrchannel;c--;)
				ret.at<cv::Vec3b>(i,j)[c] = img.at<cv::Vec3b>(xs,ys)[c];
			*/
			
			
			const uchar* pi = img.ptr<uchar>((int)xs);
			pr[j*3] = (uchar) pi[(int)ys*3];//r.mat[x][y];
            pr[j*3+1] = (uchar) pi[(int)ys*3+1];//g.mat[x][y];
            pr[j*3+2] = (uchar) pi[(int)ys*3+2];//b.mat[x][y];
			
		}
	}
	 
	
	return ret;
}

mat::Mat<float> dewarp(const mat::Mat<float>& img, int R1, int R2, int cx, int cy)
{
	float Hs = abs(R2-R1);
	float Hd = 2*Hs;
	float Wd = 1*2*PI*Hs;
	
	int nbrchannel = img.getDepth();
	mat::Mat<float> ret((int)Hd,(int)Wd, nbrchannel);
	#ifdef calib
	mat::Mat<float> img1(img);
	#endif
	int offset = 1;
	
	for(int i=(int)Hd;i--;)
	{
		for(int j=(int)Wd;j--;)
		{
			float r = ((float)i)/((float)Hd)*((float)(Hs))+(float)R1;
			float theta = ((float)j)/((float)Wd)*2*PI;
			float xs = (float)cx+r*cos(theta);
			float ys = (float)cy+r*sin(theta);
			
			//std::cout << "r = " << r << " theta = " << theta << std::endl;
			
			for(int c=nbrchannel;c--;)
			{
				if(offset == 1)
				{
					ret(i+1,j+1,c+1) = img(xs,ys,c+1);
				}
				else
				{
					ret(i+1,j+1,c+1) = mat::sum(mat::sum( mat::extract(img, xs-offset,ys-offset,c+1,xs+offset,ys+offset,c+1) )).get(1,1)/pow(offset+1,2);
				}
			}
			
			#ifdef calib		
			img1(xs,ys,1) = 0;
			#endif
		}
	}
	
	#ifdef calib
	mat::afficherMat(std::string("test dewarp img"),&img1, (Mat<float>*)NULL,(Mat<float>*)NULL,true,1);
	#endif
	/*
	Mat<float> testr( extract( ret, 1,1,1, ret.getLine(), ret.getColumn(), 1) );
	Mat<float> testb( extract( ret, 1,1,2, ret.getLine(), ret.getColumn(), 2) );
	Mat<float> testg( extract( ret, 1,1,3, ret.getLine(), ret.getColumn(), 3) );
	afficherMat(std::string("test ret img"),&testr, &testg, &testb,true,1);
	*/
	
	return ret;
}

class DewarpNode
{
	public :
	
	DewarpNode(const cv::Mat& img_) : img(img_),continuer(true),resultImg(img_)
	{
		t = new std::thread(&DewarpNode::loop, this);
	}
	
	DewarpNode() : img(cv::Mat::zeros(640,480,CV_8UC3)),continuer(true),resultImg(cv::Mat::zeros(640,480,CV_8UC3))
	{
		t = new std::thread(&DewarpNode::loop, this);
	}
	
	~DewarpNode()
	{
		this->setContinuer(false);
		
		if(t->joinable())
		{
			t->join();
		}
		
		delete t;
	}
	
	void loop()
	{

		cv::namedWindow("MyVideoDEWARP-MAT",CV_WINDOW_AUTOSIZE);
	    cv::namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	    cv::setMouseCallback("MyVideo",CallBackMouseFunc);

		int xs = 299;
		int ys = 246;
		int R1 = 86;
		int R2 = 193;
		
		
		this->mutexRES.lock();
		while(this->continuer)
		{
			this->mutexRES.unlock();
			
			
			if(this->bankimg.size()>=1)
			{
				clock_t time = clock();
				this->mutexRES.lock();
				bankimg[0].copyTo(this->img);
				this->bankimg.clear();
				this->mutexRES.unlock();
				
				
				cv::circle(img,cv::Point(xs,ys), 2, cv::Scalar(100,0,0));
				cv::imshow("MyVideo", img); //show the frame in "MyVideo" window
				
				//------------------------------------------
				/*
				mat::Mat<float> frameMat(mat::cv2Matp<float>(this->img));
				mat::Mat<float> dewarpFrame( dewarp( frameMat, R1, R2, ys,xs) );
				*/
				
				this->mutexRES.lock();
				//this->resultImg = mat::Mat2cvp<float>( dewarpFrame);
				this->resultImg = dewarp(img, R1,R2,ys,xs);
				this->mutexRES.unlock();
				
				cv::imshow("MyVideoDEWARP-MAT", resultImg );
				
				//cv::imshow("MyVideoDEWARP", dewarp( frame, R1, R2, xs,ys) );
				
				//-------------------------------------------------

		
				//cv::imshow("SCAN DEWARP:", scanning(dewarpFrameCV));
				//cv::imshow("SCAN NOT DEWARP:", scanning(frame));
		

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
							xs = Mouse(1,1);
							ys = Mouse(2,1);
							std::cout << " UPDATE X,Y CENTER : " << xs << " / " << ys <<  std::endl;
						}
						break;
		
						case 1:
						{
							R1 = sqrt( pow(xs-Mouse(1,1),2)+pow(ys-Mouse(2,1),2) );
							std::cout << " UPDATE R1 : " << R1 << std::endl;
						}
						break;
		
						case 2:
						{
							R2 = sqrt( pow(xs-Mouse(1,1),2)+pow(ys-Mouse(2,1),2) );
							std::cout << " UPDATE R2 : " << R2 << std::endl;
						}
						break;
					}
				}
				
				
				//std::this_thread::sleep_for (std::chrono::milliseconds(20));
				//std::cout << " FPS = " << CLOCKS_PER_SEC/((float)clock()-time) << std::endl;
			}
			
			
			
			this->mutexRES.lock();
		}
		this->mutexRES.unlock();
	}
	
	void operator<<(const cv::Mat& nimg)
	{
		this->mutexRES.lock();
		this->bankimg.push_back( nimg);
		this->mutexRES.unlock();
	}
	
	void setContinuer(bool c)
	{
		this->mutexRES.lock();
		this->continuer = c;
		this->mutexRES.unlock();
	}
	
	cv::Mat operator>>(cv::Mat& img)
	{
		this->mutexRES.lock();
		this->resultImg.copyTo(img);
		this->mutexRES.unlock();
		
		return img;
	}
	
	private :
	
	std::thread* t;
	std::mutex mutexRES;
	std::vector<cv::Mat> bankimg;
	cv::Mat img;
	cv::Mat resultImg;
	bool continuer;
};

