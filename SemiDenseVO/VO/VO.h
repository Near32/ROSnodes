#ifndef VO_H
#define VO_H

#define OPENCV_USE
#include "../../MATv2/Mat.h"

//#include "../../StructureFromMotion/SFM.h"

#include "../../RAND/rand.h"
//#include "../../ONSC/ONSC.h"
//#include "../../ONSC/ONSCNewton.h"
#include "../../MVG/MVG.h"
#include "../../MVG/Frame.h"

//#include "../../EKF/v2/EKF.h"


#include "../LS/LS.h"
#include "../ThreadPool/ThreadPool.h"
#include "../RunningStats/RunningStats.h"


#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <mutex>
#include <vector>



#define DIVISION_EPS 1e-10f
#define UNZERO(val) (val < 0 ? (val > -1e-10 ? -1e-10 : val) : (val < 1e-10 ? 1e-10 : val))



// ============== stereo & gradient calculation ======================
#define MIN_DEPTH 0.05f // this is the minimal depth tested for stereo.

// particularely important for initial pixel.
#define MAX_EPL_LENGTH_CROP 10.0f // maximum length of epl to search.
#define MIN_EPL_LENGTH_CROP (3.0f) // minimum length of epl to search.

// this is the distance of the sample points used for the stereo descriptor.
#define GRADIENT_SAMPLE_DIST 1.0f

// pixel a point needs to be away from border... if too small: segfaults!
#define SAMPLE_POINT_TO_BORDER 2

// pixels with too big an error are definitely thrown out.
#define MAX_ERROR_STEREO (1300.0f) // maximal photometric error for stereo to be successful (sum over 5 squared intensity differences)
#define MIN_DISTANCE_ERROR_STEREO (1.5f) // minimal multiplicative difference to second-best match to not be considered ambiguous.

// defines how large the stereo-search region is. it is [mean] +/- [std.dev]*STEREO_EPL_VAR_FAC
#define STEREO_EPL_VAR_FAC 2.0f

#define cameraPixelNoise2 4*4

// ============== initial stereo pixel selection ======================
#define MIN_EPL_GRAD_SQUARED (1.0f*1.0f)
#define MIN_EPL_LENGTH_SQUARED (0.1f*0.1f)
#define MIN_EPL_ANGLE_SQUARED (0.15f*0.15f)

#define FAIL_VAR_INC_FAC 1.5f
#define MAX_VAR	500.0f
#define SUCC_VAR_INC_FAC 0.75f;

#define VALIDITY_COUNTER_INC 1
#define VALIDITY_COUNTER_DEC 1
#define MIN_BLACKLIST -5


#define RSfilepath "./RunningStats.txt"

//#define useHuber
#define HUBERKVAL 1.0e-2
#define LEVENBERGMARQUARD

//#define DEBUGGING

//#define debugWITHOUTTHREADPOOL

//TODO :
//1) debug the depth hypothesis. a.ground truth b. tweeting... 
//2) implement the regularization process with multithreading.
//3) integrate a Kalman Filter if the previous step are not enough.
//END TODO;


template<typename T>
Mat<T> debugPose(Frame<T>* f1, Frame<T>* f2, const Mat<T>& dm1, const Mat<T>& R, const Mat<T>& t, const Mat<T>& K, const Mat<T>& invK, T factor);
inline cv::Mat computeGradientXY(cv::Mat* im, cv::Mat* imgradX, cv::Mat* imgradY);

std::mutex rMutex,Imutex;

template<typename T>
class TrackFrame
{
	public :
	
	Frame<T> I;
	Frame<T> gradI;
	Frame<T> gradIX;
	Frame<T> gradIY;
	
	std::vector<Frame<T> > pyramidI;
	std::vector<Frame<T> > pyramidGradI;
	std::vector<Frame<T> > pyramidGradIX;
    std::vector<Frame<T> > pyramidGradIY;
    
    Mat<T> deltaKhi;
    Mat<T> deltaPose;
    
    
    Mat<T> K_otherToThis_R;
	Mat<T> K_otherToThis_t;
	Mat<T> otherToThis_t;
	Mat<T> K_thisToOther_t;
	Mat<T> thisToOther_R;
	Mat<T> otherToThis_R_row0;
	Mat<T> otherToThis_R_row1;
	Mat<T> otherToThis_R_row2;
	Mat<T> thisToOther_t;
    
    T initialTrackedResidual;
    //---------------------------------
    
    
    
    
    TrackFrame() : I(Mat<T>()), gradI(Mat<T>()), gradIX(Mat<T>()), gradIY(Mat<T>()), deltaKhi(Mat<T>((T)0,6,1)), deltaPose(Mat<T>((T)0,4,4)), K_otherToThis_R(Mat<T>(1,1)), K_otherToThis_t(Mat<T>(1,1)), otherToThis_t(Mat<T>(1,1)), K_thisToOther_t(Mat<T>(1,1)), thisToOther_R(Mat<T>(1,1)), otherToThis_R_row0(Mat<T>(1,1)), otherToThis_R_row1(Mat<T>(1,1)), otherToThis_R_row2(Mat<T>(1,1)), thisToOther_t(Mat<T>(1,1)), initialTrackedResidual(0)
    {
    
    }
    
    TrackFrame( const Frame<T>& I_, const Frame<T>& gradI_, const Frame<T>& gradIX_, const Frame<T>& gradIY_) : I(I_), gradI(gradI_), gradIX(gradIX_), gradIY(gradIY_), deltaKhi(Mat<T>((T)0,6,1)), deltaPose(Mat<T>((T)0,4,4)), K_otherToThis_R(Mat<T>(1,1)), K_otherToThis_t(Mat<T>(1,1)), otherToThis_t(Mat<T>(1,1)), K_thisToOther_t(Mat<T>(1,1)), thisToOther_R(Mat<T>(1,1)), otherToThis_R_row0(Mat<T>(1,1)), otherToThis_R_row1(Mat<T>(1,1)), otherToThis_R_row2(Mat<T>(1,1)), thisToOther_t(Mat<T>(1,1)), initialTrackedResidual(0)
    {
    
    }
    
    
    ~TrackFrame()
    {
    
    }
    
    void prepareForStereoWith( Mat<T> SE3otherToThis, Mat<T> K)
	{
		//Mat<T> thisToOther( invGJ(SE3otherToThis) );
#ifdef DEBUGGING		
		std::cout << "PREPARATION TRACKED FRAME : " << std::endl;
		SE3otherToThis.afficher();
#endif	
		
		Mat<T> RotherToThis( extract( &SE3otherToThis, 1,1,3,3) );
		
		otherToThis_t = extract( &SE3otherToThis, 1,4,3,4);
									
		
		K_otherToThis_R = K * RotherToThis;
		K_otherToThis_t = K * otherToThis_t;


		thisToOther_t = transpose(RotherToThis)*(((T)(-1.0f))*otherToThis_t);//((T)(-1))*otherToThis_t;//RotherToThis*(otherToThis_t);
		K_thisToOther_t = K * thisToOther_t;
		thisToOther_R = transpose(RotherToThis);
		
		otherToThis_R_row0 = extract( &thisToOther_R, 1,1, 1,3);
		otherToThis_R_row1 = extract( &thisToOther_R, 2,1, 2,3);
		otherToThis_R_row2 = extract( &thisToOther_R, 3,1, 3,3);

		//distSquared = ( transpose(otherToThis_t)*otherToThis_t).get(1,1);

	}
    
};


template<typename T>
class VO
{
	public :
	
	bool continuer;
	
	
	VO(int gradTreshold_, int nbrHypSearch_, int scalingFactor_, const Mat<T>& K_, int pyramidDepth_ = 3, float initMeanID_ = 1.0f, float initVarID_ = 0.9f) : nbrFrame(0), nbrKFrame(0), gradTreshold(gradTreshold_), scalingFactor(scalingFactor_), pyramidDepth(pyramidDepth_), initMeanID(initMeanID_), initVarID(initVarID_), requireKF(true), requireInitIDKF(true), idxFrameToTrack(0), K(K_), invK(invGJ(K)), offset(Mat<T>(3,1)), continuer(true)
	{
		Mat<T> tempK(K);
		pyramidK.insert( pyramidK.begin(), tempK);
		pyramidKinv.insert( pyramidKinv.begin(), invK);
		for(int k=pyramidDepth-1;k--;)
		{
			tempK.set( (T)tempK.get(1,3)/2, 1,3);
		    tempK.set( (T)tempK.get(2,3)/2, 2,3);
		    tempK.set( (T)tempK.get(1,1)/2, 1,1);
		    tempK.set( (T)tempK.get(2,2)/2, 2,2);		    
		    pyramidK.insert( pyramidK.begin(), tempK );
		    pyramidKinv.insert( pyramidKinv.begin(), invGJ(tempK) );
		}
		

		this->h = K.get(1,3)*2;//scalingFactor;
		this->w = K.get(2,3)*2;//scalingFactor;
		this->offset.set( this->h/2, 1,1);
		this->offset.set( this->w/2, 2,1);
		this->fx = K.get(1,1);//scalingFactor;
		this->fy = K.get(2,2);//scalingfactor;
		this->cx = this->h/2;
		this->cy = this->w/2;
		this->fxi = invK.get(1,1);//scalingFactor;
		this->fyi = invK.get(2,2);//scalingfactor;
		this->cxi = invK.get(1,3);
		this->cyi = invK.get(2,3);
		
		ls = new LS<T>();
		int nbrThreadSync = 2;
		threadPool = new ThreadPool<T,VO<T> >( nbrThreadSync );
		rs = new RunningStats<float>(RSfilepath);
	}
	
	~VO()
	{
		delete ls;
		delete threadPool;
		delete rs;
	}
	
	
	
	void operator<<( const cv::Mat& image)
	{
		cv::Mat frame;
		cv::resize(image, frame, cv::Size(0,0), 1.0f/scalingFactor, 1.0f/scalingFactor);
	
		rMutex.lock();
		nbrFrame++;
		rMutex.unlock();
		
		Frame<T> I = cv2Matp<T>( frame );
		cv::Mat gradX, gradY;
		Frame<T> gradI( cv2Matp<T>( computeGradientXY( &frame, &gradX, &gradY) ) );
		
		Imutex.lock();
		this->I.insert( this->I.end(), TrackFrame<T>( I, gradI, cv2Matp<T>( gradX), cv2Matp<T>( gradY) ) );
		Imutex.unlock();
		
		//let us compute the pyramids :
		computePyramidsI(this->pyramidDepth);
		
		while(nbrFrame > idxFrameToTrack+3);
		
	}
	
	
	
	
	
	
	
	void loop()
	{
		//initialization :
		std::cout << "INIT : " ;
		deltaKhi.insert( deltaKhi.end(), Mat<T>((T)0,6,1) );
		deltaPose.insert( deltaPose.end(), expM( deltaKhi[0] ) );
		std::cout << "OKAY." << std::endl;
		Mat<T> globalKhiPose((T)0,6,1);
		
		while( nbrFrame < 2);
		
		cv::namedWindow("Output KF");
		
		rMutex.lock();
		while(continuer)
		{
			rMutex.unlock();
			//Check for a KFrame : if we need one, then we will take the lastly updated Frame of I as reference.
			//At the beginning, the first Frame of I, is updated with the reference world frame without anything to do...
			rMutex.lock();
			if(requireKF)
			{
				requireKF = false;
				nbrKFrame++;
				rMutex.unlock();
				
				std::cout << "KF INIT : " ;
				
				KF.insert( KF.end(), I[idxFrameToTrack].I );
				gradKF.insert( gradKF.end(), I[idxFrameToTrack].gradI );
			
				//let us initialize or propagate the IDmap :
				rMutex.lock();
				if( requireInitIDKF)
				{	
					requireInitIDKF = false;
					rMutex.unlock();
					//let us initialize it :
					initIDKF();
				
				}
				else
				{
					rMutex.unlock();
					//let us propagate the current one in accordance with the current pose computed :
					propagateIDKF();
				}
			
				//let us compute the pyramids :
				computePyramidsKF(this->pyramidDepth);
				
				std::cout << "OKAY." << std::endl;
				
				rMutex.lock();			
			}
			rMutex.unlock();
			
			
			//check for a new frame :
			rMutex.lock();
			if( nbrFrame-2 > idxFrameToTrack)
			{	
				idxFrameToTrack++;
				rMutex.unlock();
				
				std::cout << "PROCESSING NEW FRAME : " << idxFrameToTrack << " : ";
				clock_t time = clock();
				
				/*With previous se3 as initialization : */
				//deltaKhi.insert( deltaKhi.end(), trackFrame( deltaKhi[idxFrameToTrack-1]) );
				/*init at 0 : */
				deltaKhi.insert( deltaKhi.end(), trackFrame( Mat<T>((T)0,6,1)) );
				
				deltaPose.insert( deltaPose.end(), expM(deltaKhi[idxFrameToTrack]) );
				I[idxFrameToTrack-1].deltaKhi = deltaKhi[idxFrameToTrack];
				I[idxFrameToTrack-1].deltaPose = deltaPose[idxFrameToTrack];
				
				
				//---------------------------------
				FILE* log = fopen("./logKHI.txt", "a");
				if(log == NULL)
				{
					cout << "ERROR : cannot open the file." << endl;
					exit(1);
				}
				else
		    		cout << "File opened." << endl;
				globalKhiPose += deltaKhi[idxFrameToTrack];
				stringstream s;
		       	for(int i=0;i<=globalKhiPose.getLine()-1;i++)	
		       	{
		       		s << globalKhiPose.get(i+1,1) << ",";
		       	}
		       	s << endl;
		       	fputs( s.str().c_str(), log);
		       	fclose(log);
		       	//------------------------------
		       	
		       	
				//DEPTH MAP UPDATE :
				observeDepth( idxFrameToTrack-1);
				
				
				std::cout << "VO : current delta Khi : " << std::endl;
				transpose(deltaKhi[idxFrameToTrack]).afficher();
				std::cout << "VO : current delta Pose : " << std::endl;
				deltaPose[idxFrameToTrack].afficher();
				//TODO : handle the global slam wit EKF...
				
				
				displayInverseCodedFrame(string("DEPTH MAP"), (Mat<float>*)&(I[idxFrameToTrack].I), &(IDKF[nbrKFrame-1]), true, scalingFactor/2);
				displayInverseCodedFrame(string("DEPTH MAP smoothed"), (Mat<float>*)&(I[idxFrameToTrack].I), &(IDKFsmoothed[nbrKFrame-1]), true, scalingFactor/2);
				
				Mat<float> ResidualAfterAlignement(w,h);
				Mat<T> rot( extract( deltaPose[idxFrameToTrack], 1,1, 3,3) );
				Mat<T> t( extract( deltaPose[idxFrameToTrack], 1,4, 3,4) );
				
				rMutex.lock();
				ResidualAfterAlignement = debugPose<T>( &(KF[nbrKFrame-1]), &(I[idxFrameToTrack].I), IDKF[nbrKFrame-1], rot, t, K, invK, scalingFactor/2);
				rMutex.unlock();
				
				
				std::cout << " DONE." << std::endl;
				std::cout << "PROCESSING TOOK : " << (float)(clock()-time)/CLOCKS_PER_SEC << " seconds." << std::endl;
				
				rMutex.lock();
			}
			rMutex.unlock();
			
			cv::imshow( "Output KF", Mat2cvp(KF[nbrKFrame-1],KF[nbrKFrame-1],KF[nbrKFrame-1]) );
			
			if(cv::waitKey(30)>=0)
	            continuer = false;
			
			
			rMutex.lock();
		}
		rMutex.unlock();
	}
	
	
	
	
	
	
	
	private :
	
	
	int gradTreshold;
	int nbrHypSearch;
	int scalingFactor;
	int pyramidDepth;
	
	T fx;
	T fy;
	T cx;
	T cy;
	T fxi;
	T fyi;
	T cxi;
	T cyi;
	Mat<T> K;
	Mat<T> invK;
    std::vector<Mat<T> > pyramidK;
    std::vector<Mat<T> > pyramidKinv;
    	
	float initMeanID;
	float initVarID;
	
	int w;
	int h;
	Mat<T> offset;
	
	int idxFrameToTrack;
	int nbrFrame;
	int nbrKFrame;
	std::vector<TrackFrame<T> > I;
	
	bool requireKF;
	bool requireInitIDKF;
	std::vector<Frame<T> > KF;
	std::vector<Frame<T> > gradKF;
	
	std::vector<Mat<T> > IDKF;
	std::vector<Mat<T> > IDKFsmoothed;
	std::vector<Mat<T> > varIDKF;
	std::vector<Mat<T> > varIDKFsmoothed;
	
	std::vector<Mat<T> > validityIDKF;
	std::vector<Mat<T> > blacklistedIDKF;
	    
	std::vector<Frame<T> > pyramidKF;    
    std::vector<Mat<T> > pyramidIDKF;
    std::vector<Mat<T> > pyramidVarIDKF;
    
    std::vector<Mat<T> > deltaPose;
    Mat<T> globalPose;
    std::vector<Mat<T> > deltaKhi;
    Mat<T> globalKhi;
    
    LS<T>* ls;
    ThreadPool<T,VO<T> >* threadPool;
    RunningStats<float>* rs;
    
    
    protected :
    
    void computePyramidsI(int nbrL)
    {
#ifdef debugOptim
		clock_t time = clock();
#endif
        Mat<T> kernel((T)0.25f,2,2);
        
        Imutex.lock();
        I[nbrFrame-1].pyramidI.clear();
        I[nbrFrame-1].pyramidI.insert( I[nbrFrame-1].pyramidI.end(), I[nbrFrame-1].I );
        
		I[nbrFrame-1].pyramidGradIX.clear();
		I[nbrFrame-1].pyramidGradIX.insert( I[nbrFrame-1].pyramidGradIX.end(), I[nbrFrame-1].gradIX );
		I[nbrFrame-1].pyramidGradIY.clear();
		I[nbrFrame-1].pyramidGradIY.insert( I[nbrFrame-1].pyramidGradIY.end(), I[nbrFrame-1].gradIY );
		Imutex.unlock();
        
#ifdef debugOptim	
    	cout << "LA CREATION DE LA PYRAMIDE : INITIALISATION MALLOC :: " << ((float)(clock()-time)/CLOCKS_PER_SEC) << " secondes." << endl;
#endif 
#ifdef debugOptim        
        clock_t cumulPool = 0;
#endif
		
		for(int i=nbrL-1;i--;)
		{
#ifdef debugOptim        
			clock_t incPool = clock();
#endif 		
			Imutex.lock();
		    I[nbrFrame-1].pyramidI.insert( I[nbrFrame-1].pyramidI.begin(), Frame<T>( 1.0f*subsampled( &(I[nbrFrame-1].pyramidI[0]), &kernel), I[nbrFrame-1].I.getChannel() ) );
		    Imutex.unlock();
#ifdef debugOptim        
        	cumulPool += clock()-incPool;
#endif 		    
			Imutex.lock();
		    I[nbrFrame-1].pyramidGradIX.insert( I[nbrFrame-1].pyramidGradIX.begin(), Frame<T>( 1.0f*subsampled( &(I[nbrFrame-1].pyramidGradIX[0]), &kernel), I[nbrFrame-1].gradIX.getChannel() ) );
		    Imutex.unlock();
		    Imutex.lock();
		    I[nbrFrame-1].pyramidGradIY.insert( I[nbrFrame-1].pyramidGradIY.begin(), Frame<T>( 1.0f*subsampled( &(I[nbrFrame-1].pyramidGradIY[0]), &kernel), I[nbrFrame-1].gradIY.getChannel() ) );
		    Imutex.unlock();
		    
		}	
	
#ifdef debugOptim	
    	cout << "LA CREATION DE LA PYRAMIDE : CUMUL POOL :: " << ((float)(cumulPool)/CLOCKS_PER_SEC) << " secondes." << endl;
#endif 

    }
#define debugOptim   
    void computePyramidsKF(int nbrL)
    {
#ifdef debugOptim
		clock_t time = clock();
#endif
        Mat<T> kernel((T)0.25f,2,2);
        pyramidKF.clear();
        pyramidKF.insert( pyramidKF.end(), KF[nbrKFrame-1] );
        
		pyramidIDKF.clear();
        pyramidIDKF.insert( pyramidIDKF.end(), IDKF[nbrKFrame-1] );
        
        pyramidVarIDKF.clear();
        pyramidVarIDKF.insert( pyramidVarIDKF.end(), varIDKF[nbrKFrame-1] );
        //Mat<T> tempaff( ((T)200)*varIDKF[0]);
        //varIDKF[0].afficher();
        //afficherMat( &tempaff, (Mat<T>*)NULL,(Mat<T>*)NULL, false, 4);
        
        //afficherMat( string("debug KF,gradKF, varIDKF"), (Mat<T>*)&(KF[nbrKFrame-1]), (Mat<T>*)&(IDKF[nbrKFrame-1]), (Mat<T>*)NULL,false, 2);
        
#ifdef debugOptim	
    	cout << "LA CREATION DE LA PYRAMIDE : INITIALISATION MALLOC :: " << ((float)(clock()-time)/CLOCKS_PER_SEC) << " secondes." << endl;
#endif 
#ifdef debugOptim        
        clock_t cumulPool = 0;
#endif
		
		for(int i=nbrL-1;i--;)
		{
#ifdef debugOptim        
			clock_t incPool = clock();
#endif 		
		    pyramidKF.insert( pyramidKF.begin(), Frame<T>( subsampled( &pyramidKF[0], &kernel), this->KF[0].getChannel() ) );
#ifdef debugOptim        
        	cumulPool += clock()-incPool;
#endif 		    
		    pyramidVarIDKF.insert( pyramidVarIDKF.begin(), pooling( &pyramidVarIDKF[0], &kernel,1) );
		    //carefull : pyramidSubsampling is using the variance at the level of the previous computation.. It has to be the same sizes as the current pyramidIDK[0] :
		    pyramidIDKF.insert( pyramidIDKF.begin(), pyramidSubsampling( pyramidIDKF[0], pyramidVarIDKF[1]) );
		    //pyramidIDKF.insert( pyramidIDKF.begin(), subsampled( &pyramidIDKF[0], &kernel) );
			
			/*
		    Mat<T> temp( ((T)200.0f)*pyramidIDKF[0]);
		    Mat<T> tempvar( ((T)200)*pyramidVarIDKF[0]);
		    afficherMat(&pyramidKF[1],&pyramidKF[0], &temp,false,4.0);
		    */
		}	
	
#ifdef debugOptim	
    	cout << "LA CREATION DE LA PYRAMIDE : CUMUL POOL :: " << ((float)(cumulPool)/CLOCKS_PER_SEC) << " secondes." << endl;
#endif 

    }
    
    
    
    
    Mat<T> pyramidSubsampling(const Mat<T>& idMap, const Mat<T>& varIdMap)
	{
		Mat<T> ridMap((T)numeric_limits<T>::epsilon(),idMap.getLine()/2,idMap.getColumn()/2);
	
		for(int i=ridMap.getLine();i--;)
		{	
			for(int j=ridMap.getColumn();j--;)
			{
				//T varMean = (T)0;
				T coeffMean = (T)0;
				T id = (T)0;
				int usedValue = 0;
			
				for(int ii=2;ii--;)
				{
					for(int jj=2;jj--;)
					{
						T currentId = idMap.get(2*i+ii+1,2*j+jj+1); 
					
						if(currentId != (T)0)
						{
							//if there is an actual value, then we incorporate
							usedValue++;
						
							T currentVar = varIdMap.get(2*i+ii+1,2*j+jj+1);
							if(currentVar == (T)0)
								currentVar = numeric_limits<T>::epsilon();
						
							coeffMean += (T)(1.0f/currentVar);
						
							id += (1.0f/currentVar)*currentId;
							//varMean += currentVar;
						}
						
					}
				}
			
				if(usedValue)
				{
					ridMap.set( id/coeffMean, i+1,j+1);
					//if there was an actual pixel used here, then we incorporate the value, else we set 0 :
				}
				else
				{
					ridMap.set( (T)0, i+1,j+1);
				}
			}
		}
	
		return ridMap;
	}
    
    Mat<T> initializeInvDM(Frame<T>* frame, T gradIntTreshold, T variance, T mean)
	{
		T meanid = 0.0f;
		int nbrpoint = 0;
		
		if(gradIntTreshold >= 255)
			gradIntTreshold = 255;
		else if(gradIntTreshold < 0)
			gradIntTreshold = 0;
			 
		NormalRand gen(0,variance,(long)170);
		Mat<T> idm((T)0.0, frame->getLine(), frame->getColumn());
		Mat<T> x((T)0.0,2,1);

		for(int i=frame->getLine();i--;)
		{
		    x.set(i+1,1,1);

		    for(int j=frame->getColumn();j--;)
		    {
		        x.set(j+1,2,1);

		        //inverse depth value is assigned only if the gradient is sufficient.
		        if( fabs_(frame->get(x).get(1,1)) >= (T)gradIntTreshold)
		        {
		        	T temp = (T)(mean+gen.dev());
		            idm.set( temp, i+1,j+1);
		            //initialized inverse depth between -0.9 and 1.1.
		            
		            meanid += temp;
		            nbrpoint++;
		        }

		    }
		}
		
		meanid /= nbrpoint;
		//idm *= (T)(1.0f/meanid);
		
		//TODO : regularization...

		return idm;
	}
	
	
    
    void initIDKF()
    {
    	IDKF.insert( IDKF.end(), initializeInvDM(&gradKF[nbrKFrame-1], this->gradTreshold, this->initVarID, this->initMeanID) );
    	IDKFsmoothed.insert( IDKFsmoothed.end(), IDKF[nbrKFrame-1] );
		varIDKF.insert( varIDKF.end(), Mat<T>((T)numeric_limits<T>::epsilon(), this->IDKF[nbrKFrame-1].getLine(), this->IDKF[nbrKFrame-1].getColumn()) );
		
		
		this->h = this->IDKF[nbrKFrame-1].getLine();
		this->w = this->IDKF[nbrKFrame-1].getColumn();
		 
		for(int i=h;i--;)
		{
			for(int j=w;j--;)
			{
				if(IDKF[nbrKFrame-1].get(i+1,j+1) != (float)0)
				{
					varIDKF[nbrKFrame-1].set( this->initVarID, i+1,j+1);
				}
			}
		}
		
		varIDKFsmoothed.insert( varIDKFsmoothed.end(), varIDKF[nbrKFrame-1] );
		
		validityIDKF.insert( validityIDKF.end(), Mat<T>((T)1,h,w) );
		blacklistedIDKF.insert( blacklistedIDKF.end(), Mat<T>((T)0,h,w) );
    }
    
    
    void propagateIDKF()
    {
    	//careful : the current IDKF is now indexed by nbrKFrame-2 because this index has been updated...
    	IDKF.insert( IDKF.end(), propagateDepthMap( this->gradKF[nbrKFrame-2], this->deltaPose[idxFrameToTrack], this->IDKF[nbrKFrame-2] ) );
    	IDKFsmoothed.insert( IDKFsmoothed.end(), IDKF[nbrKFrame-1] );
    	
    	//TODO : propagate the varianceIDKF and varIDKsmoothed...
    	//TODO : propagate the validity and blacklisted IDKF
    }
    
    
    Mat<T> propagateDepthMap(const Frame<float>& gradframe, const Mat<T>& deltaPose, const Mat<T>& invdm)
	{
	
		Mat<T> PI0((T)0,3,4);
		for(int i=3;i--;)	PI0.set((T)1,i+1,i+1);	
	
		Mat<T> rinvdm((T)0,invdm.getLine(),invdm.getColumn());
		Mat<T> x((T)1,3,1),xoffset(x);
		Mat<T> rX(4,1),rx(3,1),rxoffset(rx);
	 
		NormalRand gen(0,this->initVarID,(long)10);


		for(int i=rinvdm.getLine();i--;)
		{
			x.set(i+1,1,1);
	
			for(int j=rinvdm.getColumn();j--;)
			{
				x.set(j+1,2,1);
		
				if(invdm.get(i+1,j+1) != (T)0)
				{				
					xoffset = x+offset;
					T depth = ((T)1.0/invdm.get(i+1,j+1));
				
					//Rigid body motion :
					rX = operatorC( depth*(invK*xoffset), Mat<T>((T)1,1,1) );
					rX = deltaPose*rX;			
											
					/*set the depth of the correct point in the new KF :*/
					rx = K*(PI0*rX);
					//set the correct depth to each point of the new KF according the frame variation ://
			
					homogeneousNormalization(&rx);
					rxoffset = rx-offset;
			
					float new_id = 1.0f/(depth-deltaPose.get(3,4));
				
					if(!isnan(new_id))
					{
						if(rinvdm.get(rxoffset.get(1,1),rxoffset.get(2,1)) == (T)0)
							rinvdm.set( new_id, rxoffset.get(1,1), rxoffset.get(2,1));
						else
						{
							// there was a previous assumption...						
							//occlusion case :
							if(new_id > rinvdm.get(rxoffset.get(1,1),rxoffset.get(2,1)) )
								rinvdm.set( new_id, rxoffset.get(1,1), rxoffset.get(2,1));
							//else we keep that previous assumption.
						}
					}
					else
					{
						//random initialization :
						rinvdm.set( (float)this->initMeanID+(float)gen.dev(), i+1,j+1);
					}
				}
		
				else if( fabs_(gradframe.get(i+1,j+1)) >= (float)gradTreshold)
				{
					//inverse depth value is assigned only if the gradient is sufficient.
			
					//TODO : discussion as to know whether the point is a new point or if it is already pictured ine the last frame and if the point in the last frame and this one are really backprojecting into the same point or if it is an appearing point due to occlusion...
					// Meanwhile, we randomized those and put our faith in the ones that do not have such problem...
					rinvdm.set( (float)this->initMeanID+(float)gen.dev(), i+1,j+1);
					//initialized inverse depth between -0.9 and 1.1.
				}
			
				if( isnan(rinvdm.get(i+1,j+1)))
				{
					rinvdm.set( 1.0f+(float)this->initMeanID+(float)gen.dev(), i+1,j+1);
				}
		
							
			}
		}
	
		return rinvdm;
	}
	
	
	
	
	
    
    
    
    
    
    
    Mat<T> trackFrame(const Mat<T>& lastSol)
    {
    	clock_t timer = clock();
    	T maxvaluedepth = (T)(1.0f/numeric_limits<T>::epsilon());
    	
    	
		T lambdaLM = (T)1e-7;
		T Ldown = (T)1.0f/9;
		T Lup = (T)11;
		
		T error = 0;
		T lastError = 1e20f;
		int countErrorRising = 0;
		
		Mat<T> sol(lastSol);
		
		int level = 0;
    	int maxlevel = pyramidKF.size()-1;
    	int itsperlowlevel = 5;
    	int itsperlevel = 3;
    	
        for(;level<= (maxlevel>3?3:maxlevel) ;level++)
    	{
    		clock_t intimer = clock();
			lambdaLM = (T)10;
    		
    		Mat<T> offset(3,1);
    		offset.set( this->pyramidKF[level].getLine()/2, 1,1);
			offset.set( this->pyramidKF[level].getColumn()/2, 2,1);
			offset.set( (T)0, 3,1);
			
			T fx = this->pyramidK[level].get(1,1);
	     	T fy = this->pyramidK[level].get(2,2);
	     	T cx = this->pyramidK[level].get(1,3);
	     	T cy = this->pyramidK[level].get(2,3);
			
			//iteration per level :	
			lastError = 1e20f;
    	    for(int i=1;i<=(level<=maxlevel/2? itsperlowlevel : itsperlevel);i++)
    	    {
		    	int nbrLA = computePyramidResidualA(sol, level);
		    	
		    	
		    	//Least-Square Resolution for this level :    		    
					    
				
#ifdef LEVENBERGMARQUARD				
				Mat<T> A( ls->A);
				for(int iLM=6;iLM--;)
				{
					float val = A.get(iLM+1,iLM+1);
					A.set( val+val*lambdaLM, iLM+1,iLM+1);
				}
				ls->A = A;
				ls->finish();
#endif//LEVENBERGMARQUARD
				
				Mat<T> x(ls->A.getLine(),1);
				ls->solve(x);
				
				if( isnanM(x))
				{
					x = Mat<T>((T)0,6,1);
				}

				//INCREMENTAL UPDATE 
				//compute error :
				
				error = (T)0;
				Mat<T> xi((T)1,3,1);
				Mat<T> xioffset(3,1);
				Mat<T> rxioffset(4,1);
				Mat<T> tempsol(sol+x);
				Mat<T> warpedxi(3,1);
				Mat<T> khihat(expM(tempsol));
		    	Mat<T> rot(extract(khihat,1,1,3,3) );
		    	Mat<T> t(extract(khihat,1,4,3,4) );
    		    		    
				for(int ii= this->pyramidIDKF[level].getLine();ii--;)
				{
					xi.set((T)ii+1, 1,1);

					for(int jj= this->pyramidIDKF[level].getColumn();jj--;)
					{
						xi.set((T)jj+1, 2,1);					

						if( (this->pyramidIDKF[level]).get( ii+1, jj+1) != (T)0)
						{										

							//xioffset = xi+offset;
							xioffset = xi;
							//xioffset = retroproj(&xioffset);
                    		T zx = (T)((float)1.0/(float)(this->pyramidIDKF[level].get( ii+1, jj+1 )) );
							rxioffset = zx*((this->pyramidKinv[level])*(xioffset)) ;
							
							//Forward Additive Algorithm :
							warpedxi = rot * rxioffset+t;

							//----- RESIDUAL -----
							//Forward Additive Algorithm :
							Mat<T> projection( this->pyramidK[level]*warpedxi );
							homogeneousNormalization(&projection);
							//projection -= offset;

							Imutex.lock();
							//T residual = fabs_( (this->pyramidKF[level]).mat[ii][jj] - (this->I[idxFrameToTrack].pyramidI[level]).get( projection ).get(1,1) );
							T residual = fabs_( (this->pyramidKF[level]).mat[ii][jj] - (this->I[idxFrameToTrack].pyramidI[level]).getInterpolatedElement( projection ) );
							Imutex.unlock();
							
							//T val = (T)1.0f/this->pyramidVarIDKF[level].get( ii+1, jj+1);                   
							T val = (T)1.0f/((T)this->pyramidVarIDKF[level].get(ii+1,jj+1));
							
							if( !(std::isfinite(val)) || isnan(val) || fabs_(val) > 1e4f || val == (T)0 )
							{
								std::cout << "val = " << val << std::endl;
								val = 0.1f;
							}
							
							if( !(std::isfinite(residual)) || isnan(residual) || fabs_(residual) > 1e4f )
							{
								//std::cout << "residual = " << residual << std::endl;
								residual = 0.1f;
							}
							
							T derr = residual*residual*val;
							
							//rs->tadd(-33+level, derr);
							
							error = error + derr;
							//std::cout << "residual = " << residual << " ; val = " << val << std::endl;
							//error += residual*residual;
						}		
					}
				}
			
				cout << " LEVEL = " << level << " i:" << i << " ; error = " << error << " ; scaled error = " << error/ (this->pyramidIDKF[level].getColumn())*(this->pyramidIDKF[level].getLine()) << " et lasterror = " << lastError << endl; 
				cout << " LS ERROR = " << ls->error << std::endl;
				//error /= (this->pyramidIDKF[level].getColumn())*(this->pyramidIDKF[level].getLine());
			
				if(error < lastError)
				{
					sol += x;
					
					lastError = error;
					countErrorRising = 0;
					
					/*
					if(i>0)
						i--;
					*/
					
					lambdaLM = lambdaLM*Lup;
					if(lambdaLM < (T)1e-7)
						lambdaLM = (T)1e-7;
						
					//cout << "LAMBDA LM rising = " << lambdaLM << endl;
					
				}
				else
				{
					lambdaLM = lambdaLM*Ldown;
					if(lambdaLM > (T)1e7)
						lambdaLM = (T)1e7;
						
					//cout << "LAMBDA LM falling = " << lambdaLM << endl;
					
					//counter which reveals when the resolution is no longer going to evolve :
					countErrorRising++;
				}
				
				/*
				if(countErrorRising >= 3)
				{
					//if the error has been kept from falling for the last level then we drop the above level :
					if(level != maxlevel)
					{
						i = itsperlowlevel;
					}
					i = itsperlowlevel;
					countErrorRising = 0;
					//we only allow for one last iteration at the maximum level directly 
				}*/

				// problem solved for this level.

			}
			
			std::cout << "LEVEL tracking : " << (float)(clock()-intimer)/CLOCKS_PER_SEC << " seconds." << std::endl;

        }
	
		I[idxFrameToTrack].initialTrackedResidual = ls->error/255;
		rs->tadd(10, ls->error/255);
		
		std::cout << "TRACKING FRAME : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " seconds." << std::endl;
		
		return sol;
    }
    
    
    
    
    
    
    
    
    int computePyramidResidualA(const Mat<T>& sol, size_t level)
    {
    	
		clock_t timer = clock();

    	/*initializations :*/    	
    	float error = 0;
     	Mat<T> xi(3,1);
     	xi.set((T)1, 3,1);
     	Mat<T> xioffset(3,1);
     	Mat<T> rxioffset(3,1);
     	Mat<T> warpedxi(3,1);
     	Mat<T> rxi(4,1);
     	
     	Mat<T> khi_hat( expM(sol));
     	Mat<T> rot( extract(&khi_hat, 1,1,3,3) );
     	Mat<T> t( extract(&khi_hat, 1,4,3,4) );
     	
     	T fx = this->pyramidK[level].get(1,1);
     	T fy = this->pyramidK[level].get(2,2);
     	T cx = this->pyramidK[level].get(1,3);
     	T cy = this->pyramidK[level].get(2,3);
     	T ifx = (this->pyramidKinv[level]).get(1,1);
     	T ify = (this->pyramidKinv[level]).get(2,2);
     	T icx = (this->pyramidKinv[level]).get(1,3);
     	T icy = (this->pyramidKinv[level]).get(2,3);
     	
     	
     	ls->initialize();
     	Mat<T> J(6,1);	
     	
     	int count = 0;
     	int countVAR2SMALL = 0;
     	
     	int step = 1;

        for(int i= this->pyramidIDKF[level].getLine();i--;)
     	{
            xi.set((T)i+1, 1,1);
     		
            for(int j= this->pyramidIDKF[level].getColumn();j--;)
     	    {               
	     		
                if( (this->pyramidIDKF[level]).get( i+1, j+1) != (T)0)
     			{

         		    xi.set((T)j+1, 2,1);
         		    count++;
         		    
     		    
         	    	//xioffset = xi+offset;    
         	    	xioffset = xi;
                    //rxi = retroproj(&xioffset);
                    T zx = (T)((float)1.0/(float)(this->pyramidIDKF[level].get( i+1, j+1 )) );
	        
					rxioffset = zx*((this->pyramidKinv[level])*(xioffset)) ;
				
                    //Forward Additive Algorithm :
                    warpedxi = rot * rxioffset+t;
                    
                    T wxi_x = warpedxi.mat[0][0];
                    T wxi_y = warpedxi.mat[1][0];
                    T wxi_z = warpedxi.mat[2][0];
                    
                    Imutex.lock();
                    T pI2gradX = this->I[idxFrameToTrack].pyramidGradIX[level].get(i+1,j+1);
                    T pI2gradY = this->I[idxFrameToTrack].pyramidGradIY[level].get(i+1,j+1);
                    Imutex.unlock();
                    
                    T val = (T)1.0f/((T)this->pyramidVarIDKF[level].get(i+1,j+1));
					
				    /*----- RESIDUAL -----*/
             	    //Forward Additive Algorithm :
             	    Mat<T> projection( this->pyramidK[level]*warpedxi );
					homogeneousNormalization(&projection);
					//projection -= offset;
					Imutex.lock();
		    		//T residual = fabs_( (this->pyramidKF[level]).mat[i][j] - (this->I[idxFrameToTrack].pyramidI[level]).get(projection).get(1,1) );
		    		T residual = fabs_( (this->pyramidKF[level]).mat[i][j] - (this->I[idxFrameToTrack].pyramidI[level]).getInterpolatedElement( projection ) );
		    		Imutex.unlock();
		    
		    		
		    		if( !(std::isfinite(val)) || isnan(val) || fabs_(val) > 1e4f || val == (T)0 )
					{
						std::cout << " PYRAMID val = " << val << std::endl;
						val = 0.1f;
					}
					
					if( !(std::isfinite(residual)) || isnan(residual) || fabs_(residual) > 1e4f )
					{
						//std::cout << "residual = " << residual << std::endl;
						residual = 0.1f;
					}
					
					
				#ifdef	useHuber
					val *= residual;//this->pyramidvar_depthMapI1[level].get(i+1,j+1);
				
					//HUBER :
				
					T huberK = HUBERKVAL;
					//T huberK = 10.345;
				
					if(fabs_(val) <= (T)huberK)
					{
						countVAR2SMALL++;
					val = (T)val/(2*huberK);//*this->pyramidvar_depthMapI1[level].get(i+1,j+1);
					}
					else
						val = fabs_(val)- huberK/2;	    
					
				
				
					//HUBER (kerl):
					/*
					T huberK = 1.345e-2;
				
					if(fabs_(val) < (T)huberK)
					{
						countVAR2SMALL++;
					val = (T)1;
					}
					else
						val = huberK/fabs_(val);
					*/
				
					//TUKEY :
					/*
					T tukeyB = 4.685;
				
					if( fabs_(val) < tukeyB)
					{
						countVAR2SMALL++;
						val = pow( (1- pow(val/tukeyB, (float)2)), (float)2);
					}
					else
						val = 0;
				     	    */
				     	    
				    error += val;
				#else
				
					error += val*residual*residual;
				#endif
             	        	    
             	    J.mat[0][0] =   ( pI2gradX * fx * ((T)1.0f/wxi_z) ); 
             	    J.mat[1][0] =   ( pI2gradY * fy * ((T)1.0f/wxi_z) ); 
             	    J.mat[2][0] =   ( -((T)1.0f/(wxi_z*wxi_z)) * ( pI2gradX * wxi_x * fx + pI2gradY * wxi_y * fy ) );
             	     
             	    J.mat[3][0] =   ( (-pI2gradY * fy)*(1 + wxi_y*wxi_y/(wxi_z*wxi_z) ) + ( (-pI2gradX *fx)*( wxi_y*wxi_x/(wxi_z*wxi_z) ) ) );
             	     
             	    J.mat[4][0] =   ( (pI2gradX * fx)*(1 + wxi_x*wxi_x/(wxi_z*wxi_z) ) + ( (pI2gradY *fy)*( wxi_y*wxi_x/(wxi_z*wxi_z) ) ) );
             	     
             	    J.mat[5][0] =   ( (T)1.0f/wxi_z * ( -pI2gradX * wxi_y * fx + pI2gradY * wxi_x * fy ) );
             	    
             	    //std::cout << "grad : ( " << pI2gradX << " , " <<  pI2gradY << " ) ; wxi_Z : " << wxi_z << std::endl;
             	    //std::cout << "wxi : ( " << wxi_x << " , " <<  wxi_y << " ) ; wxi_Z : " << wxi_z << std::endl;
             	    //std::cout << "proj : ( " << projection.get(1,1) << " , " <<  projection.get(2,1) << " , " << projection.get(3,1) << " )" << std::endl;
             	    //transpose(J).afficher();
             	    //std::cout << "K : ( " << fx << " , " <<  fy << " ) ; wxi_Z : " << wxi_z << std::endl;
             	    //std::cout << "res val : ( " << residual << " , " <<  val << " ) ; wxi_Z : " << wxi_z << std::endl;
             	    
             	    //-----------------
             	    
             	    if(isnanM(J) || isnan(residual*val))
             	    {
             	    	throw;
             	    }
             	    
             	    ls->update( J, residual, val);
         		
         		}
     	    }
     	}
	
		return count;
                	    
    }
    
    
    
    bool makeAndCheckEPL(const int x, const int y, const int idxTrackFrame, float* pepx, float* pepy)
	{
		Imutex.lock();
		Mat<T> deltaso3( I[idxTrackFrame].deltaKhi );
		Imutex.unlock();
		/* make epl */
		// calculate the plane spanned by the two camera centers and the point (x,y,1)
		// intersect it with the keyframe's image plane (at depth=1)
		float epx = - fx * deltaso3.get(1,1) + deltaso3.get(3,1)*(x - cx);
		float epy = - fy * deltaso3.get(2,1) + deltaso3.get(3,1)*(y - cy);

		if(isnanf(epx+epy))
		{
#ifdef DEBUGGING				
			std::cout << "EPL IS NAN !" << std::endl;
#endif			
			return false;
		}

		
		/* check epl length */
		float eplLengthSquared = epx*epx+epy*epy;
		if(eplLengthSquared < MIN_EPL_LENGTH_SQUARED)
		{
			//std::cout << "EPL IS TOO SMALL..." << std::endl;
			rs->add(-11);
			rs->mean(-111, eplLengthSquared);
			//return false;
		}

		/* check epl-grad magnitude */
		rMutex.lock();
		float gx = KF[nbrKFrame-1].get(x+1,y) - KF[nbrKFrame-1].get(x-1,y);
		float gy = KF[nbrKFrame-1].get(x,y+1) - KF[nbrKFrame-1].get(x,y-1);
		rMutex.unlock();
		float eplGradSquared = gx * epx + gy * epy;
		eplGradSquared = eplGradSquared*eplGradSquared / eplLengthSquared;	// square and norm with epl-length
		
		if(eplGradSquared < MIN_EPL_GRAD_SQUARED)
		{
			//std::cout << "EPL IS ORTHOGONAL TO GRAD..." << std::endl;
			rs->add(-12);
			return false;
		}


		/* check epl-grad angle */
		if(eplGradSquared / (gx*gx+gy*gy) < MIN_EPL_ANGLE_SQUARED)
		{
			//std::cout << "EPL COMPOSANT ALONG GRAD IS TOO SMALL AFTER NORMALIZATION...! : epl angle squared = " << eplGradSquared / (gx*gx+gy*gy) << " / " << MIN_EPL_ANGLE_SQUARED << std::endl;
			rs->add(-13);
			return false;
		}


		/* return "normalized" epl */
		float fac = 1.0f / sqrt(eplLengthSquared);
		*pepx = epx * fac;
		*pepy = epy * fac;

		return true;
	}
	
	
	
	inline float doLineStereo( 	const float u, const float v, 
								const float epxn, const float epyn,
								const float min_idepth, const float prior_idepth, float max_idepth,
								TrackFrame<T>* referenceFrame,
								T &result_idepth, T &result_var, T &result_eplLength )
	{
		Frame<T>* activeKF = &(KF[nbrKFrame-1]);
		
		// calculate epipolar line start and end point in old image
		Mat<T> KinvP(3,1);
		KinvP.set( fxi*u+cxi, 1,1);
		KinvP.set( fyi*v+cyi, 2,1);
		KinvP.set( (T)1.0f, 3,1);
		
		Mat<T> pInf( referenceFrame->K_otherToThis_R * KinvP);
		Mat<T> pReal( (1.0f/prior_idepth)*pInf + referenceFrame->K_otherToThis_t );
	
		float rescaleFactor = pReal.get(3,1) * prior_idepth;

		float firstX = u - 2*epxn*rescaleFactor;
		float firstY = v - 2*epyn*rescaleFactor;
		float lastX = u + 2*epxn*rescaleFactor;
		float lastY = v + 2*epyn*rescaleFactor;
		
		// OOB error :
		if (firstX <= 0 || firstX >= this->w - 2
			|| firstY <= 0 || firstY >= this->h - 2
			|| lastX <= 0 || lastX >= this->w - 2
			|| lastY <= 0 || lastY >= this->h - 2) 
		{
			return -1;
		}

		if(!(rescaleFactor > 0.7f && rescaleFactor < 1.4f))
		{
			//it has be experienced that the mean of this data, when it is assumed correct, is 1.01f
			rs->add(-20);
			rs->mean(-200,rescaleFactor);
			return -1;
		}

		//std::cout << "RESCALE FACTOR = " << rescaleFactor << " prior depth = " << prior_idepth << std::endl;

		// calculate values to search for
		float realVal_p1 = activeKF->getInterpolatedElement(u + epxn*rescaleFactor, v + epyn*rescaleFactor);
		float realVal_m1 = activeKF->getInterpolatedElement(u - epxn*rescaleFactor, v - epyn*rescaleFactor);
		float realVal = activeKF->getInterpolatedElement(u, v);
		float realVal_m2 = activeKF->getInterpolatedElement(u - 2*epxn*rescaleFactor, v - 2*epyn*rescaleFactor);
		float realVal_p2 = activeKF->getInterpolatedElement(u + 2*epxn*rescaleFactor, v + 2*epyn*rescaleFactor);


		Mat<T> pClose( pInf + max_idepth*referenceFrame->K_otherToThis_t);
		// if the assumed close-point lies behind the
		// image, have to change that.
		if(pClose.get(3,1) < 0.001f)
		{
			max_idepth = (0.001f-pInf.get(3,1)) / referenceFrame->K_otherToThis_t.get(3,1);
			pClose = pInf + max_idepth*referenceFrame->K_otherToThis_t;
		}
		pClose *= (T)(1.0f/pClose.get(3,1)); // pos in new image of point (xy), assuming max_idepth

		Mat<T> pFar( pInf + min_idepth*referenceFrame->K_otherToThis_t);
		// if the assumed far-point lies behind the image or closer than the near-point,
		// we moved past the Point it and should stop.
		if(pFar.get(3,1) < 0.001f || max_idepth < min_idepth)
		{
#ifdef DEBUGGING				
			std::cout << "PFAR Z = " << pFar.get(3,1) << " MAX IDEPTH = " << max_idepth << " MIN IDEPTH = " << min_idepth << std::endl;
			pInf.afficher();
#endif			
			return -1;
		}
		pFar *= (T)(1.0f/ pFar.get(3,1)); // pos in new image of point (xy), assuming min_idepth


		// check for nan due to eg division by zero.
		//BIG ARITHMETIC ERROR
		if(isnan((float)(pFar.get(1,1)+pClose.get(1,1))) )
		{
			std::cout << "BIG NAN ERROR." << std::endl;
			return -4;
		}

		// calculate increments in which we will step through the epipolar line.
		// they are sampleDist (or half sample dist) long
		float incx = pClose.get(1,1) - pFar.get(1,1);
		float incy = pClose.get(2,1) - pFar.get(2,1);
		float eplLength = sqrt(incx*incx+incy*incy);
		if(!eplLength > 0 || std::isinf(eplLength)) return -4;

		if(eplLength > MAX_EPL_LENGTH_CROP)
		{
			//then we have make it closer... :
			pClose.set( pFar.get(1,1) + incx*MAX_EPL_LENGTH_CROP/eplLength, 1,1);
			pClose.set( pFar.get(2,1) + incy*MAX_EPL_LENGTH_CROP/eplLength, 2,1);
		}

		incx *= GRADIENT_SAMPLE_DIST/eplLength;
		incy *= GRADIENT_SAMPLE_DIST/eplLength;


		// extend one sample_dist to left & right.
		pFar.set( pFar.get(1,1) - incx, 1,1);
		pFar.set( pFar.get(2,1) - incy, 2,1);
		pClose.set( pClose.get(1,1) + incx, 1,1);
		pClose.set( pClose.get(2,1) + incy, 2,1);


		/*
		// make epl long enough (pad a little bit).
		if(eplLength < MIN_EPL_LENGTH_CROP)
		{
			float pad = (MIN_EPL_LENGTH_CROP - (eplLength)) / 2.0f;
			pFar[0] -= incx*pad;
			pFar[1] -= incy*pad;

			pClose[0] += incx*pad;
			pClose[1] += incy*pad;
		}
		*/

		// if inf point is outside of image: skip pixel.
		if(
				pFar.get(2,1) <= SAMPLE_POINT_TO_BORDER ||
				pFar.get(2,1) >= this->w-SAMPLE_POINT_TO_BORDER ||
				pFar.get(1,1) <= SAMPLE_POINT_TO_BORDER ||
				pFar.get(1,1) >= this->h-SAMPLE_POINT_TO_BORDER)
		{
			//OOB error
#ifdef DEBUGGING					
			std::cout << "OOB ERROR. prior idepth = " << prior_idepth << " h,w = " << this->h << " , " << this->w << " pFar : " << pFar.get(1,1) << "," << pFar.get(2,1) <<  std::endl;
#endif			
			return -1;
		}



		// if near point is outside: move inside, and test length again.
		if(
				pClose.get(2,1) <= SAMPLE_POINT_TO_BORDER ||
				pClose.get(2,1) >= this->w-SAMPLE_POINT_TO_BORDER ||
				pClose.get(1,1) <= SAMPLE_POINT_TO_BORDER ||
				pClose.get(1,1) >= this->h-SAMPLE_POINT_TO_BORDER)
		{
			if(pClose.get(1,1) <= SAMPLE_POINT_TO_BORDER)
			{
				float toAdd = (SAMPLE_POINT_TO_BORDER - pClose.get(1,1)) / incx;
				pClose.set( pClose.get(1,1) + toAdd * incx, 1,1);
				pClose.set( pClose.get(2,1) + toAdd * incy, 2,1);
			}
			else if(pClose.get(2,1) >= this->w-SAMPLE_POINT_TO_BORDER)
			{
				float toAdd = (this->w-SAMPLE_POINT_TO_BORDER - pClose.get(2,1)) / incy;
				pClose.set( pClose.get(1,1) + toAdd * incx, 1,1);
				pClose.set( pClose.get(2,1) + toAdd * incy, 2,1);
			}

			if(pClose.get(2,1) <= SAMPLE_POINT_TO_BORDER)
			{
				float toAdd = (SAMPLE_POINT_TO_BORDER - pClose.get(2,1)) / incy;
				pClose.set( pClose.get(1,1) + toAdd * incx, 1,1);
				pClose.set( pClose.get(2,1) + toAdd * incy, 2,1);
			}
			else if(pClose.get(1,1) >= this->h-SAMPLE_POINT_TO_BORDER)
			{
				float toAdd = (this->h-SAMPLE_POINT_TO_BORDER - pClose.get(2,1)) / incx;
				pClose.set( pClose.get(1,1) + toAdd * incx, 1,1);
				pClose.set( pClose.get(2,1) + toAdd * incy, 2,1);
			}

			// get new epl length
			float fincx = pClose.get(1,1) - pFar.get(1,1);
			float fincy = pClose.get(2,1) - pFar.get(2,1);
			float newEplLength = sqrt(fincx*fincx+fincy*fincy);

			// test again
			if(
					pClose.get(2,1) <= SAMPLE_POINT_TO_BORDER ||
					pClose.get(2,1) >= this->w-SAMPLE_POINT_TO_BORDER ||
					pClose.get(1,1) <= SAMPLE_POINT_TO_BORDER ||
					pClose.get(1,1) >= this->h-SAMPLE_POINT_TO_BORDER ||
					newEplLength < 8.0f
					)
			{
				//OOB error
#ifdef DEBUGGING						
				std::cout << "OOB ERROR 2 prior idepth = " << prior_idepth << " h,w = " << this->h << " , " << this->w << " pClose : " << pClose.get(1,1) << "," << pClose.get(2,1) <<  " ; new epl = " << newEplLength << std::endl;
#endif				
				return -1;
			}


		}


		// from here on:
		// - pInf: search start-point
		// - p0: search end-point
		// - incx, incy: search steps in pixel
		// - eplLength, min_idepth, max_idepth: determines search-resolution, i.e. the result's variance.


		float cpx = pFar.get(1,1);
		float cpy =  pFar.get(2,1);

		float val_cp_m2 = referenceFrame->I.getInterpolatedElement(cpx-2.0f*incx, cpy-2.0f*incy);
		float val_cp_m1 = referenceFrame->I.getInterpolatedElement(cpx-incx, cpy-incy);
		float val_cp = referenceFrame->I.getInterpolatedElement(cpx, cpy);
		float val_cp_p1 = referenceFrame->I.getInterpolatedElement(cpx+incx, cpy+incy);
		float val_cp_p2;



		/*
		 * Subsequent exact minimum is found the following way:
		 * - assuming lin. interpolation, the gradient of Error at p1 (towards p2) is given by
		 *   dE1 = -2sum(e1*e1 - e1*e2)
		 *   where e1 and e2 are summed over, and are the residuals (not squared).
		 *
		 * - the gradient at p2 (coming from p1) is given by
		 * 	 dE2 = +2sum(e2*e2 - e1*e2)
		 *
		 * - linear interpolation => gradient changes linearely; zero-crossing is hence given by
		 *   p1 + d*(p2-p1) with d = -dE1 / (-dE1 + dE2).
		 *
		 *
		 *
		 * => I for later exact min calculation, I need sum(e_i*e_i),sum(e_{i-1}*e_{i-1}),sum(e_{i+1}*e_{i+1})
		 *    and sum(e_i * e_{i-1}) and sum(e_i * e_{i+1}),
		 *    where i is the respective winning index.
		 */


		// walk in equally sized steps, starting at depth=infinity.
		int loopCounter = 0;
		float best_match_x = -1;
		float best_match_y = -1;
		float best_match_err = 1e50;
		float second_best_match_err = 1e50;

		// best pre and post errors.
		float best_match_errPre=NAN, best_match_errPost=NAN, best_match_DiffErrPre=NAN, best_match_DiffErrPost=NAN;
		bool bestWasLastLoop = false;

		float eeLast = -1; // final error of last comp.

		// alternating intermediate vars
		float e1A=NAN, e1B=NAN, e2A=NAN, e2B=NAN, e3A=NAN, e3B=NAN, e4A=NAN, e4B=NAN, e5A=NAN, e5B=NAN;

		int loopCBest=-1, loopCSecond =-1;
		while(((incx < 0) == (cpx > pClose.get(1,1)) && (incy < 0) == (cpy > pClose.get(2,1))) || loopCounter == 0)
		{
			// interpolate one new point
			val_cp_p2 = referenceFrame->I.getInterpolatedElement(cpx+2*incx, cpy+2*incy);


			// hacky but fast way to get error and differential error: switch buffer variables for last loop.
			float ee = 0;
			if(loopCounter%2==0)
			{
				// calc error and accumulate sums.
				e1A = val_cp_p2 - realVal_p2;ee += e1A*e1A;
				e2A = val_cp_p1 - realVal_p1;ee += e2A*e2A;
				e3A = val_cp - realVal;      ee += e3A*e3A;
				e4A = val_cp_m1 - realVal_m1;ee += e4A*e4A;
				e5A = val_cp_m2 - realVal_m2;ee += e5A*e5A;
			}
			else
			{
				// calc error and accumulate sums.
				e1B = val_cp_p2 - realVal_p2;ee += e1B*e1B;
				e2B = val_cp_p1 - realVal_p1;ee += e2B*e2B;
				e3B = val_cp - realVal;      ee += e3B*e3B;
				e4B = val_cp_m1 - realVal_m1;ee += e4B*e4B;
				e5B = val_cp_m2 - realVal_m2;ee += e5B*e5B;
			}


			// do I have a new winner??
			// if so: set.
			if(ee < best_match_err)
			{
				// put to second-best
				second_best_match_err=best_match_err;
				loopCSecond = loopCBest;

				// set best.
				best_match_err = ee;
				loopCBest = loopCounter;

				best_match_errPre = eeLast;
				best_match_DiffErrPre = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
				best_match_errPost = -1;
				best_match_DiffErrPost = -1;

				best_match_x = cpx;
				best_match_y = cpy;
				bestWasLastLoop = true;
			}
			// otherwise: the last might be the current winner, in which case i have to save these values.
			else
			{
				if(bestWasLastLoop)
				{
					best_match_errPost = ee;
					best_match_DiffErrPost = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
					bestWasLastLoop = false;
				}

				// collect second-best:
				// just take the best of all that are NOT equal to current best.
				if(ee < second_best_match_err)
				{
					second_best_match_err=ee;
					loopCSecond = loopCounter;
				}
			}


			// shift everything one further.
			eeLast = ee;
			val_cp_m2 = val_cp_m1; val_cp_m1 = val_cp; val_cp = val_cp_p1; val_cp_p1 = val_cp_p2;

			cpx += incx;
			cpy += incy;

			loopCounter++;
		}

		// if error too big, will return -3, otherwise -2.
		/*
		if(best_match_err > 4.0f*(float)MAX_ERROR_STEREO)
		{
			std::cout << "ERROR TOO BIG..... : " << best_match_err << std::endl;
			return -3;
		}
		*/


		// check if clear enough winner
		/*
		if(fabs_(loopCBest - loopCSecond) > 1.0f && MIN_DISTANCE_ERROR_STEREO * best_match_err > second_best_match_err)
		{
			std::cout << "NO CLEAR ENOUGH WINNER...." << std::endl;
			return -2;
		}
		*/

		
		bool didSubpixel = false;
		/*
		if(useSubpixelStereo)
		{
			// ================== compute exact match =========================
			// compute gradients (they are actually only half the real gradient)
			float gradPre_pre = -(best_match_errPre - best_match_DiffErrPre);
			float gradPre_this = +(best_match_err - best_match_DiffErrPre);
			float gradPost_this = -(best_match_err - best_match_DiffErrPost);
			float gradPost_post = +(best_match_errPost - best_match_DiffErrPost);

			// final decisions here.
			bool interpPost = false;
			bool interpPre = false;

			// if one is oob: return false.
			if(enablePrintDebugInfo && (best_match_errPre < 0 || best_match_errPost < 0))
			{
				stats->num_stereo_invalid_atEnd++;
			}


			// - if zero-crossing occurs exactly in between (gradient Inconsistent),
			else if((gradPost_this < 0) ^ (gradPre_this < 0))
			{
				// return exact pos, if both central gradients are small compared to their counterpart.
				if(enablePrintDebugInfo && (gradPost_this*gradPost_this > 0.1f*0.1f*gradPost_post*gradPost_post ||
				   gradPre_this*gradPre_this > 0.1f*0.1f*gradPre_pre*gradPre_pre))
					stats->num_stereo_invalid_inexistantCrossing++;
			}

			// if pre has zero-crossing
			else if((gradPre_pre < 0) ^ (gradPre_this < 0))
			{
				// if post has zero-crossing
				if((gradPost_post < 0) ^ (gradPost_this < 0))
				{
					if(enablePrintDebugInfo) stats->num_stereo_invalid_twoCrossing++;
				}
				else
					interpPre = true;
			}

			// if post has zero-crossing
			else if((gradPost_post < 0) ^ (gradPost_this < 0))
			{
				interpPost = true;
			}

			// if none has zero-crossing
			else
			{
				if(enablePrintDebugInfo) stats->num_stereo_invalid_noCrossing++;
			}


			// DO interpolation!
			// minimum occurs at zero-crossing of gradient, which is a straight line => easy to compute.
			// the error at that point is also computed by just integrating.
			if(interpPre)
			{
				float d = gradPre_this / (gradPre_this - gradPre_pre);
				best_match_x -= d*incx;
				best_match_y -= d*incy;
				best_match_err = best_match_err - 2*d*gradPre_this - (gradPre_pre - gradPre_this)*d*d;
				if(enablePrintDebugInfo) stats->num_stereo_interpPre++;
				didSubpixel = true;

			}
			else if(interpPost)
			{
				float d = gradPost_this / (gradPost_this - gradPost_post);
				best_match_x += d*incx;
				best_match_y += d*incy;
				best_match_err = best_match_err + 2*d*gradPost_this + (gradPost_post - gradPost_this)*d*d;
				if(enablePrintDebugInfo) stats->num_stereo_interpPost++;
				didSubpixel = true;
			}
			else
			{
				if(enablePrintDebugInfo) stats->num_stereo_interpNone++;
			}
		}
		*/


		// sampleDist is the distance in pixel at which the realVal's were sampled
		float sampleDist = GRADIENT_SAMPLE_DIST*rescaleFactor;

		float gradAlongLine = 0;
		float tmp = realVal_p2 - realVal_p1;  gradAlongLine+=tmp*tmp;
		tmp = realVal_p1 - realVal;  gradAlongLine+=tmp*tmp;
		tmp = realVal - realVal_m1;  gradAlongLine+=tmp*tmp;
		tmp = realVal_m1 - realVal_m2;  gradAlongLine+=tmp*tmp;

		gradAlongLine /= sampleDist*sampleDist;

		// check if interpolated error is OK. use evil hack to allow more error if there is a lot of gradient.
		/*
		if(best_match_err > (float)MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20)
		{
			std::cout << "INTERPOLATED ERROR IS NOT GOOD..." << std::endl;
			return -3;
		}
		*/


		// ================= calc depth (in KF) ====================
		// * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we search for, in the KF.
		// * best_match_x = x-coordinate of found correspondence in the reference frame.

		float idnew_best_match;	// depth in the new image
		//float idnew_best_matchX, idnew_best_matchY;
		float alpha; // d(idnew_best_match) / d(disparity in pixel) == conputed inverse depth derived by the pixel-disparity.
		//float alphaX, alphaY;
		float nominator;
		if(incx*incx>incy*incy)
		{
			float oldX = fxi*best_match_x+cxi;
			/*float*/ nominator = (oldX*referenceFrame->otherToThis_t.get(3,1) - referenceFrame->otherToThis_t.get(1,1));
			float dot0 = ( referenceFrame->otherToThis_R_row0 * KinvP ).get(1,1);
			float dot2 = ( referenceFrame->otherToThis_R_row2 * KinvP ).get(1,1);

			//std::cout << "dot0 : " << dot0 << " ; oldX*dot2 : " << oldX*dot2 << " ; nominator : " << nominator << std::endl;
			idnew_best_match = (dot0 - oldX*dot2) / nominator;
			alpha = incx*fxi*(dot0*referenceFrame->otherToThis_t.get(3,1) - dot2*referenceFrame->otherToThis_t.get(1,1)) / (nominator*nominator);

		}
		else
		{
			float oldY = fyi*best_match_y+cyi;

			/*float*/ nominator = (oldY*referenceFrame->otherToThis_t.get(3,1) - referenceFrame->otherToThis_t.get(2,1));
			float dot1 = ( referenceFrame->otherToThis_R_row1 * KinvP ).get(1,1);
			float dot2 = ( referenceFrame->otherToThis_R_row2 * KinvP ).get(1,1);

			idnew_best_match = (dot1 - oldY*dot2) / nominator;
			alpha = incy*fyi*(dot1*referenceFrame->otherToThis_t.get(3,1) - dot2*referenceFrame->otherToThis_t.get(2,1)) / (nominator*nominator);

		}
		
		/*
		idnew_best_match = (idnew_best_matchX+idnew_best_matchY)/2;
		alpha = (alphaX+alphaY)/2;

		std::cout << "dot0 : " << dot0 << " ; oldX*dot2 : " << oldX*dot2 << " ; nominator : " << nominator << std::endl;
		std::cout << " estX = " << idnew_best_matchX << " ; estY : " << idnew_best_matchY << " ; est = " << idnew_best_match << std::endl;
		*/
		if(idnew_best_match < 0)
		{
			//std::cout << "NEW ESTIMATION IDEPTH IS NEGATIVE.... : " << idnew_best_match <<  std::endl;
			//return -2;
			idnew_best_match = fabs_(idnew_best_match);
		}

		// ================= calc var (in NEW image) ====================

		// calculate error from photometric noise
		float photoDispError = 4.0f * cameraPixelNoise2 / (gradAlongLine + DIVISION_EPS);

		float trackingErrorFac = 0.25f*(1.0f+referenceFrame->initialTrackedResidual);

		// calculate error from geometric noise (wrong camera pose / calibration)
		float gx =  gradKF[nbrKFrame-1].getInterpolatedElement( u, v) ;
		float gy = gx;
		//TODO : register x and y KF grads...
		
		float geoDispError = (gx*epxn + gy*epyn) + DIVISION_EPS;
		geoDispError = trackingErrorFac*trackingErrorFac*(gx*gx + gy*gy) / (geoDispError*geoDispError);


		//geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5 *result_idepth);

		// final error consists of a small constant part (discretization error),
		// geometric and photometric error.
		result_var = alpha*alpha*((didSubpixel ? 0.05f : 0.5f)*sampleDist*sampleDist +  geoDispError + photoDispError);	// square to make variance
		
		/*RUNNING STATS : */
		//--------------------------------------------------------------
		
		rs->ladd("nominatorNBR");
		rs->lmean("nominatorSUM", nominator);
		
		rs->ladd("alphaNBR");
		rs->lmean("alphaSUM", alpha);
		
		rs->ladd("geoDispNBR");
		rs->lmean("geoDispSUM", geoDispError);
		
		rs->ladd("phototDispNBR");
		rs->lmean("photoDispSUM", photoDispError);
		/*
		rs->ladd("trackedErrNBR");
		rs->lmean("trackedErrSUM", referenceFrame->initialTrackedResidual);
		*/
		//--------------------------------------------------------------


		/*
		if(plotStereoImages)
		{
			if(rand()%5==0)
			{
				//if(rand()%500 == 0)
				//	printf("geo: %f, photo: %f, alpha: %f\n", sqrt(geoDispError), sqrt(photoDispError), alpha, sqrt(result_var));


				//int idDiff = (keyFrame->pyramidID - referenceFrame->id);
				//cv::Scalar color = cv::Scalar(0,0, 2*idDiff);// bw

				//cv::Scalar color = cv::Scalar(sqrt(result_var)*2000, 255-sqrt(result_var)*2000, 0);// bw

	//			float eplLengthF = std::min((float)MIN_EPL_LENGTH_CROP,(float)eplLength);
	//			eplLengthF = std::max((float)MAX_EPL_LENGTH_CROP,(float)eplLengthF);
	//
	//			float pixelDistFound = sqrtf((float)((pReal[0]/pReal[2] - best_match_x)*(pReal[0]/pReal[2] - best_match_x)
	//					+ (pReal[1]/pReal[2] - best_match_y)*(pReal[1]/pReal[2] - best_match_y)));
	//
				float fac = best_match_err / ((float)MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20);

				cv::Scalar color = cv::Scalar(255*fac, 255-255*fac, 0);// bw


				//
				//if(rescaleFactor > 1)
				//	color = cv::Scalar(500*(rescaleFactor-1),0,0);
				//else
				//	color = cv::Scalar(0,500*(1-rescaleFactor),500*(1-rescaleFactor));
				//

				cv::line(debugImageStereoLines,cv::Point2f(pClose[0], pClose[1]),cv::Point2f(pFar[0], pFar[1]),color,1,8,0);
			}
		}
		*/

		result_idepth = idnew_best_match;

		result_eplLength = eplLength;
		
		//std::cout << "ID new = " << result_idepth << " : var = " << result_var << std::endl;

		return best_match_err;
	}
	
	
	
	
	
	
	
	/*
	bool DepthMap::observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats)
	{
		DepthMapPixelHypothesis* target = currentDepthMap+idx;

		Frame* refFrame = activeKeyFrameIsReactivated ? newest_referenceFrame : oldest_referenceFrame;

		if(refFrame->getTrackingParent() == activeKeyFrame)
		{
			bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();
			if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])
			{
				if(plotStereoImages)
					debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
				return false;
			}
		}

		float epx, epy;
		bool isGood = makeAndCheckEPL(x, y, refFrame, &epx, &epy, stats);
		if(!isGood) return false;

		if(enablePrintDebugInfo) stats->num_observe_create_attempted++;

		float new_u = x;
		float new_v = y;
		float result_idepth, result_var, result_eplLength;
		float error = doLineStereo(
				new_u,new_v,epx,epy,
				0.0f, 1.0f, 1.0f/MIN_DEPTH,
				refFrame, refFrame->image(0),
				result_idepth, result_var, result_eplLength, stats);

		if(error == -3 || error == -2)
		{
			target->blacklisted--;
			if(enablePrintDebugInfo) stats->num_observe_blacklisted++;
		}

		if(error < 0 || result_var > MAX_VAR)
			return false;
	
		result_idepth = UNZERO(result_idepth);

		// add hypothesis
		*target = DepthMapPixelHypothesis(
				result_idepth,
				result_var,
				VALIDITY_COUNTER_INITIAL_OBSERVE);

		if(plotStereoImages)
			debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,255); // white for GOT CREATED

		if(enablePrintDebugInfo) stats->num_observe_created++;
	
		return true;
	}
	*/
	
	
	bool observeDepthUpdate(const int x, const int y, const int idxTrackFrame)
	{
		float epx, epy;
		bool isGood = makeAndCheckEPL(x, y, idxTrackFrame, &epx, &epy);
		if(!isGood) 
		{
			//std::cout << "EPL NOT GOOD...!" << std::endl;
			rs->add(-10);
			return false;
		}

		// which exact point to track, and where from.
		/*
		float sv = sqrt(varIDKFsmoothed[nbrKFrame-1].get(x,y));
		float min_idepth = IDKFsmoothed[nbrKFrame-1].get(x,y) - sv;
		float max_idepth = IDKFsmoothed[nbrKFrame-1].get(x,y) + sv;
		*/
		float sv = sqrt(varIDKF[nbrKFrame-1].get(x,y));
		float min_idepth = IDKF[nbrKFrame-1].get(x,y) - sv;
		float max_idepth = IDKF[nbrKFrame-1].get(x,y) + sv;
		if(min_idepth < 0) min_idepth = 0;
		if(max_idepth > 1/MIN_DEPTH) max_idepth = 1/MIN_DEPTH;

		float result_idepth, result_var, result_eplLength;
		float error = doLineStereo(
				x,y,epx,epy,
				//min_idepth, IDKFsmoothed[nbrKFrame-1].get(x,y) ,max_idepth,
				min_idepth, IDKF[nbrKFrame-1].get(x,y) ,max_idepth,
				&(I[idxTrackFrame]),
				result_idepth, result_var, result_eplLength);

		//float diff = result_idepth - IDKFsmoothed[nbrKFrame-1].get(x,y);
		float diff = result_idepth - IDKF[nbrKFrame-1].get(x,y);

		// if oob: (really out of bounds)
		if(error == -1)
		{
			// do nothing, pixel got oob, but is still in bounds in original. I will want to try again.
			rs->add(-1);
			return false;
		}

		// if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
		else if(error == -2)
		{
			validityIDKF[nbrKFrame-1].set( validityIDKF[nbrKFrame-1].get(x,y) - VALIDITY_COUNTER_DEC, x,y);
			if(validityIDKF[nbrKFrame-1].get(x,y) < 0) validityIDKF[nbrKFrame-1].set(0,x,y);

			varIDKF[nbrKFrame-1].set( varIDKF[nbrKFrame-1].get(x,y) * FAIL_VAR_INC_FAC, x,y);
			if(varIDKF[nbrKFrame-1].get(x,y) > MAX_VAR)
			{
				blacklistedIDKF[nbrKFrame-1].set( blacklistedIDKF[nbrKFrame-1].get(x,y) -1, x,y);
			}
			
			rs->add(-2);
			return false;
		}

		// if not found (error too high)
		else if(error == -3)
		{
			rs->add(-3);
			return false;
		}

		else if(error == -4)
		{
			//... big arithmetic error
			rs->add(-4);
			return false;
		}

		else
		{
			// one more successful observation!
			// do textbook ekf update:
			// increase var by a little (prediction-uncertainty)
			float id_var = varIDKF[nbrKFrame-1].get(x,y)*SUCC_VAR_INC_FAC;

			// update var with observation
			/*test :*/
			//float w = result_var / (result_var + id_var);
			float w = 0.5f;
			float new_idepth = (1-w)*result_idepth + w*IDKF[nbrKFrame-1].get(x,y);
			IDKF[nbrKFrame-1].set( UNZERO(new_idepth), x,y);

			// variance can only decrease from observation; never increase.
			id_var = id_var * w;
			if(id_var < varIDKF[nbrKFrame-1].get(x,y))
				varIDKF[nbrKFrame-1].set( id_var, x,y);

			// increase validity!
			validityIDKF[nbrKFrame-1].set(  validityIDKF[nbrKFrame-1].get(x,y) + VALIDITY_COUNTER_INC, x,y);
			/*
			float absGrad = gradKF[nbrKFrame-1].get(x,y);
			
			if(target->validity_counter > VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
				target->validity_counter = VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
			*/
			
			/*RUNNING STATS : */
			//--------------------------------------------------------------
			rs->ladd("resultVarNBR");
			rs->lmean("resultVarSUM",result_var);
			rs->ladd("(1-w)NBR");
			rs->lmean("(1-w)SUM",1.0f-w);
			
			//--------------------------------------------------------------
			
			return true;
		}
	}
	
	
	bool regularizeDepthUpdate(const int x, const int y, const int idxTrackFrame)
	{
		float epx, epy;
		bool isGood = makeAndCheckEPL(x, y, idxTrackFrame, &epx, &epy);
		if(!isGood) 
		{
			//std::cout << "EPL NOT GOOD...!" << std::endl;
			rs->add(-10);
			return false;
		}

		// which exact point to track, and where from.
		/*
		float sv = sqrt(varIDKFsmoothed[nbrKFrame-1].get(x,y));
		float min_idepth = IDKFsmoothed[nbrKFrame-1].get(x,y) - sv;
		float max_idepth = IDKFsmoothed[nbrKFrame-1].get(x,y) + sv;
		*/
		float sv = sqrt(varIDKF[nbrKFrame-1].get(x,y));
		float min_idepth = IDKF[nbrKFrame-1].get(x,y) - sv;
		float max_idepth = IDKF[nbrKFrame-1].get(x,y) + sv;
		if(min_idepth < 0) min_idepth = 0;
		if(max_idepth > 1/MIN_DEPTH) max_idepth = 1/MIN_DEPTH;

		float result_idepth, result_var, result_eplLength;
		float error = doLineStereo(
				x,y,epx,epy,
				//min_idepth, IDKFsmoothed[nbrKFrame-1].get(x,y) ,max_idepth,
				min_idepth, IDKF[nbrKFrame-1].get(x,y) ,max_idepth,
				&(I[idxTrackFrame]),
				result_idepth, result_var, result_eplLength);

		//float diff = result_idepth - IDKFsmoothed[nbrKFrame-1].get(x,y);
		float diff = result_idepth - IDKF[nbrKFrame-1].get(x,y);

		// if oob: (really out of bounds)
		if(error == -1)
		{
			// do nothing, pixel got oob, but is still in bounds in original. I will want to try again.
			rs->add(-1);
			return false;
		}

		// if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
		else if(error == -2)
		{
			validityIDKF[nbrKFrame-1].set( validityIDKF[nbrKFrame-1].get(x,y) - VALIDITY_COUNTER_DEC, x,y);
			if(validityIDKF[nbrKFrame-1].get(x,y) < 0) validityIDKF[nbrKFrame-1].set(0,x,y);

			varIDKF[nbrKFrame-1].set( varIDKF[nbrKFrame-1].get(x,y) * FAIL_VAR_INC_FAC, x,y);
			if(varIDKF[nbrKFrame-1].get(x,y) > MAX_VAR)
			{
				blacklistedIDKF[nbrKFrame-1].set( blacklistedIDKF[nbrKFrame-1].get(x,y) -1, x,y);
			}
			
			rs->add(-2);
			return false;
		}

		// if not found (error too high)
		else if(error == -3)
		{
			rs->add(-3);
			return false;
		}

		else if(error == -4)
		{
			//... big arithmetic error
			rs->add(-4);
			return false;
		}

		else
		{
			// one more successful observation!
			// do textbook ekf update:
			// increase var by a little (prediction-uncertainty)
			float id_var = varIDKF[nbrKFrame-1].get(x,y)*SUCC_VAR_INC_FAC;

			// update var with observation
			/*test :*/
			//float w = result_var / (result_var + id_var);
			float w = 0.75f;
			float new_idepth = (1-w)*result_idepth + w*IDKF[nbrKFrame-1].get(x,y);
			IDKF[nbrKFrame-1].set( UNZERO(new_idepth), x,y);

			// variance can only decrease from observation; never increase.
			id_var = id_var * w;
			if(id_var < varIDKF[nbrKFrame-1].get(x,y))
				varIDKF[nbrKFrame-1].set( id_var, x,y);

			// increase validity!
			validityIDKF[nbrKFrame-1].set(  validityIDKF[nbrKFrame-1].get(x,y) + VALIDITY_COUNTER_INC, x,y);
			/*
			float absGrad = gradKF[nbrKFrame-1].get(x,y);
			
			if(target->validity_counter > VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
				target->validity_counter = VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
			*/
			
			/*RUNNING STATS : */
			//--------------------------------------------------------------
			rs->ladd("resultVarNBR");
			rs->lmean("resultVarSUM",result_var);
			rs->ladd("(1-w)NBR");
			rs->lmean("(1-w)SUM",1.0f-w);
			
			//--------------------------------------------------------------
			
			return true;
		}
	}
	
	
	
	
	//void observeDepthRow(int yMin, int yMax, const int idxTrackFrame)
	//void observeDepthRow(const Param<T>& param)
	void observeDepthRow( Param<T>* param, bool* finished)
	{
		int yMin = param->param.get(1,1);
		int yMax = param->param.get(2,1);
		int idxTrackFrame = param->param.get(3,1);
		
		int attempt = 0;
		int successes = 0;

		for(int y=yMin;y<yMax; y++)
		{
			for(int x=3;x<this->w-3;x++)
			{
				bool hasHypothesis = IDKF[nbrKFrame-1].get(x,y) > 0.0f ? true : false;

				if(hasHypothesis && gradKF[nbrKFrame-1].get(x,y) < gradTreshold)
				{
					IDKF[nbrKFrame-1].set( (T)0, x,y);
					continue;
				}

				if( blacklistedIDKF[nbrKFrame-1].get(x,y) < MIN_BLACKLIST)
					continue;


				bool success = false;
				/*if(!hasHypothesis)
					success = observeDepthCreate(x, y, idxTrackFrame);
				else*/
				if(hasHypothesis)
				{
					attempt++;
					success = observeDepthUpdate(x, y, idxTrackFrame);
				}

				if(success)
					successes++;
			}
		}
		
		//TODO : debug with nbr successes...
		std::cout << " YMIN/MAX = " << yMin << " / " << yMax << " ; SUCCESSED DEPTH OBSERVATION = " << successes << " / " << attempt << " == " << (((float)successes)/((float)attempt))*100.0f << " %. " << std::endl;
		
		*finished = true;

	}
	
	
	void observeDepth(const int idxTrackFrame)
	{
		
		//prepare the Frame :
		I[idxTrackFrame].prepareForStereoWith( invGJ(I[idxTrackFrame].deltaPose), this->K );
		
		//prepare the multi threading
		int nbrParam = 3;
		
		Mat<T> paramBegin(nbrParam,1);
		Mat<T> paramStep(nbrParam,1);
		Mat<T> paramEnd(nbrParam,1);
		
		
		int pad = 3;
		int lw = this->w-6;
		int step = lw/4;
		
#ifndef debugWITHOUTTHREADPOOL		
		paramBegin.set( pad, 1,1);
		paramBegin.set( pad+step, 2,1);
		paramBegin.set( idxTrackFrame,3,1);
#else
		paramBegin.set( 3, 1,1);
		paramBegin.set( this->w-3, 2,1);
		paramBegin.set( idxTrackFrame,3,1);
#endif//debugWITHOUTTHREADPOOL		

		paramStep.set( step,1,1);
		paramStep.set( step,2,1);
		paramStep.set( 0,3,1);
		paramEnd.set( this->w-step-pad, 1,1);
		paramEnd.set( this->w-pad, 2,1);
		paramEnd.set( idxTrackFrame,3,1);
		
		Param<T> pB(paramBegin,this);
		Param<T> pS(paramStep,this);
		Param<T> pE(paramEnd,this);
#ifndef debugWITHOUTTHREADPOOL
		threadPool->reduce( &VO<T>::observeDepthRow, this, pB, pS, pE);
#else
		clock_t timer = clock();
		bool finished=false;
		this->observeDepthRow( &pB,&finished);
		
		//MEAN & VAR:
		float mean = nonzeroMEAN( IDKF[nbrKFrame-1] );
		float var = nonzeroVAR( IDKF[nbrKFrame-1], mean);
		rs->tadd(22, mean);
		rs->tadd(23, var);
		
		
		std::cout << "DEPTH MEAN = " << mean << " & VAR = " << var << std::endl;
		
		std::cout << "DEPTH OBSERVATION TOOK : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " seconds." << std::endl;
#endif//debugWITHOUTTHREADPOOL		

	}
	
	void regularizeDepthRow( Param<T>* param, bool* finished)
	{
		int yMin = param->param.get(1,1);
		int yMax = param->param.get(2,1);
		int idxTrackFrame = param->param.get(3,1);
		
		int attempt = 0;
		int successes = 0;

		for(int y=yMin;y<yMax; y++)
		{
			for(int x=3;x<this->w-3;x++)
			{
				bool hasHypothesis = IDKF[nbrKFrame-1].get(x,y) > 0.0f ? true : false;

				if(hasHypothesis && gradKF[nbrKFrame-1].get(x,y) < gradTreshold)
				{
					IDKFsmoothed[nbrKFrame-1].set( (T)0, x,y);
					continue;
				}

				if( blacklistedIDKF[nbrKFrame-1].get(x,y) < MIN_BLACKLIST)
					continue;


				bool success = false;
				
				if(hasHypothesis)
				{
					attempt++;
					success = regularizeDepthUpdate(x, y, idxTrackFrame);
				}
				/*else
				{
					success = regularizeDepthCreate(x, y, idxTrackFrame);
				}*/

				if(success)
					successes++;
			}
		}
		
		//TODO : debug with nbr successes...
		std::cout << " YMIN/MAX = " << yMin << " / " << yMax << " ; SUCCESSED DEPTH REGULARIZATION = " << successes << " / " << attempt << " == " << (((float)successes)/((float)attempt))*100.0f << " %. " << std::endl;
		
		*finished = true;

	}
	
	
	void regularizeDepth(const int idxTrackFrame)
	{
		
		//prepare the Frame :
		I[idxTrackFrame].prepareForStereoWith( invGJ(I[idxTrackFrame].deltaPose), this->K );
		
		//prepare the multi threading
		int nbrParam = 3;
		
		Mat<T> paramBegin(nbrParam,1);
		Mat<T> paramStep(nbrParam,1);
		Mat<T> paramEnd(nbrParam,1);
		
		
		int pad = 3;
		int lw = this->w-6;
		int step = lw/4;
		
		/*
		paramBegin.set( pad, 1,1);
		paramBegin.set( pad+step, 2,1);
		paramBegin.set( idxTrackFrame,3,1);
		*/
		paramBegin.set( 3, 1,1);
		paramBegin.set( this->w-3, 2,1);
		paramBegin.set( idxTrackFrame,3,1);
		
		paramStep.set( step,1,1);
		paramStep.set( step,2,1);
		paramStep.set( 0,3,1);
		paramEnd.set( this->w-step-pad, 1,1);
		paramEnd.set( this->w-pad, 2,1);
		paramEnd.set( idxTrackFrame,3,1);
		
		Param<T> pB(paramBegin,this);
		Param<T> pS(paramStep,this);
		Param<T> pE(paramEnd,this);
		/*
		threadPool->reduce( &VO<T>::regularizeDepthRow, this, pB, pS, pE);
		*/
		clock_t timer = clock();
		bool finished=false;
		this->regularizeDepthRow( &pB,&finished);
		std::cout << "DEPTH REGULARIZATION TOOK : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " seconds." << std::endl;
		
	}
    
};



inline cv::Mat computeGradientXY(cv::Mat* im, cv::Mat* imgradX, cv::Mat* imgradY)
{
    int ddepth = CV_16S;
    int scale = 1;
    int delta = 0;

    if(im->channels() != 1)
        cv::cvtColor(*im,*im,CV_BGR2GRAY);
    //cv::GaussianBlur(*im,*im,cv::Size(0,0),(float)2.0/sqrt(2),(float)2.0/sqrt(2));


    //imgradX = new cv::Mat();
    *imgradX = cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type());
    //imgradY = new cv::Mat();
    *imgradY = cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type());

    // Gradient X
    Sobel( *im, *imgradX, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
    convertScaleAbs( *imgradX, *imgradX );

    // Gradient Y
    Sobel( *im, *imgradY, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
    convertScaleAbs( *imgradY, *imgradY );

    return ((float)(0.5))*( *imgradX+ *imgradY);
    /*
    cv::Mat grad;
    Sobel(*im,grad,ddepth,1,1,3,scale,delta,cv::BORDER_DEFAULT);
    //cv::GaussianBlur(grad,grad,cv::Size(0,0),(float)1.0/sqrt(2),(float)1.0/sqrt(2));
    convertScaleAbs(grad,grad);
    return absMAt(grad);
    */
}

template<typename T>
Mat<T> debugPose(Frame<T>* f1, Frame<T>* f2, const Mat<T>& dm1, const Mat<T>& R, const Mat<T>& t, const Mat<T>& K, const Mat<T>& invK, T factor)
{
    int h = f1->getLine();
    int w = f1->getColumn();
    int offsetx = K.get(1,3);
    int offsety = K.get(2,3);
    Mat<T> offset(3,1);
    offset.set( (T)offsetx, 1,1);
    offset.set( (T)offsety, 2,1);
    offset.set( (T)0, 3,1);
    
    Mat<T> pos(3,1);
    Mat<T> pos1(3,1);
    pos1.set((T)1.0,3,1);
    Mat<T> im((T)0,h,w);

    for(int i=h;i--;)
    {
        pos1.set(i+1,1,1);

        for(int j=w;j--;)
        {

            if(dm1.get(i+1,j+1) != 0.0)
            {
                pos1.set(j+1,2,1);

                //pos.afficher();

                pos = invK*(pos1+offset);

                pos = ((T)1.0/dm1.get(i+1,j+1))*pos;
                pos.set( ((T)1.0/dm1.get(i+1,j+1)), 3,1);

                //cout << " Renvoye vers : " << endl;
                //pos.afficher();

                pos = (R*pos+t);

                //T norme = norme2(pos);
                //pos = ((T)1.0/norme)*pos;
                

                pos = K*pos;
                //pos.mat[0][0] =  K.get(1,1)*pos.get(1,1)/pos.get(3,1) ;
	        //pos.mat[0][1] = K.get(2,2)*pos.get(2,1)/pos.get(3,1) ;
                
                //homogeneousNormalization(&pos);
                if(pos.get(3,1) != (T)0.0 && pos.get(3,1) != (T)1.0)
                    pos = ((T)1.0/pos.get(3,1))*pos;
                    
                //pos1 -= offset;                
                pos -= ((T)1)*offset;
                //pos.afficher();
                //pos1.afficher();
              
                //cout << "////////////////////////////////" << endl;

		//T value = (f1->get(pos1)).get(1,1);
		//cout << value << " " << 10*h/2+pos.get(1,1) << " " << 10*w/2+pos.get(2,1) << endl;
                //im.set( (f1->get(pos1)).get(1,1), 10*h/2+pos.get(1,1), 10*w/2+pos.get(2,1) );
                im.set( (f1->get(pos1)).get(1,1), pos.get(1,1), pos.get(2,1) );
                
                
            }
        }

    }


    Mat<T> imDiff((*f2)-im);
    afficherMat( string("debug ref2, f2, imDiff"), &im, f2, &imDiff,true, factor);
    //afficherMat( string("debug ref2"), &im, (Mat<T>*)NULL, (Mat<T>*)NULL,true, factor/10);
    
    return dm1-im;

}

template<typename T>
float nonzeroMEAN(const Mat<T>& m)
{
	float r = 0.0f;
	int nbr = 0;
	for(int i=1;i<=m.getLine();i++)
	{
		for(int j=1;j<=m.getColumn();j++)
		{
			if( m.get(i,j) > 0.0f)
			{
				r+= m.get(i,j);
				nbr++;
			}
		}
	}
	
	if(nbr==0)
		throw;
			
	return r/(float)nbr;
}


template<typename T>
float nonzeroVAR(const Mat<T>& m, const T& nzmean)
{
	float r = 0.0f;
	int nbr = 0;
	for(int i=1;i<=m.getLine();i++)
	{
		for(int j=1;j<=m.getColumn();j++)
		{
			if( (float)(m.get(i,j)) > 0.0f)
			{
				r+= (m.get(i,j)-nzmean)*(m.get(i,j)-nzmean);
				nbr++;
			}
		}
	}
	
	if(nbr<=1)
		throw;
			
	return r/(float)(nbr-1);
}

#endif
