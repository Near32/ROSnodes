#define OPENCV_USE
#include <cstdlib>
#include <cstdio>
#include <string>
#include "../StructureFromMotion/SFM.h"




#include "../RAND/rand.h"
#include "../ONSC/ONSC.h"
#include "../ONSC/ONSCNewton.h"
#include "../MVG/MVG.h"
#include "../MVG/Frame.h"

//#include <QApplication>
//#include "myWindow.h"
#include "../ONSC/LS3wSDOptim2.h"
#include "../EKF/v2/EKF.h"

//#define IRRLICHT_USE
#ifdef IRRLICHT_USE
#include <irr/irrlicht.h>
#endif

Mat<float> Mouse((float)0,2,1);
bool update = false;
void CallBackMouseFunc(int event, int x, int y, int flag, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        Mouse.set( (float)x, 1,1);
        Mouse.set( (float)y, 2,1);
        update = true;
        cout << "Position de la souris : " << endl;
        Mouse.afficher();
    }
}


Mat<float> c2pointsMouseClick();
inline cv::Mat computeGradientXY(cv::Mat* im, cv::Mat* imgradX, cv::Mat* imgradY);
Mat<float> initializeInvDM(Frame<float>* frame,float gradIntTreshold = 255/10, float variance = 0.5, float mean = 0.9);
Mat<float> poseEstimationLSGRAD(const Mat<float>& lastKhiPose, Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Frame<float>* gradI1X, Frame<float>* gradI1Y, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI, int pyramidDepth = 3);
inline int DepthUpdate21(int nbrHypSearch, Frame<float>* f1, Frame<float>* f2, Mat<float>* invDepthMap1, Mat<float>* invDepthMap2, Mat<float>* R, Mat<float>* t, Mat<float>* var_invDepthMap1, Mat<float>* var_invDepthMap2, Mat<float>* K, Mat<float>* invK, float factor, Mat<float>* validityCheck, Mat<int>* blacklisted, float treshBlacklist = 3.0f);
Mat<float> debugPose(Frame<float>* f1, Frame<float>* f2, const Mat<float>& dm1, const Mat<float>& R, const Mat<float>& t, const Mat<float>& K, const Mat<float>& invK, float factor = 1.0/20);
template<typename T>
void regularizePropagatedDepthMap( Frame<T>* f2Grad, int gradIntTreshold, Mat<T>* invdm, Mat<T>* var_invdm, T depthvar_init = (T)10.5);
template<typename T>
Mat<T> propagateDepthMap(Frame<float>* frame, float gradIntTreshold, float variance, float mean, Mat<T>* K, Mat<T>* invK, Mat<T>* deltaPose, Mat<T>* invdm);


inline cv::Mat computeGradientXY(cv::Mat* im, cv::Mat* imgradX, cv::Mat* imgradY)
{
    int ddepth = CV_16S;
    int scale = 1;
    int delta = 0;

    if(im->channels() != 1)
        cv::cvtColor(*im,*im,CV_BGR2GRAY);
    cv::GaussianBlur(*im,*im,cv::Size(0,0),(float)2.0/sqrt(2),(float)2.0/sqrt(2));


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

Mat<float> initializeInvDM(Frame<float>* frame, float gradIntTreshold, float variance, float mean)
{
    float meanid = 0.0f;
    int nbrpoint = 0;
    
    if(gradIntTreshold >= 255)
    	gradIntTreshold = 255;
    else if(gradIntTreshold < 0)
    	gradIntTreshold = 0;
    	 
    NormalRand gen(0,variance,(long)170);
    Mat<float> idm(0.0, frame->getLine(), frame->getColumn());
    Mat<float> x(0.0,2,1);

    for(int i=frame->getLine();i--;)
    {
        x.set(i+1,1,1);

        for(int j=frame->getColumn();j--;)
        {
            x.set(j+1,2,1);

            //inverse depth value is assigned only if the gradient is sufficient.
            if( fabs_(frame->get(x).get(1,1)) >= (float)gradIntTreshold)
            {
            	float temp = (float)mean+(float)gen.dev();
                idm.set( temp, i+1,j+1);
                //initialized inverse depth between -0.9 and 1.1.
                
                meanid += temp;
                nbrpoint++;
            }

        }
    }
    
    meanid /= nbrpoint;
    (*frame) *= 1.0f/meanid;

    //TODO : regularization...

    return idm;
}


#define debugOptim
Mat<float> poseEstimationLSGRAD(const Mat<float>& lastKhiPose, Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Frame<float>* gradI2X, Frame<float>* gradI2Y, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI, int pyramidDepth)
{
    clock_t timer = clock();
    int nbrL = pyramidDepth;


    LSSE3<float> instanceLS(lastKhiPose, f1,f2, gradI2X, gradI2Y, K,invK,dm1, var_invDepthMap1, nbrL);
    Mat<float> X( instanceLS.energy( Mat<float>((float)nbrL,1,1) ) );
    
#ifdef debugOptim    
    cout << " L'execution de LSSE3 a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;
#endif    
    //X.afficher();
    *RI = absM(instanceLS.getResidual(nbrL));

    //return expM(X); //if sum of sol
    return X;	// if product of exp sol
    //return expMSIM3( operatorC(X,Mat<float>((float)1,1,1) ) );
}


#define debugDepth
//#define debug_lvl1
#define SMOOTHER
#define RESCALER
#define saliency
int DepthUpdate21(int nbrHypSearch, Frame<float>* f1, Frame<float>* f2, Mat<float>* invDepthMap1, Mat<float>* invDepthMap2, Mat<float>* R, Mat<float>* t, Mat<float>* var_invDepthMap1, Mat<float>* var_invDepthMap2, Mat<float>* K, Mat<float>* invK, float factor, Mat<float>* validityCheck,
Mat<int>* blacklisted, float treshBlacklist)
{
	
	Mat<float> offset(3,1);
	offset.set( K->get(1,3)/factor, 1,1);
	offset.set( K->get(2,3)/factor, 2,1);	
	offset.set( (float)0, 3,1);
	float h = f1->getLine();
	float w = f1->getColumn();	
	float f = (K->get(1,1)+K->get(2,2))/2;
	
	int count = 0;
	Mat<float> x1((float)1,3,1);
	Mat<float> x1offset(3,1);
	Mat<float> projx1(3,1);
	
	float rapport = abs(t->get(3,1)/(abs(t->get(1,1))+abs(t->get(2,1))));
	rapport = (rapport > 1e-10 ? rapport : 1e-10 );
	float depthVar_init = (float)0.5;// * (rapport > 1e-2 ? rapport : 1.0 );
	float depthVar_search = (float)10.0;	
	
	
	int nbrHyp = nbrHypSearch;
	Mat<float> x_2(3,1);
	Mat<float> x_2offset(3,1);
	Mat<float> x2offsetmin(3,1);
	
	NormalRand gen(0,depthVar_init,(long)170);
	Mat<float> epiline(3,1);
	float lambdamax = 0;
	float lambdamin = 0;
	
#ifdef debugDepth	
	cv::Mat debugIm( Mat2cvp(*f2, *f2, *f2) );
	Mat<float> disparityMap(0.0f,h,w);
	int draw = 0;
#endif

	//-------------
	Mat<float> Xlambda((*K)*(((float)-1)*(*t)));
	homogeneousNormalization(&Xlambda);
	Mat<float> Xlambdanooffset( Xlambda-offset);
	
	Mat<float> XdispMIN(3,1);
	Mat<float> XdispMAX(3,1);	
	//---------------
	
	float EPI_LENGTH_MAX = 10;
	float EPI_LENGTH_MIN = 5;
	//Mat<float> disparityMap((float)0,h,w);
	Mat<float> KRKinv( (*K) * (*R) *(*invK) );
	Mat<float> Kt((*K)*(*t));
	
	float idmean = 0.0f;
	float len_id_search = 0.0f;
	float epiLength = 0.0f;
	float  var_Inoise = 4;
	float var_epinoise = 1e-4;///norme2(*t);
	
	float meanPhotoDist=0.0f;
	
	int dostereoCount = 0;
	Mat<int> doStereoMap(0,f1->getLine(),f1->getColumn());
	
	for(int it=1;it--;)
	{
        for(int i=invDepthMap1->getLine();i--;)
        {
			x1.set(i+1,1,1);

			for(int j=invDepthMap1->getColumn();j--;)
			{

				if(blacklisted->get(i+1,j+1))
				{
					invDepthMap2->set( 0.0f, i+1,j+1);
					invDepthMap1->set( 0.0f, i+1,j+1);
					var_invDepthMap1->set( 0.0f, i+1,j+1);
					var_invDepthMap2->set( 0.0f, i+1,j+1);
				}
				
				if(invDepthMap1->get(i+1,j+1) != 0.0 )
				{
					count++;

					float dist = (float)1.0/invDepthMap1->get(i+1,j+1);
					
					if(dist == 0.0f )
						dist = numeric_limits<float>::epsilon();
						
					if(dist < 0.0f)
						dist = -dist;
					
					float gx = (f1->get(i,j+1)-f1->get(i+2,j+1));
					float gy = (f1->get(i+1,j)-f1->get(i+1,j+2));
					Mat<float> grad((float)0,3,1);
					grad.set(gx,1,1);
					grad.set(gy,2,1);
					//float localgrad = fabs_(gx)+fabs_(gy);
					float localgrad = sqrt(gx*gx+gy*gy);
					float varest = rapport / (localgrad != (float)0 ? localgrad : (float)1);
					//float varest = depthVar_init;
					float variance = var_invDepthMap1->get(i+1,j+1);//+depthVar_init*varest;
					len_id_search = 4*sqrt(variance);
				
				
					x1.set(j+1,2,1);
					x1offset = x1+offset;								
					projx1 = (*K)*((*R)*(dist*((*invK)*x1offset)))+(Kt);
				
					Mat<float> KRKinvXPrior(projx1);
				
					homogeneousNormalization(&projx1);
					projx1 -= offset;
								
					float photoDist = 0.0f;
					float minphotoDist = (float)255;
	#ifdef saliency				
					//-------------------------------------
					//		WITH SALIENCY
					//-------------------------------------
					float photoRef = f1->get(&x1).get(1,1);
					//--------------------------------------
					//--------------------------------------
	#else				
					//-------------------------------------
					//	WITH PATCH			
					//-------------------------------------		
					int dsizePatch = 2;								
					Mat<float> photoRef( extract(f1, ceil(x1.get(1,1))-dsizePatch,ceil(x1.get(2,1))-dsizePatch, ceil(x1.get(1,1))+dsizePatch, ceil(x1.get(2,1))+dsizePatch));
					//-------------------------------------
					//--------------------------------------
	#endif				
				
					//initialization :	
					//XdispMIN = ((float)1.0/numeric_limits<float>::epsilon()) * ( (*invK)*x1offset);
					//XdispMIN = ((float)numeric_limits<float>::epsilon()) * ( (*invK)*x1offset);
					//XdispMIN = ((float)-1)*(*t);
				
					//XdispMIN = ((float)1.0/(1.0f/dist+2*sqrt(variance+depthVar_search))) * ( (*invK)*x1offset);
					//XdispMIN = ((float)1.0/(1.0f/dist+2*sqrt(variance))) * ( (*invK)*x1offset);
					//XdispMIN = ( (*invK)* ( ((float)1.0/(1.0f/dist+2*sqrt(variance))) * x1offset) );
					
					//float minidepth = (float)(1.0f/dist+2*sqrt(variance));
					float minidepth = (float)(1.0f/dist+2*sqrt(variance));
					//Do the assumption that the rotations ares ill detected therefore the nearest point is always not near enough...
					XdispMIN = KRKinvXPrior+minidepth*Kt;
				
					//XdispMIN = ((float)1.0/(1.0f/dist+2*sqrt(variance+depthVar_search))) * (x1offset);
				
					//XdispMAX = ((float)1.0/(1.0f/dist+sqrt(variance+depthVar_search*varest))) * ( (*invK)*x1offset);
					//XdispMAX = ((float)1.0/(1.0f/dist-2*sqrt(variance+depthVar_search))) * ( (*invK)*x1offset);
					//XdispMAX = ((float)1.0/(1.0f/dist-2*sqrt(variance))) * ( (*invK)*x1offset);
					//XdispMAX = ( (*invK)* (((float)1.0/(1.0f/dist-2*sqrt(variance))) * x1offset) );
					float maxidepth = (float)(1.0f/dist-2*sqrt(variance));
					XdispMAX = KRKinvXPrior+maxidepth*Kt;
				
					//XdispMAX = ((float)1.0/(1.0f/dist-2*sqrt(variance+depthVar_search))) * (x1offset);
				
					//3D vector from the current estimation and the camera center of projection.
					//XdispMAX = (*R)*XdispMAX + (*t);
					//XdispMAX = (KRKinv)*XdispMAX + (Kt);	//problem with floating rounding error
					//XdispMIN = (*R)*XdispMIN + (*t);
					//XdispMIN = (KRKinv)*XdispMIN + (Kt);	//problem with floating rounding error
				
					//XdispMAX = (*K)*XdispMAX;
					//XdispMIN = (*K)*XdispMIN;
					homogeneousNormalization(&XdispMIN);
					homogeneousNormalization(&XdispMAX);
					XdispMIN -= offset;
					XdispMAX -= offset;
				
					bool doStereo = true;
					//Epipolar Line computation :
					//XdispMIN =Xlambdanooffset;
					epiline = XdispMAX - XdispMIN;
					//epiline = Xlambdanooffset - XdispMIN;				
					//epiline = XdispMAX - Xlambdanooffset;
					epiLength = norme2(extract(epiline,1,1,2,1));				
				
					/*	
					if(epiLength >= EPI_LENGTH_MAX)
						EPI_LENGTH_MAX;
				
					if(epiLength <	EPI_LENGTH_MIN)
						epiLength = EPI_LENGTH_MIN;
					*/	
					float minlambda = 0;
					lambdamax = epiLength;
					float lambdaStep = epiLength/nbrHyp;
					float lambdaoffset = lambdamax;
				
					//normalization of epiline :
					epiline *= (float)1.0/epiLength;
														
					//offset that is to be added to the computed lambda... see below :				
				
					//check that the whole search space is within Sigma :
				
					int k = 0;

					//--------------------------------------------------------------------------------------				
					//--------------------------------------------------------------------------------------
					//--------------------------------------------------------------------------------------
					//--------------------------------------------------------------------------------------
				
					float epx = epiline.get(1,1);
					float epy = epiline.get(2,1);
				
					// check towards which boundary of Sigma we are heading to, with respecto to each direction.
					float borderoffset = (float)3;
					float boundx = (epx > (float)0 ? h-borderoffset : (float)0+borderoffset);	
					float boundy = (epy > (float)0 ? w-borderoffset : (float)0+borderoffset); 
				
				
					//if( fabs_((boundx-XdispMIN.get(1,1))/epx) > fabs_((boundy-XdispMIN.get(2,1))/epy) )
					if(XdispMAX.get(1,1) < 0+borderoffset || XdispMAX.get(2,1) < 0+borderoffset || XdispMAX.get(1,1) > h+borderoffset || XdispMAX.get(2,1) > w+borderoffset )
					{
						//continue;
						doStereo = false;
						if( fabs_((boundx-projx1.get(1,1))/epx) > fabs_((boundy-projx1.get(2,1))/epy) )
						{
							//it means that the epiline will encounter the y-related border before the x-related one.
							//therefore, the position of the XdispMAX :
							//XdispMAX.set( (boundy-XdispMIN.get(2,1))*epx/epy+XdispMIN.get(1,1), 1,1);
							XdispMAX.set( (boundy-projx1.get(2,1))*epx/epy+projx1.get(1,1), 1,1);
							//XdispMAX.set( (boundy-Xlambdanooffset.get(2,1))*epx/epy+Xlambdanooffset.get(1,1), 1,1);
							XdispMAX.set( boundy, 2,1);
					
						} 
						else
						{
							//it means that the epiline will encounter the x-related border before the y-related one.
							//therefore, the position of the XdispMAX :
							XdispMAX.set( boundx, 1,1);
							//XdispMAX.set( epy/epx*(boundx-XdispMIN.get(1,1))+XdispMIN.get(2,1), 2,1);
							XdispMAX.set( epy/epx*(boundx-projx1.get(1,1))+projx1.get(2,1), 2,1);
							//XdispMAX.set( epy/epx*(boundx-Xlambdanooffset.get(1,1))+Xlambdanooffset.get(2,1), 2,1);
					
						}
					}
				
				
				
					//same thing in order to deduce ce XdispMIN real.
					epx = -epx;
					epy = -epy;
				
					// check towards which boundary of Sigma we are heading to, with respecto to each direction.				
					boundx = (epx > (float)0 ? h-borderoffset : (float)0+borderoffset);	
					boundy = (epy > (float)0 ? w-borderoffset : (float)0+borderoffset); 
					Mat<float> tempdispm(projx1);
					//Mat<float> tempdispm(XdispMIN);
				
					
					if(XdispMIN.get(1,1) < 0+borderoffset || XdispMIN.get(2,1) < 0+borderoffset || XdispMIN.get(1,1) > h+borderoffset || XdispMIN.get(2,1) > w+borderoffset )
					{						
						//doStereo = false;
						if( fabs_((boundx-projx1.get(1,1))/epx) > fabs_((boundy-projx1.get(2,1))/epy) )
						{
							//it means that the epiline will encounter the y-related border before the x-related one.
							//therefore, the position of the XdispMAX :
							XdispMIN.set( (boundy-tempdispm.get(2,1))*epx/epy+tempdispm.get(1,1), 1,1);
							XdispMIN.set( boundy, 2,1);
					
						} 
						else
						{
							//it means that the epiline will encounter the x-related border before the y-related one.
							//therefore, the position of the XdispMAX :
							XdispMIN.set( boundx, 1,1);
							XdispMIN.set( epy/epx*(boundx-tempdispm.get(1,1))+tempdispm.get(2,1), 2,1);
					
						}
					}
				
											
					if(doStereo)
					{
						doStereoMap.set(1,i+1,j+1);
						
						//re-initialization :				
						epiline = XdispMAX-XdispMIN;
						//epiline = XdispMAX-Xlambdanooffset;
						epiLength = norme2(extract(epiline,1,1,2,1));
						lambdaStep = epiLength/nbrHyp;
						//normalization of epiline :
						epiline *= (float)(1.0/epiLength);
						//Xlambda has been reinitialised in the previous loop, for the next loop.
				
						//--------------------------------------------------------------------------------------
						//--------------------------------------------------------------------------------------
						//--------------------------------------------------------------------------------------
						//--------------------------------------------------------------------------------------
										
						//x_2 = XdispMIN;				
						//x_2offset = x_2;			
		#ifdef saliency									
						//--------------------------------------
						//		WITH SALIENCY
						//--------------------------------------
						//minphotoDist = fabs_( photoRef- f2->get(&XdispMAX).get(1,1));
						minphotoDist = fabs_( photoRef- f2->get(&XdispMIN).get(1,1));
						//--------------------------------------
						//---------------------------------------
		#else				
						//-------------------------------------
						//	WITH PATCH			
						//-------------------------------------						
						minphotoDist = pow( sum(sum(photoRef- extract(f2, ceil(XdispMAX.get(1,1))-dsizePatch,ceil(XdispMAX.get(2,1))-dsizePatch, ceil(XdispMAX.get(1,1))+dsizePatch, ceil(XdispMAX.get(2,1))+dsizePatch) )).get(1,1), 2 );
						//-------------------------------------
						//-------------------------------------s
		#endif				
				
						/*
						Mat<float> kernel( (float)1.0/25,5,5);
						int x1i1 = floor(x1.get(1,1))-2;
						int x1j1 = floor(x1.get(2,1))-2;
						int x2i1 = floor(XdispMAX.get(1,1))-2;
						int x2j1 = floor(XdispMAX.get(2,1))-2;
						minphotoDist = sum(sum( kernel % absM(extract(f1, x1i1, x1j1, x1i1+4, x1j1+4) - extract(f2, x2i1, x2j1, x2i1+4, x2j1+4) ) ) ) ;
						*/
						//minphotoDist = Mat<float>((float)255,1,1);
				
						int kmin = 0;
				
				
						for( k=0;k<nbrHyp+1;k++)
						{
							//Compute position in frame 2 :
							//x_2offset = Xlambdanoffset + (k*lambdaStep)*epiline;
					
							//x_2offset = XdispMIN + (k*lambdaStep)*epiline;
					
							x_2offset.set( XdispMIN.get(1,1) + (k*lambdaStep)*epiline.get(1,1), 1,1);
							x_2offset.set( XdispMIN.get(2,1) + (k*lambdaStep)*epiline.get(2,1), 2,1);
							//x_2offset.set( Xlambdanooffset.get(1,1) + (k*lambdaStep)*epiline.get(1,1), 1,1);
							//x_2offset.set( Xlambdanooffset.get(2,1) + (k*lambdaStep)*epiline.get(2,1), 2,1);
					
					
							//-------------------------------------
							//	WITH PATCH			
							//-------------------------------------						
							//x2i1 = floor(x_2offset.get(1,1))-2;
							//x2j1 = floor(x_2offset.get(2,1))-2;
							//photoDist = absM( sum(sum( kernel%(extract(f1, x1i1, x1j1, x1i1+4, x1j1+4) - extract(f2, x2i1, x2j1, x2i1+4, x2j1+4) ) ) ) );
							//photoDist = sum(sum( kernel % absM(extract(f1, x1i1, x1j1, x1i1+4, x1j1+4) - extract(f2, x2i1, x2j1, x2i1+4, x2j1+4) ) ) ) ;
							//--------------------------------------
							//--------------------------------------
		#ifdef saliency					
							//----------------------------------------
							//WITH SALIENCY
							//----------------------------------------
							photoDist = fabs_( photoRef-f2->get(&x_2offset).get(1,1) );
							//-----------------------------------------
							//------------------------------------------
		#else					
							//------------------------------------------
							// 	WITH PATCH
							//--------------------------------------------
							photoDist = pow( sum(sum(photoRef- extract(f2, ceil(x_2offset.get(1,1))-dsizePatch,ceil(x_2offset.get(2,1))-dsizePatch, ceil(x_2offset.get(1,1))+dsizePatch, ceil(x_2offset.get(2,1))+dsizePatch) )).get(1,1), 2 );
							//--------------------------------------------
							//--------------------------------------------
		#endif					
					
		#ifdef debugDepth						
							//x_2offset.afficher();
							//cout << " Hyp : " << k << " ; photoDist = " << photoDist << " min = " << minphotoDist << endl;
							//cv::circle(debugIm,cv::Point(x_2offset.get(2,1),x_2offset.get(1,1)),(int)2, cv::Scalar(255,0,255));
							//afficher(&debugIm,NULL,NULL,false, factor);
		#endif					

							if(photoDist <= minphotoDist)
							{
								minlambda = k*lambdaStep;
								minphotoDist = photoDist;
								x2offsetmin = x_2offset;
								kmin = k;
							}

						}
				
						/*
						//NO LONGER USEFUL...
						if((x2offsetmin.get(1,1) < (float)0) || (x2offsetmin.get(1,1) > h) || (x2offsetmin.get(2,1) < (float)0)  || (x2offsetmin.get(2,1) > w) )
						{
							//value not good cause the point is not within Sigma :
							x2offsetmin.set( i+2,1,1);
							x2offsetmin.set( j+2,2,1);
							minlambda = 1;
						}
						*/
				
						/*
						if(draw ==0)	
							afficher(&debugIm,NULL,NULL,false, factor);
						*/
		#ifdef debugDepth					
						//cout << i << " " << j << " min = " << minlambda << " pour k = " << kmin << " / " << nbrHyp << endl;
				
						//(Xlambda+minlambda*epiline).afficher();				
				
				
						if(draw == 0)
						{
							Mat<float> Xdmin(XdispMAX);
							Mat<float> Xdmax(XdispMIN); //not equal to XdispMIN and MAX....???				
							//Mat<float> Xdmax(Xlambdanooffset); //not equal to XdispMIN and MAX....???				
							cv::line(debugIm,						     
										 cv::Point(Xdmin.get(2,1), Xdmin.get(1,1)),
										 cv::Point(Xdmax.get(2,1), Xdmax.get(1,1)),
										 cv::Scalar(0,0,255));
							cv::circle(debugIm,cv::Point(x2offsetmin.get(2,1),x2offsetmin.get(1,1)),(int)3, cv::Scalar(255,0,0));
							//cv::circle(debugIm,cv::Point(Xdmin.get(2,1),Xdmin.get(1,1)),(int)2, cv::Scalar(255,255,0));
							//cv::circle(debugIm,cv::Point(Xdmax.get(2,1),Xdmax.get(1,1)),(int)2, cv::Scalar(255,255,0));
							//XdispMAX.afficher();				
							//epiline.afficher();
							//afficher(&debugIm,NULL,NULL,false, factor);
							draw++;
						}
						else if( draw == 20)
							draw = 0;
						else
							draw++;
		#endif
				
				
						if(isnan(minphotoDist) )
						{
							cout << "element : " << count << " nan value... " << endl;
							//x2offsetmin.afficher();
							//x1.afficher();	
							doStereo = false;
						}
						meanPhotoDist += minphotoDist;
				
						if( doStereo && minphotoDist < 1.0f && minphotoDist > 0.0f )//&& epiLength <= 0.5f)
						{
							dostereoCount++;
							//Regularize with respect to the offset the value of lambda disparity :
							//minlambda += lambdaoffset;
						
							//Linearization refinement :
							//minlambda += sqrt(minphotoDist)/(localgrad != (float)0 ? localgrad : (float)1);//(f2->get(x2offsetmin.get(1,1)-2, x2offsetmin.get(2,1))-f2->get(x2offsetmin.get(1,1)-2, x2offsetmin.get(2,1)) );
							//extract the value of the depth from the disparity :
						
							/*Method 1 : usual case if there was a simple translation :*/
							/*	
							float baseline = norme2(*t);
							//float idepth = minlambda/(baseline*sqrt(K->get(1,1) * K->get(2,2))); 
							float idepth = norme2(extract(x1-x2offsetmin,1,1,2,1))/(baseline*(K->get(1,1) + K->get(2,2))/2); 
							*/
							/*Method 11 : usual case if there was a simple translation :*/
							/*	
							float baseline = norme2(*t);
							//float idepth = minlambda/(baseline*sqrt(K->get(1,1) * K->get(2,2))); 
							float idepth = (sum((x1-x2offsetmin)%grad).get(1,1)/ sum(grad % x2offsetmin).get(1,1) )/(baseline*(K->get(1,1) + K->get(2,2))/2); 
							*/
							//-----------------------------------------------------------
							/*Method 2 : with respect to the fact that there is a rotation : */
							/*
							Mat<float> KinvP((*invK)*(x1offset));
							float idepth = 0;
						
							if(epiline.get(1,1)*epiline.get(1,1) > epiline.get(2,1)*epiline.get(2,1))
							{
								float oldX = invK->get(1,1)*(x2offsetmin.get(1,1)+offset.get(1,1))+invK->get(1,3);
								float nominator = (oldX*t->get(3,1) - t->get(1,1));
								float dot1 = KinvP.get(1,1)*R->get(1,1)+KinvP.get(2,1)*R->get(1,2)+KinvP.get(3,1)*R->get(1,3);
								float dot3 = KinvP.get(1,1)*R->get(3,1)+KinvP.get(2,1)*R->get(3,2)+KinvP.get(3,1)*R->get(3,3);

								idepth = (dot1 - oldX*dot3) / nominator;
							}
							else
							{
								float oldY = invK->get(2,2)*(x2offsetmin.get(2,1)+offset.get(2,1))+invK->get(2,3);
								float nominator = (oldY*t->get(3,1) - t->get(2,1));
								float dot2 = KinvP.get(1,1)*R->get(2,1)+KinvP.get(2,1)*R->get(2,2)+KinvP.get(3,1)*R->get(2,3);
								float dot3 = KinvP.get(1,1)*R->get(3,1)+KinvP.get(2,1)*R->get(3,2)+KinvP.get(3,1)*R->get(3,3);

								idepth = (dot2 - oldY*dot3) / nominator;
							}
							*/
							//---------------------------------------------------------------------
						
							/*Method 3 : with respect to the fact that there is a rotation version 2: */
							/*
							Mat<float> KinvP((*invK)*(x1offset));
							float idepth = 0;
						
							if(epiline.get(1,1)*epiline.get(1,1) > epiline.get(2,1)*epiline.get(2,1))
							{
								float oldX = invK->get(1,1)*(x2offsetmin.get(1,1)+offset.get(1,1))+invK->get(1,3);
							float nominator = (oldX*t->get(3,1) - t->get(1,1));
							float dot2 = KinvP.get(1,1)*R->get(2,1)+KinvP.get(2,1)*R->get(2,2)+KinvP.get(3,1)*R->get(2,3);
							float dot3 = KinvP.get(1,1)*R->get(3,1)+KinvP.get(2,1)*R->get(3,2)+KinvP.get(3,1)*R->get(3,3);
							//KinvP = (*R)*KinvP;

							idepth = (dot2*t->get(3,1) - dot3*t->get(2,1)) / nominator;
							}
							else
							{
								float oldY = invK->get(2,2)*(x2offsetmin.get(2,1)+offset.get(2,1))+invK->get(2,3);
								float nominator = (oldY*t->get(3,1) - t->get(2,1));
								float dot1 = KinvP.get(1,1)*R->get(1,1)+KinvP.get(2,1)*R->get(1,2)+KinvP.get(3,1)*R->get(1,3);
								float dot3 = KinvP.get(1,1)*R->get(3,1)+KinvP.get(2,1)*R->get(3,2)+KinvP.get(3,1)*R->get(3,3);
								//KinvP = (*R)*KinvP;

								idepth = (dot3*t->get(1,1) - dot1*t->get(3,1)) / nominator;
							}
						
							*/
							//---------------------------------------------------------------------
						
							/*Method 4 : translation really :*/
							/*
							float baseline = norme2(*t);
							Mat<float> projTrans((*K)*(transpose(*R)*((*invK)*(x2offsetmin+offset))));
							homogeneousNormalization(&projTrans);
							projTrans -= offset;
							float idepth = norme2(x1-projTrans)/(baseline*sqrt(K->get(1,1) * K->get(2,2))); 		    		
							*/
							//---------------------------------------------------------------------
							/*Method 5 : 3D euclidian reconstruction : */
										
							float idepth = 0;
							//Mat<float> temp(KRKinv*x1offset);
							Mat<float> temp( (*K)* ( (*R)* ((*invK)*x1offset) ) );
							homogeneousNormalization(&temp);
							temp -= offset;
						
							if(epiline.get(1,1)*epiline.get(1,1) < epiline.get(2,1)*epiline.get(2,1))
							{
								float newX = x2offsetmin.get(1,1);
								float nominator = newX*t->get(3,1)-Kt.get(1,1);
								float denominator = temp.get(1,1)-temp.get(3,1)*newX;

								idepth = nominator/denominator;
							}
							else
							{
								float newY = x2offsetmin.get(2,1);
								float nominator = newY*t->get(3,1)-Kt.get(2,1);
								float denominator = temp.get(2,1)-temp.get(3,1)*newY;

								idepth = nominator/denominator;
							}
						
						
							//--------------------------------------------------------------------
							/*
							cout << " id = " << idepth << endl;
							transpose(x1).afficher();
							transpose(temp).afficher();
							transpose(x2offsetmin).afficher();
							*/
							//cout << " disparity = " << norme2(x1-x2offsetmin) << endl;
							disparityMap.set( norme2(x1-x2offsetmin), x1.get(1,1),x1.get(2,1));
							//---------------------------------------------------------------------
							/*Method ?+2 : Moyenne des deux estimateurs :*/
						
							float baseline2 = norme2(*t);
							float idepth2 = norme2(extract(x1-x2offsetmin,1,1,2,1))/(baseline2*(K->get(1,1) + K->get(2,2))/2); 
						
							float alpha = 1.0f;
							idepth = alpha*idepth+(1-alpha)*idepth2;
						
							//----------------------------------------------------------
										
							if(isnan(idepth))
							{
								idepth = (float)1.0/numeric_limits<float>::epsilon();
								//continue;
								doStereo = false;
							}
							if( idepth <=(float)0)
							{
								idepth = numeric_limits<float>::epsilon();
								//continue;
								doStereo = false;
							}
							//test for 'inf':
							if( idepth >=(float)1.0f/numeric_limits<float>::epsilon())
							{
								idepth = 1.0f/numeric_limits<float>::epsilon();
								//continue;
								doStereo = false;
							}
						
							//-------------DISPARITY -------------------
						    	//disparityMap.set( norme2(x2offsetmin-x1), i+1,j+1);
						
							//ii) exhaustive search of intensity match
							//iii) depth update with translation along the optical axis (z)
							if(doStereo)
							{
								doStereoMap.set(1, i+1,j+1);
								
								
								float estimatedDepth = (float)idepth; //(lambda != 0 ? lambda : numeric_limits<float>::epsilon());
								float predictedDepth = (float)1.0/dist; //(dist == 0.0f ? dist : numeric_limits<float>::epsilon());//(dist + t->get(3,1) );

								//iv) covariance update :
								float prodgepi = fabs_( (transpose(grad)*epiline).get(1,1));
								float var_geo = var_epinoise/(prodgepi!=0.0f?prodgepi: 0.01f);
				
								//gradient along the epipolar line :
								float g_epi = fabs_(f2->get(XdispMAX.get(1,1),XdispMAX.get(2,1))-f2->get(XdispMIN.get(1,1),XdispMIN.get(2,1))); //pow((f2->get(XdispMAX.get(1,1),XdispMAX.get(2,1))-f2->get(XdispMIN.get(1,1),XdispMIN.get(2,1))), 2);
								//---------------------------------
								//float var_photo = 2*var_Inoise/(g_epi != 0.0f ? g_epi : 0.0001f);
								float photoDistNormalizer = 10.0f;
								float var_photo = 4*var_Inoise*(minphotoDist*photoDistNormalizer)/(g_epi != 0.0f ? g_epi : 1.0f);
				
								//varest = (epiLength/abs(nbrHypSearch))/(len_id_search)*(var_geo+var_photo); 
								varest = ((len_id_search)/abs(nbrHypSearch))*(var_geo+var_photo); 
				
								//TODO : test :
								//if(minphotoDist <= 1.0f)
								//	varest = variance;
								if(varest <= 0.0f)	varest = depthVar_init;
								if(isnan(varest))	varest = depthVar_init;
				
								float varpred = variance;
				
								//varest /= baseline2*f;
								//varest *= (minphotoDist>0?minphotoDist:numeric_limits<float>::epsilon()); 
								//float varOFFSET = abs(predictedDepth-estimatedDepth);
								//TODO : estimate the initialization variance for each estimation step ?
								var_invDepthMap1->set(  varest, i+1, j+1 );
								float new_var = (varpred*varest)/(varpred+varest);
								float min_var = 0.000001f;
								if(new_var < min_var)
									new_var = min_var;
				
								//var_invDepthMap1.set(  (varOFFSET*var_invDepthMap1.get(x1.get(1,1), x1.get(2,1)))/(varOFFSET+var_invDepthMap1.get(x1.get(1,1), x1.get(2,1))), x1.get(1,1), x1.get(2,1) );

								//fusion of the prior normal distribution variance and the estimation variance constant initialization :

								//predictedDepth = predictedDepth/(varpred+varest);
								//estimatedDepth = estimatedDepth/(varpred+varest);
								//invDepthMap2->set( varest*estimatedDepth + varpred*predictedDepth, i+1, j+1 );
								float new_id = new_var*(estimatedDepth/varest + predictedDepth/varpred);					
				
								#ifdef debug_lvl1
								if(draw == 0)
								{
								cout << " element : " << count << " photoDist : " << minphotoDist << endl;
								cout << " element : " << count << " epilength : " << len_id_search << endl;
								cout << " element : " << count << " length epi search : " << epiLength << endl;
								cout << " element : " << count << " prodgepi : " << prodgepi << " varepinoise : " << var_epinoise << endl;
								cout << " element : " << count << " g_epi: " << g_epi << " varInoise : " << var_Inoise << endl;
								cout << " element : " << count << " vargeo : " << var_geo << " varphoto : " << var_photo << endl;
								cout << " element : " << count << " idest : " << idepth << " predid : " << predictedDepth << endl;
								cout << " element : " << count << " varest : " << varest << " varpred : " << varpred << endl;
								cout << " element : " << count << " last var : " << varpred << " new var : " << new_var << endl;
								cout << " element : " << count << " last id  : " << invDepthMap2->get(i+1,j+1) << " newid : " << new_id << endl  << endl;
								}
								#endif
				
								invDepthMap2->set( (isnan(new_id)?0.01f:new_id), i+1, j+1 );
								invDepthMap1->set( (isnan(new_id)?0.01f:new_id), i+1, j+1 );
								
								if(new_var < varpred)
								{
									var_invDepthMap2->set(  new_var, i+1, j+1 );
								}
								//invDepthMap1->set( minphotoDist, i+1, j+1 );
				
								idmean += new_id;
							}
							else
							{
								doStereoMap.set(0,i+1,j+1);
							}
						}
						else
						{
							doStereoMap.set(0,i+1,j+1);
						}
					}
					
					if(!doStereo)
					{
						//var_invDepthMap2->set( var_invDepthMap2->get(i+1,j+1)*2.1f, i+1,j+1);
						//too much if we do not do enough sampling along the epipolar line
						var_invDepthMap2->set( var_invDepthMap2->get(i+1,j+1)*1.1f, i+1,j+1);
						//var_invDepthMap2->set( 0.0f, i+1,j+1);
						//invDepthMap2->set(0.0f, i+1,j+1);
			
						//new random initialization :
						//invDepthMap2->set( 1.0f/(1.0f+fabs_(idmean)), i+1,j+1);
						
						float validity = validityCheck->get(i+1,j+1);
						validityCheck->set( validity-1.0f, i+1,j+1);
						
						if(validity-1.0f <= treshBlacklist)
						{
							//blacklist this pixel :
							blacklisted->set(1,i+1,j+1);
						}
					}
					else
					{
						float validity = validityCheck->get(i+1,j+1);
						validityCheck->set( validity+1.0f, i+1,j+1);
					}
			
				}
			}
		}	
	
		idmean /=count;
		meanPhotoDist /= count;
		cout << "MEAN idepth = " << idmean << endl; 
		cout << "MEAN photoDist = " << meanPhotoDist << endl; 
		cout << "DoStereo : COUNT : " << dostereoCount << " / " << count << endl;
	
	
	#ifdef debugDepth	
		afficher(&debugIm,NULL,NULL,true, factor);
	#endif
		displayInverseCodedFrame(string("ESTIMATED DEPTH MAP"), (Mat<float>*)f1, invDepthMap1, true, factor);
		displayInverseCodedFrame(string("ESTIMATED VARIANCE"), (Mat<float>*)f1, var_invDepthMap1, true, factor); 
		//displayInverseCodedFrame(string("DISPARITY MAP"), (Mat<float>*)f1, &disparityMap, true, factor);
	
		//*invDepthMap2 = *invDepthMap1;//+*invDepthMap2;	
		
		//SMOOTHER : 
	#ifdef SMOOTHER
	
	
    	Mat<float> kernel((float)1.0/9,3,3);
    	Mat<float> k(kernel);
    	
    	int size = 10;
     	for(int i=invDepthMap2->getLine();i--;)
		{
			x1.set(i+1,1,1);

			for(int j=invDepthMap2->getColumn();j--;)
			{

				if(true )//|| invDepthMap2->get(i+1,j+1) != (float)0.0 )
				{
					//float prc =0.995f;
					float idepth = invDepthMap2->get(i+1,j+1);
					float mostTrustableId = idepth;
					float min_var = var_invDepthMap2->get(i+1,j+1);
					if(min_var == (float)0)
						min_var = 1.0f;
					float max_var = min_var;
					float ect = sqrt(min_var);
					//float idlocalmean = sum(sum( k % extract(invDepthMap1, i,j,i+2,j+2)  )).get(1,1);
					float idlocalmean = 0.0f;
					float idlocalmeanBiased = 0.0f;
					int counterUnbiasedlocalmean = 0;
					float denom = 0.0f;
					int counterPixelUsed = 0;
					
					if(invDepthMap2->get(i+1,j+1) != (float)0.0 )
						counterPixelUsed++;

					for(int ii=size;ii--;)
					{
						for(int jj=size;jj--;)
						{
							float id_iijj = invDepthMap1->get((i-size/2)+ii,(j-size/2)+jj);							
							
							if(id_iijj > (float)0)
							{
								idlocalmeanBiased += id_iijj;
								counterUnbiasedlocalmean++;
								float var_iijj = var_invDepthMap2->get((i-size/2)+ii,(j-size/2)+jj);
								if(var_iijj == (float)0)
									continue;//var_iijj == 100.0f;
									
								if(id_iijj <= idepth+2*ect && id_iijj >= idepth-2*ect)
								{
									idlocalmean += id_iijj/var_iijj;							
									denom += 1.0f/var_iijj;
									
									if(var_iijj<min_var)
									{
										min_var = var_iijj;
										mostTrustableId = id_iijj;
									}
									if(var_iijj>max_var)
										max_var = var_iijj;
										
									counterPixelUsed++;
								}
							}
						}
					}
					
					if(idepth != (float)0 && counterPixelUsed >= size*size/2/*size/2*/+1 && denom != 0.0f && doStereoMap.get(i+1,j+1) )//&& idlocalmean != 0.0f)	//obviously since it uses the point (i,j) that it is to be within the range settled...
					{
						invDepthMap2->set(  (2*idepth+idlocalmean/denom)/3 , i+1, j+1);
						
						//invDepthMap2->set(mostTrustableId,i+1,j+1);
						//var_invDepthMap2->set( min_var, i+1,j+1);
						
						//we update the variance with the min of the local value.
						//var_invDepthMap2->set( min_var, i+1,j+1);
						//var_invDepthMap2->set( max_var, i+1,j+1);
						//var_invDepthMap2->set( max_var*min_var/(max_var+min_var), i+1,j+1);
						//var_invDepthMap2->set( (max_var+min_var)/2, i+1,j+1);
						
					}
					else
					{
						
						//else it is a value that can be recruited or it is flawed
						if(idepth != (float)0)
						{
							//then it was a flawed value, let us reassigned it :
							//invDepthMap2->set(idmean,i+1,j+1);
							
							/*
							if(idlocalmeanBiased !=(float)0)
								invDepthMap2->set(idlocalmeanBiased/counterUnbiasedlocalmean,i+1,j+1);
							*/
							/*
							if(false && idlocalmeanBiased !=(float)0)
								invDepthMap2->set( idlocalmeanBiased/counterUnbiasedlocalmean+gen.dev(), i+1,j+1);
							else
								invDepthMap2->set( fabs_(gen.dev()), i+1,j+1);
							*/
							//var_invDepthMap2->set( max_var, i+1,j+1);
							
							invDepthMap2->set((2*mostTrustableId+idmean)/3,i+1,j+1);
							//var_invDepthMap2->set( min_var, i+1,j+1);
						}
						//else it is a value that could be recruited :
						/*
						//let us recrut this point :
						else if(counterPixelUsed >=size/2+1 )
						{
							invDepthMap2->set(idlocalmean/denom,i+1,j+1);
							var_invDepthMap2->set( max_var, i+1,j+1);
						}
						*/	
						
					}
					//invDepthMap2->set(  (prc*idepth+(1-prc)*idlocalmean) , i+1, j+1);
				}
				/*else 
				{
					float idlocalmean = sum(sum( k % extract(invDepthMap1, i,j,i+2,j+2)  )).get(1,1);
					invDepthMap2->set( idlocalmean, i+1,j+1);
				}*/
	
			}	
		}	
#endif	

#ifdef	RESCALER
		*invDepthMap2 *= (float)(1.0f/idmean);
#endif
	
	}

	return count;
}







Mat<float> debugPose(Frame<float>* f1, Frame<float>* f2, const Mat<float>& dm1, const Mat<float>& R, const Mat<float>& t, const Mat<float>& K, const Mat<float>& invK, float factor)
{
    int h = f1->getLine();
    int w = f1->getColumn();
    int offsetx = K.get(1,3);
    int offsety = K.get(2,3);
    Mat<float> offset(3,1);
    offset.set( (float)offsetx, 1,1);
    offset.set( (float)offsety, 2,1);
    offset.set( (float)0, 3,1);
    
    Mat<float> pos(3,1);
    Mat<float> pos1(3,1);
    pos1.set((float)1.0,3,1);
    Mat<float> im((float)0,h,w);

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

                pos = ((float)1.0/dm1.get(i+1,j+1))*pos;
                pos.set( ((float)1.0/dm1.get(i+1,j+1)), 3,1);

                //cout << " Renvoye vers : " << endl;
                //pos.afficher();

                pos = (R*pos+t);

                //float norme = norme2(pos);
                //pos = ((float)1.0/norme)*pos;
                

                pos = K*pos;
                //pos.mat[0][0] =  K.get(1,1)*pos.get(1,1)/pos.get(3,1) ;
	        //pos.mat[0][1] = K.get(2,2)*pos.get(2,1)/pos.get(3,1) ;
                
                //homogeneousNormalization(&pos);
                if(pos.get(3,1) != (float)0.0 && pos.get(3,1) != (float)1.0)
                    pos = ((float)1.0/pos.get(3,1))*pos;
                    
                //pos1 -= offset;                
                pos -= ((float)1)*offset;
                //pos.afficher();
                //pos1.afficher();
              
                //cout << "////////////////////////////////" << endl;

		//float value = (f1->get(pos1)).get(1,1);
		//cout << value << " " << 10*h/2+pos.get(1,1) << " " << 10*w/2+pos.get(2,1) << endl;
                //im.set( (f1->get(pos1)).get(1,1), 10*h/2+pos.get(1,1), 10*w/2+pos.get(2,1) );
                im.set( (f1->get(pos1)).get(1,1), pos.get(1,1), pos.get(2,1) );
                
                
            }
        }

    }


    Mat<float> imDiff((*f2)-im);
    afficherMat( string("debug ref2, f2, imDiff"), &im, f2, &imDiff,true, factor*2);
    //afficherMat( string("debug ref2"), &im, (Mat<float>*)NULL, (Mat<float>*)NULL,true, factor/10);
    
    return dm1-im;

}




template<typename T>
void regularizePropagatedDepthMap( Frame<T>* f2Grad, int gradIntTreshold, Mat<T>* invdm, Mat<T>* var_invdm, T depthvar_init)
{
	
	for(int i=invdm->getLine();i--;)
	{		
		
		for(int j=invdm->getColumn();j--;)
		{
			if(invdm->mat[i][j] != (T)0 && f2Grad->mat[i][j] < gradIntTreshold )
			{
				invdm->set((T)0,i+1,j+1);
				var_invdm->set((T)0,i+1,j+1);
			}
			else if( invdm->mat[i][j] != (T)0)
			{
				var_invdm->set(depthvar_init*10, i+1,j+1);
			}
			/*else if( f2Grad->mat[i][j] > gradIntTreshold)
			{
				var_invdm->set( mean(extract(invdm,i,j,i+2,j+2)), i+1,j+1);
			}*/
			
			if(invdm->mat[i][j] == (T)0)
			{
				var_invdm->set((T)0,i+1,j+1);
			}
		}
	}


}











template<typename T>
Mat<T> propagateDepthMap(Frame<float>* frame, float gradIntTreshold, float variance, float mean, Mat<T>* K, Mat<T>* invK, Mat<T>* deltaPose, Mat<T>* invdm)
{
	Mat<float> offset(3,1);
	offset.set( K->get(1,3), 1,1);
	offset.set( K->get(2,3), 2,1);	
	offset.set( (float)0, 3,1);
	float w = invdm->getLine();
	float h = invdm->getColumn();
	
	Mat<T> PI0((T)0,3,4);
	for(int i=3;i--;)	PI0.set((T)1,i+1,i+1);	
	
	Mat<T> rinvdm((T)0,invdm->getLine(),invdm->getColumn());
	Mat<T> x((T)1,3,1),xoffset(x);
	Mat<T> rX(4,1),rx(3,1),rxoffset(rx);

	//Initialization of the new points :
	if(gradIntTreshold >= 255)
    	gradIntTreshold = 255;
    else if(gradIntTreshold < 0)
    	gradIntTreshold = 0;
    	 
	NormalRand gen(0,variance,(long)10);


	for(int i=rinvdm.getLine();i--;)
	{
		x.set(i+1,1,1);
	
		for(int j=rinvdm.getColumn();j--;)
		{
			x.set(j+1,2,1);
		
			if(invdm->get(i+1,j+1) != (T)0)
			{				
				xoffset = x+offset;
				T depth = ((T)1.0/invdm->get(i+1,j+1));
				
				//Rigid body motion :
				rX = operatorC( depth*((*invK)*xoffset), Mat<T>((T)1,1,1) );
				rX = (*deltaPose)*rX;			
											
				/*set the depth of the correct point in the new KF :*/
				rx = (*K)*(PI0*rX);
				//set the correct depth to each point of the new KF according the frame variation ://
			
				homogeneousNormalization(&rx);
				rxoffset = rx-offset;
			
				float new_id = 1.0f/(depth-deltaPose->get(3,4));
				
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
					rinvdm.set( (float)mean+(float)gen.dev(), i+1,j+1);
				}
			}
		
			else if( fabs_(frame->get(i+1,j+1)) >= (float)gradIntTreshold)
			{
				//inverse depth value is assigned only if the gradient is sufficient.
			
				//TODO : discussion as to know whether the point is a new point or if it is already pictured ine the last frame and if the point in the last frame and this one are really backprojecting into the same point or if it is an appearing point due to occlusion...
				// Meanwhile, we randomized those and put our faith in the ones that do not have such problem...
				rinvdm.set( (float)mean+(float)gen.dev(), i+1,j+1);
				//initialized inverse depth between -0.9 and 1.1.
			}
			
			if( isnan(rinvdm.get(i+1,j+1)))
			{
				rinvdm.set( 1.0f+(float)mean+(float)gen.dev(), i+1,j+1);
			}
		
							
		}
	}
	
	return rinvdm;
}













int main(int argc, char* argv[])
{
	
    
    //--------------------------------------------------------
    //--------------------------------------------------------
    /*
    //EKF : with random accelerations ...
    int nbrstate = 12;
    int nbrcontrol = 0;
    int nbrobs = 6;
    float dt = 1;
    float stdnoise = 1e-25;
    bool ext = false;
    bool filteron = true;
    bool noise = false;

    Mat<float> EKFPose((float)0,nbrstate,1);  
    Mat<float> EKFPoseCovar((float)0,nbrstate,nbrstate);  
    EEKF<float> instanceEKF(nbrstate,nbrcontrol,nbrobs,dt,stdnoise,EKFPose,ext,filteron,noise);
    Mat<float> A((float)0,nbrstate,nbrstate);
    for(int i=1;i<=nbrstate;i++)	A.set((float)1,i,i);
    for(int i=1;i<=nbrstate/2;i++)	A.set((float)dt,i,nbrstate/2+i);
    A.afficher();
    instanceEKF.initA(A);
    Mat<float> C((float)0,nbrobs,nbrstate);
    for(int i=nbrstate/2+1;i<=nbrstate;i++)	C.set((float)1,i-nbrstate/2,i);
    instanceEKF.initC(C);
    //B=0;
    //Mat<float> B((float)0,nbrstate,nbrcontrol);
    //for(int i=nbrstate/2+1;i<=nbrstate;i++)	B.set((float)1,i,i-nbrstate/2);
    //instanceEKF.initB(B);

    Mat<float> Q((float)0,nbrstate,nbrstate);
    for(int i=nbrstate;i--;)	Q.set( (i+1>6 ? stdnoise : stdnoise), i+1, i+1);
    instanceEKF.initQ(Q);
    
    Mat<float> R((float)0,nbrobs,nbrobs);
    for(int i=nbrobs;i--;)	R.set( (i+1>3 ? stdnoise : stdnoise), i+1, i+1);
    instanceEKF.initR(R);
    
    bool EKFuse = false;
    if(argc > 13)
       	if(atoi(argv[13]))
       		EKFuse = true;
    */
    //--------------------------------------------------------
    //--------------------------------------------------------
    //--------------------------------------------------------
    //--------------------------------------------------------
    //EKF : usual ...
    bool cbVOuse = false;
    
    int nbrstate = 12;
    int nbrcontrol = 0;
    int nbrobs = 6;
    float dt = 1;
#define var_huberK
#ifndef var_huberK    
    float stdnoiseobs = 1e-6;
    float stdnoiseobs_lin = 1e-4;
    float stdnoiseobs_ang = 1e-4;
    float stdnoisepro = 1e-4;
    float varxrot = stdnoiseobs_ang;
    float varyrot = stdnoiseobs_ang;
    float varzrot = stdnoiseobs_ang;
    float varx = stdnoiseobs_lin;
    float vary = stdnoiseobs_lin;
    float varz = stdnoiseobs_lin*1e1;
#else    
    float stdnoiseobs = 1e-5;
    float stdnoiseobs_lin = 1e-4;
    float stdnoiseobs_ang = 1e-4;
    float stdnoisepro = 1e-3;
    float varxrot = 3.3e-5;
    float varyrot = 1e-5;
    float varzrot = 3.4e-7;
    float varx = 1.6e-4;
    float vary = 1.9e-4;
    float varz = 7.3e-4;
#endif    
    bool ext = false;
    bool filteron = true;
    bool noise = false;

    Mat<float> EKFPose((float)0,nbrstate,1);  
    Mat<float> EKFPoseCovar((float)0,nbrstate,nbrstate);  
    EEKF<float> instanceEKF(nbrstate,nbrcontrol,nbrobs,dt,stdnoiseobs,EKFPose,ext,filteron,noise,cbVOuse);
    Mat<float> A((float)0,nbrstate,nbrstate);
    for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
    //unstable if we propagate the velocities...
    for(int i=1;i<=nbrstate/2;i++)	A.set((float)dt,i,nbrstate/2+i);
    A.afficher();
    instanceEKF.initA(A);
    Mat<float> C((float)0,nbrobs,nbrstate);
    for(int i=nbrstate/2+1;i<=nbrstate;i++)	C.set((float)1,i-nbrstate/2,i);
    C.afficher();
    instanceEKF.initC(C);
    //B=0;

    Mat<float> Q((float)0,nbrstate,nbrstate);
    for(int i=nbrstate;i--;)	Q.set( (i+1>6 ? stdnoisepro : stdnoisepro), i+1, i+1);
    instanceEKF.initQ(Q);
    
    Mat<float> R((float)0,nbrobs,nbrobs);
    //for(int i=nbrobs;i--;)	R.set( (i+1>3 ? stdnoiseobs_lin : stdnoiseobs_ang), i+1, i+1);
    //R.set( stdnoiseobs_lin*1e1, 3,3);
    R.set( varxrot, 1,1);
    R.set( varyrot, 2,2);
    R.set( varzrot, 3,3);
    R.set( varx, 4,4);
    R.set( vary, 5,5);
    R.set( varz, 6,6);
    
    instanceEKF.initR(R);
    
    bool EKFuse = false;
    if(argc > 13)
       	if(atoi(argv[13]))
       		EKFuse = true;
       		
    Mat<float> statsmean(0.0f, 6,1);
    Mat<float> statsvar(0.0f,6,1);
    int nbrIteration = 0;
    //--------------------------------------------------------
    //--------------------------------------------------------
    
    
    
    float gradIntTreshold = 50;
    if(argc>2)
        gradIntTreshold = atoi(argv[2]);
        
    clock_t timertotal = clock();
    clock_t timerdepth = clock();
    float factor = (float)1.0/4;
    
    if(argc>3)
        factor = (float)1.0/atoi(argv[3]);
        
    int pyramidDepth = 3;
    
    if(argc>4)
    	pyramidDepth = atoi(argv[4]);
    	
    int nbrHypSearch = 10;
    
    if(argc>5)
    	nbrHypSearch = atoi(argv[5]);
    	
    int count = 1;
    float fps = (float)4.0;
    float goalfps = (float)2.0;
    float coeff = (float)1.0/factor/2;
    float fpserr = (float)0.0;
    bool fpsoptimization = false;
    bool debug =true;
    if(argc > 9)
    	if(atoi(argv[9]))
    		debug = true;
    	else
    		debug = false;
    
    bool debugrotonly = false;
    
    Mat<float> khi((float)0,7,1);
    khi.set((float)1,7,1);

    Mat<float> deltaPose( expMSIM3(khi) );
    Mat<float> khiPose(6,1);
    Mat<float> lastKhiPose(0.0f,6,1);
    vector<Mat<float> > pose;
    Mat<float> globalPose(deltaPose);
    Mat<float> globalKhiPose(khiPose);
    deltaPose.afficher();
    cv::Mat frame,frame1,frame2;
    vector<cv::Mat> framelist;
    int nbrFrame = 0;
    vector<cv::Mat> KFlist;
    int nbrKF = 0;
    //bool kframeChange = true;
    bool withMultipleKF = false;
    
    if(argc>6)
    	if(atoi(argv[6]))
    		withMultipleKF = true;
    	else
    		withMultipleKF = false;
    		
    cout << "Usage : " << argv[0] << " mode=12 gradTresh=50 invfactor=4 pyramidDepth=3 nbrHypSearch=10 withMultipleKF=0:false;1:true meanDepth=0.9 initVarDepth=0.5 debug=0:false;1=true TUMBenchmark=0:false-->LiveCamera;1:true DEBUGMODE:FramePerFrame=1:false-->Live;0:true DEBUGMODE:IrrlichtVisualisationOfDepthMAP=1:true;0:false useExtendedKalmanFilterWithVO=1:true;0:false" << endl;		
    int nbrFramePerKF = 5;//ceil(1.0/ceil(fps))*4;
    int countFramePerKF = 0;
    
    cv::Mat debugIm;
    cv::Scalar colorbleue(255,0,0);
    cv::Scalar colorvert(0,255,0);


    //cv::VideoCapture cap(1);
    cv::VideoCapture cap;
    /*
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            return -1;
    }
	*/
    
    cv::namedWindow("Entry");
    //if(debug)
	//cv::namedWindow("DEBUG");    
    //--------------------

    //containers
    bool tum = true;
    /*
    if(argc >10)
    	tum = atoi(argv[10]);
    */	


    bool continuer = true;
    for(int i=0 ; i<= 20; i++)  cap >> frame1;
    cap >> frame1;
    for(int i=0 ; i<= 20; i++)  cap >> frame2;

    if(tum)
    {
#define test1
#ifdef test1
	    frame1 = cv::imread("../data/Images/LSD-SLAM_room/00001.png", CV_LOAD_IMAGE_COLOR);
	    frame2 = cv::imread("../data/Images/LSD-SLAM_room/00002.png", CV_LOAD_IMAGE_COLOR);
#else
	    frame1 = cv::imread("../data/Images/LSD-SLAM_room1/00501.png", CV_LOAD_IMAGE_COLOR);
	    frame2 = cv::imread("../data/Images/LSD-SLAM_room1/00502.png", CV_LOAD_IMAGE_COLOR);
#endif	    
    }
#ifdef test1    
    string path("../data/Images/LSD-SLAM_room/");
#else    
    string path("../data/Images/LSD-SLAM_room1/");
#endif    
    int fcount = 2;
    
    cv::resize(frame1,frame1,cv::Size(0,0),factor,factor);

    framelist.insert(framelist.end(),frame1);
    KFlist.insert(KFlist.end(),frame1);
    nbrFrame++;
    nbrKF++;
    
    Mat<float> ResidualImage((float)0,1,1);
    Frame<float> f1;
    Frame<float> f1GradX, f1GradY, f1Grad;
    cv::Mat f1gradX;
    cv::Mat f1gradY;
    
	f1 = cv2Matp<float>( frame1 );
	f1Grad = cv2Matp<float>( computeGradientXY( &frame1, &f1gradX, &f1gradY) );
	f1GradX = cv2Matp<float>( f1gradX);
	f1GradY = cv2Matp<float>( f1gradY);
    	    
    Frame<float> f2(f1);
    Frame<float> f2GradX, f2GradY, f2Grad;
    cv::Mat f2gradX;
    cv::Mat f2gradY;    
    
    int h = f1.getLine();
    int w = f1.getColumn();


    float meandepth = 0.9;
    float initVariance = 1.0;
    
    if(argc>7)
    	meandepth = atof(argv[7]);
    if(argc>8)
    	initVariance = atof(argv[8]);
    	
    Mat<float> invDepthMap1((float)0,h,w);
   	invDepthMap1 =  initializeInvDM(&f1Grad, gradIntTreshold, initVariance, meandepth);
    	
    Mat<float> invDepthMap2(invDepthMap1);
    float depthVar_init = (float)100.0;
    
    Mat<float> var_invDepthMap1(0.0f,h,w);
    Mat<float> var_invDepthMap2(0.0f,h,w);
    
    for(int i=h;i--;)
    {
    	for(int j=w;j--;)
    	{
    		if(invDepthMap1.get(i+1,j+1) != (float)0)
    		{
    			var_invDepthMap1.set( depthVar_init*initVariance, i+1,j+1);
    			var_invDepthMap2.set( depthVar_init*initVariance, i+1,j+1);
    		}
    	}
    }
    
    Mat<float> validityCheck(0.0f,h,w);
	Mat<int> blacklisted((int)0,(int)h,(int)w);
	float treshBlacklist = -10.0f;

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
    //K.set((float)723.546970*factor,1,1); //fx
    //K.set((float)729.176276*factor,2,2); //fy
    //K.set((float)-h/2,1,3); //cx
    //K.set((float)-w/2,2,3); //cy
    K.set((float)0.39738586545*h,1,1); //fx
    K.set((float)0.78319662809*w,2,2); //fy
    K.set((float)0.41778421402*h,1,3); //cx
    K.set((float)0.48249810536*w,2,3); //cy
    
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
        countFramePerKF++;
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
			path = string("../data/Images/LSD-SLAM_room/");
#else		
			path = string("../data/Images/LSD-SLAM_room1/");
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
        
    
		f2 = cv2Matp<float>( frame2 );
		f2Grad = cv2Matp<float>( computeGradientXY( &frame2, &f2gradX, &f2gradY) );
		f2GradX = cv2Matp<float>( f2gradX);
		f2GradY = cv2Matp<float>( f2gradY);
        
        

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
		   	//cout << s.str() << endl;
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


        //pose = pose*deltaPose;
        pose.insert(pose.end(),deltaPose);        
        
        rot = extract(deltaPose,1,1,3,3);
        invRot = transpose(rot);
        // rot € SO(3) !!!
        if(!debugrotonly)
        	t = extract(deltaPose,1,4,3,4);
        
        
		timerdepth = clock();
	
		float baseline = norme2(t);
		float f = (K.get(1,1)+K.get(2,1))/2;
		
        count = DepthUpdate21(nbrHypSearch, &f1, &f2, &invDepthMap1, &invDepthMap2, &rot, &t, &var_invDepthMap1, &var_invDepthMap2, &K, &invK, 1.0/factor, &validityCheck, &blacklisted, treshBlacklist);
        
        
#ifdef debugOptim
		cout << "L'execution depth-map a prise : " << (float)(clock()-timerdepth)/CLOCKS_PER_SEC << " secondes." << endl;
#endif

		Mat<float> ResidualAfterAlignement(w,h);
	    ResidualAfterAlignement = debugPose( &f1, &f2, invDepthMap2/*depthmap assigned to f1..*/, rot, t, K, invK, coeff);
		

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
	
		if(tum)
			displayInverseCodedFrame(string("VARIANCE OF DEPTH MAP"), (Mat<float>*)&f1, &var_invDepthMap2, displaycontinuer, coeff);		
		else
			displayInverseCodedFrame(string("VARIANCE OF DEPTH MAP"), (Mat<float>*)&f1, &var_invDepthMap2, displaycontinuer, coeff);		
#ifdef debugOptim	
		//cout << "l'execution IDEPTH DISPLAY a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
#endif

        //Depth Map update :
        invDepthMap1 = invDepthMap2;
        var_invDepthMap1 = var_invDepthMap2;

        
        //FPS HANDLING :
        /*
        if(fpsoptimization)
        {
			float P = (float)2.0*fabs_(goalfps-fps);
			float I = (float)0.0;
			if( fps < (float)0.8*goalfps)
			{
				fpserr += goalfps - fps;
				gradIntTreshold+= P*(goalfps-fps)+I*fpserr;
				invDepthMap1 = initializeInvDM(&f1, gradIntTreshold) ;
			
			}
			else if( fps > goalfps*(float)1.2)
			{
				fpserr += goalfps - fps;
				gradIntTreshold+= P*(goalfps-fps)+I*fpserr;
				invDepthMap1 = initializeInvDM(&f1, gradIntTreshold) ;        
			}
		}
		*/
        
        
        //KF Handling :        
        //if( withMultipleKF && countFramePerKF >= nbrFramePerKF )
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
        float val_tresh_move = 20;
        float val = ( transpose(khiPose) *(W * khiPose) ).get(1,1);
        
        cout << "MOVEMENT : tresh = "<< val_tresh_move << " ; val = " << val << endl;
        if( withMultipleKF &&  val >= val_tresh_move)
        {		
        	countFramePerKF = 0;
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

    cap.release();
    cv::destroyAllWindows();
    
    //---------------
    
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
    	
	for(int i=0;i<invDepthMap1.getLine();i++)	
	{
		stringstream s;
		for(int j=0;j<invDepthMap1.getColumn();j++)
		{
			
			s << (float)(1.0/invDepthMap1.get(i+1,j+1)) << ",";
		}
		
		s << endl;
		//cout << s.str();
		fputs( s.str().c_str(), logDEPTH);
	}
	
    
    //------------------------------------
    
    //Fermeture du fichier :
    if(fclose(log) == EOF)
    {
    	cout << "ERROR : cannot close the file." << endl;
    	exit(1);
    }
    else
    	cout << "File closed." << endl;
    	
    //Fermeture du fichier :
    if(fclose(logDEPTH) == EOF)
    {
    	cout << "ERROR : cannot close the file DEPTH." << endl;
    	exit(1);
    }
    else
    	cout << "File closed DEPTH." << endl;


    return 0;
}


