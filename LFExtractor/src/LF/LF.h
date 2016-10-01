#ifndef OPENCV_USE
#define OPENCV_USE
#endif

#include "../MAT/Mat2.h"
using namespace mat;

template<typename T1,typename T2>
T1 po(T1 a, T2 p);
template<typename T>
T RAWmoment(Mat<T> I, int p, int q);
template<typename T>
Mat<T> centroid(Mat<T> I);
template<typename T>
Mat<T> computeSO(double theta, int dimension = 2, int axis = 1);

template<typename T>
class LF 
{
	private :
	
	Mat<T>* ImageR;		/*image to be computed.*/
	Mat<T>* ImageG;
	Mat<T>* ImageB;
	Mat<T>* Image;		/*gray scale image*/
	Mat<T>* Color;
	Mat<T>* grad_Image;	/*computed gradient image of Image*/
	Mat<T>* Landmarks;		/*2xN / N= nbr of potential landmarks.*/
	int tol;
	
	
	Mat<T> gradKernel;
	
	public :
	
	LF()
	{
		Image = new Mat<T>((int)1, (int)1);
		ImageR = new Mat<T>((int)1, (int)1);
		ImageG = new Mat<T>((int)1, (int)1);
		ImageB = new Mat<T>( (int)1, (int)1);
		Landmarks = new Mat<T>((T)0, (int)2, (int)1);
	
		grad_Image = NULL;
		tol = 255/10;
	}

	LF(Mat<T> image_)
	{
		Image = new Mat<T>(image_);
		ImageR = new Mat<T>( (int)1, (int)1);
		ImageG = new Mat<T>( (int)1, (int)1);
		ImageB = new Mat<T>( (int)1, (int)1);
	
		Color = NULL;
		Landmarks = new Mat<T>((T)0, (int)2, (int)1);
	
		grad_Image = NULL;
		tol = 255/10;
	}

	LF(Mat<T> imR, Mat<T> imG, Mat<T> imB)
	{
		Image = new Mat<T>( 1/3*(imR+imG+imB));
		ImageR = new Mat<T>(imR);
		ImageG = new Mat<T>(imG);
		ImageB = new Mat<T>(imB);
	
		Color = NULL;
		Landmarks = new Mat<T>((T)0, (int)2, (int)1);
	
		grad_Image = NULL;
		tol = 255/10;
	}

	
	~LF()
	{
		if(Image != NULL)
			delete Image;
		if(ImageR != NULL)
			delete ImageR;
		if(ImageG != NULL)
			delete ImageG;
		if(Color != NULL)
			delete Color;
		if(ImageB != NULL)
			delete ImageB;
		delete Landmarks;
	
		if(grad_Image != NULL)
			delete grad_Image;
	}	
	
	int setImage(Mat<T> image_)
	{
		if(image_.getColumn() != 0 && image_.getLine() != 0)
		{
			*Image = image_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais parametre image_." << endl;
			return 0;
		}
	}
	
	
	int setImage(Mat<T> imR, Mat<T> imG, Mat<T> imB)
	{

		*ImageR = imR;
		*ImageG = imG;
		*ImageB = imB;
	
		return 1;	
	}
	
	int setColor(Mat<T> color_, int tol_ = 255/10)
	{
		tol = (tol_<=255 && tol_>=0 ? tol_ : 255/10);
	
		if(color_.getColumn() == 3)
		{
			if(Color == NULL)
				Color = new Mat<T>(color_);
			else
				*Color = color_;
			
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais parametre color_." << endl;
			return 0;
		}
	
	}
	
	void resetLM()
	{
		if(Landmarks != NULL)
		{
			delete Landmarks;
            Landmarks = new Mat<T>((T)0, (int)2, (int)1);
		}
	}
	
	
    void computeGradient(int mode = 0)
	{
	
        if(mode >= 1)
		{
			gradKernel = Mat<T>((T)(0), (int)1,(int)3);
            gradKernel.set( (T)(-1.0/mode),(int)1, (int)1);
            gradKernel.set( (T)(+1.0/mode),(int)1, (int)3);
			
            grad_Image = new Mat<T>( absM( correlation(*Image, gradKernel)) + absM( correlation(*Image,(T)(-1)*transpose(gradKernel)) ) );
		
		}
		else
		{	
            gradKernel = Mat<T>((T)(1.0/2), (int)3,(int)3);
		
			for(int i=1;i<=3;i++)
			{
				for(int j=3-i+1;j<=3;j++)
				{
					if(j!=2 || i!=2)
						gradKernel.set((T)(-gradKernel.get(i,j)), i, j);
				}
			}
			gradKernel.set((T)0, 2, 2);

			grad_Image = new Mat<T>( correlation(*Image, gradKernel) );
		}
	}
	
	
	
    void search(int method, int step = 1)
/*method : 
 * 1 :: Great-Gradient variation Detection ; 
 * 2 :: Harris CornerDetector
 * 3 :: colorimetrique 
 * 4 :: FAST
 */
 	{
		resetLM();
	
	
		switch(method)
		{
			case 1:
			{
			computeGradient();
			T moyenne = (T)mean( *grad_Image);

            for(int i=1;i<=grad_Image->getLine();i++)
			{
                for(int j=1;j<=grad_Image->getColumn();j++)
				{
					if(grad_Image->get(i,j) >= (T)(((T)4)*moyenne))
					{
						Mat<T> t( 3, 1);
						t.set((T)i, 1,1);
						t.set((T)j, 2,1);
						t.set((T)Image->get(i,j), 3,1);
			
						*Landmarks = operatorL(*Landmarks, t);
					}
				}
			}
			}
			break;
		
			case 2 :
		
			
		
			break;
		
			case 3:
			{
			
			if(Color != NULL)
			{
				for(int i=1;i<=ImageR->getLine();i++)
				{
					for(int j=1;j<=ImageR->getColumn();j++)
					{
						if( colorMatch( *Color, ImageR->get(i,j), ImageG->get(i,j), ImageB->get(i,j)) )
						{
							Mat<T> t( 3, 1);
							t.set((T)i, 1,1);
							t.set((T)j, 2,1);
							t.set((T)Image->get(i,j), 3,1);
			
							*Landmarks = operatorL(*Landmarks, t);
						}
					}
				}
			}
			else
			{
				cout << "Impossible d'operer la rechercher, on n'a pas de color." << endl;
			}
		
			}
			break;
			
			
			case 4:	/*FAST*/
			{				
                Mat<T> t((T)0, 2, 1);

				for(int i=4;i<=Image->getLine()-3;i+=step)
				{
		                        //cout << i << endl;

		                        for(int j=4;j<=Image->getColumn()-3;j+=step)
					{

						if( check15913( *Image, i,j) )
						{	
				                    //cout << i << " " << j << endl;
				                    //cout << "OKAY !" << endl;

						    t.set((T)i, 1,1);
		  				    t.set((T)j, 2,1);
					            //t.set((T)Image->get(i,j), 3,1);
		
						    *Landmarks = operatorL(*Landmarks, t);
						}
						//else : it cannot be a corner...						
					}
				}
				
			
			}
			break;
		
			default:
		
			break;
		}
	}
 
 	
    inline bool colorMatch( const Mat<T> color, const T r, const T g, const T b)
	{
		bool ret = false;
	
		if( color.get(1,1) <= r + tol && color.get(1,1) >= r - tol)
		{
			if( color.get(2,1) <= g + tol && color.get(2,1) >= g - tol)
			{
				if( color.get(3,1) <= b + tol && color.get(3,1) >= b - tol)
				{
					ret = true;
				}
			}
	
		}
	
		return ret;
	}
	
    inline bool check15913( const Mat<T> im, const int x, const int y)
	{
		bool ret = false;
		int count = 0;
        T treshold = (T)5;
		
		for( int theta=0;theta<=270;theta+=90)
		{
            if( im.get( x+3*cos((double)(PI*(((double)theta)/360))), y+3*sin((double)(PI*(((double)theta)/360))) ) > im.get(x,y)+treshold
                    || im.get( x+3*cos((double)(PI*(((double)theta)/360))), y+3*sin((double)(PI*(((double)theta)/360))) ) < im.get(x,y)-treshold )
				count++;			
			
		}
		
        if( count >= 3)
		{
			ret = true;
			//todo : full segment search...
		}
			
		return ret;
	}
	
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	
	Mat<T> getGrad() const
	{
		return *grad_Image;
	}
	
	
	Mat<T> getLandmarks() const
	{
		return *Landmarks;
	}

	int getNbrL() const
	{
		return Landmarks->getColumn();
	}

	Mat<T> getColor() const
	{
		return *Color;
	}

	
};


template<typename T>
void Draw( Mat<T>* im, int x, int y, T color = (T)128)
{
	im->set( (T)color, x,y);
}

template<typename T>
void DrawCircle( Mat<T>* im, int x0 = 0, int y0 = 0, int r = 3,T color = (T)255)	// r = 3 seems to give a 16 pixel cirlce.
{
	int x = ( r > 0 ? r : -r);
	int y = 0;
	int RE = 1-x;
	
	while( x>=y)
	{
		Draw( im, x+x0, y+y0, color);
		Draw( im, y+x0, x+y0, color);
		Draw( im, -x+x0, y+y0, color);
		Draw( im, -y+x0, x+y0, color);
		
		Draw( im, -x+x0, -y+y0, color);
		Draw( im, -y+x0, -x+y0, color);
		Draw( im, x+x0, -y+y0, color);
		Draw( im, y+x0, -x+y0, color);
		
		y++;
		
		if( RE < 0)
		{
			RE += 2*y +1;
		}
		else
		{
			x--;
			RE += 2*(y-x +1);
		}		
	}
}


//template<typename T>
class SIFT
{
public :
    SIFT(int octave = 4)
    {
        this->octave = octave;
        imPerOct = 5;
        scale = sqrt(2);
        k = sqrt(2);

        for(int i=octave;i--;)
            landmarks.insert(landmarks.begin(), Mat<int>((int)0,3,3));
        features = Mat<int>((int)0,1,1);
    }


    SIFT(cv::Mat Im, int octave = 4)
    {
        cv::cvtColor(Im,this->Im,CV_BGR2GRAY);
        //cv::resize(this->Im,this->Im, cv::Size(0,0),(double)2,(double)2);
        //cv::GaussianBlur(this->Im,this->Im, cv::Size(0,0),1.0/2,1.0/2);
        this->octave = octave;
        imPerOct = 5;
        scale = (double)sqrt(2);
        k = (double)sqrt(2);        

        for(int i=octave;i--;)
        {
            landmarks.insert(landmarks.begin(), Mat<int>(0, (int)((double)this->Im.rows/pow(2,octave-1-i)), (int)((double)this->Im.cols/pow(2,octave-1-i))) );
        }


        features = Mat<int>((int)0, Im.rows,Im.cols);
    }

    ~SIFT()
    {

    }

    void run(int tresh = 255/10, int mode = 0)
    {
        generateImSet();
        generateDoGSet();
        seekMaxMin(mode);
        extractCorner(tresh);
    }

    void generateImSet()
    {
        cv::Mat tempI;
        cv::Size size(0,0);
        vector<cv::Mat> tempV;

        for(int oct=octave;oct--;)
        {            

            double factor = pow(2,octave-1-oct);
            cv::resize(Im,tempI,size,1.0/factor,1.0/factor);


            for(int i=imPerOct;i--;)
            {
                double sigma = pow(k,imPerOct-1-i)*scale;
                cv::Mat tempIG;
                cv::GaussianBlur(tempI,tempIG,size,sigma,sigma);

                tempV.insert(tempV.begin(), tempIG);
            }

            Images.insert(Images.begin(), tempV);

            /*cleaning loop*/
            tempV.clear();
        }

        /*blurring then scaling ? faster ?*/
    }

    void generateDoGSet()
    {
        vector<cv::Mat> tempV;

        for(int oct=octave;oct--;)
        {            
            for(int i=imPerOct-1;i--;)
            {                
                tempV.insert(tempV.begin(), Images[oct][i+1]-Images[oct][i]);
            }

            DoG.insert(DoG.begin(), tempV);

            /*clearing loop*/
            tempV.clear();
        }
    }

    void seekMaxMin(int mode = 0)
    {
        bool minMax = true;
        int offsetI = 1;
        int offsetC = 2;
        uchar* p0;
        uchar* p1;
        uchar* p_1;
        /*so that you can seek through all of the pixels that are not in the edge of the image*/

        switch(mode)
        {

            case 0 :

            for(int oct=octave;oct--;)
            {
                for(int i=(imPerOct-2)-2;i--;)
                {

                    for(int x=DoG[oct][i+offsetI].rows-offsetC;x--;)
                    {
                        p0 = DoG[oct][i+offsetI].ptr<uchar>(x+1);
                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1);
                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1);

                        for(int y=DoG[oct][i+offsetI].cols-offsetC;y--;)
                        {


                            if(p0[y+1]>=p1[y+1] && p0[y+1]>=p_1[y+1])//p0[y+1]>p1[y+1] && p0[y+1]>p_1[y+1])
                            {
                                /*check for Maxima*/

                                for(int xv=-1; xv<=1;xv++)
                                {
                                    if(xv==1)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1+xv);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1+xv);
                                    }
                                    else if(xv==0)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x);
                                    }

                                    if(p0[y+1]>=p1[y] && p0[y+1]>=p_1[y] && p0[y+1]>=p0[y])//p0[y+1]>p1[y] && p0[y+1]>p_1[y] && p0[y+1]>p0[y])
                                    {
                                        if(p0[y+1]<p1[y+2] && p0[y+1]<p_1[y+2] && p0[y+1]<p0[y+2])
                                        {
                                            minMax = false;
                                            xv=2;
                                        }
                                    }
                                    else
                                    {
                                        minMax = false;
                                        xv=2;
                                    }
                                }

                            }
                            else if(p0[y+1]<=p1[y+1] && p0[y+1]<=p_1[y+1])//p0[y+1]<p1[y+1] && p0[y+1]<p_1[y+1])
                            {
                                /*check for Minima*/

                                for(int xv=-1; xv<=1;xv++)
                                {
                                    if(xv==1)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1+xv);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1+xv);
                                    }
                                    else if(xv==0)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x);
                                    }

                                    if(p0[y+1]<=p1[y] && p0[y+1]<=p_1[y] && p0[y+1]<=p0[y])//p0[y+1]<p1[y] && p0[y+1]<p_1[y] && p0[y+1]<p0[y])
                                    {
                                        if(p0[y+1]>p1[y+2] && p0[y+1]>p_1[y+2] && p0[y+1]>p0[y+2])
                                        {
                                            minMax = false;
                                            xv=2;
                                        }
                                        else
                                        {
                                            /*
                                            cv::Mat test;
                                            cv::resize( extractCV( DoG[oct][i+offsetI], x+1, y+1, 3), test, cv::Size(0,0), (double)100, (double)100);
                                            afficher(&test);
                                            */
                                        }
                                    }
                                    else
                                    {
                                        minMax = false;
                                        xv=2;
                                    }
                                }

                            }
                            else
                                minMax = false;

                            if(minMax)
                            {
                                //cout << "MinMax found" << endl;
                                /*register as possible landmark*/

                                /*
                                if(landmarks.getLine() !=4)
                                {
                                    landmarks = Mat<int>(0,4,1);
                                    landmarks.set( x+1, 1,1);
                                    landmarks.set( y+1, 2,1);
                                    landmarks.set( i+offsetI, 3,1);
                                    landmarks.set( oct, 4,1);
                                }
                                else
                                {
                                    Mat<int> temp(0,4,1);
                                    temp.set( x+1, 1,1);
                                    temp.set( y+1, 2,1);
                                    temp.set( i+offsetI, 3,1);
                                    temp.set(oct, 4, 1);

                                    //temp.afficher();

                                    landmarks = operatorL(&landmarks,&temp);

                                }
                                */
                                /*
                                if(landmarks[oct].getLine() != Im.rows)
                                {
                                    landmarks[oct] = Mat<int>(0, DoG[oct][0].rows, DoG[oct][0].cols);
                                }
                                else
                                {
                                */
                                    landmarks[oct].set(oct+1, x+1,y+1);
                                    /*
                                }*/

                            }

                            minMax = true;
                        }
                    }


                }
            }
            break;

        case 1 :
            for(int oct=octave;oct--;)
            {
                for(int i=(imPerOct-2)-2;i--;)
                {

                    for(int x=DoG[oct][i+offsetI].rows-offsetC;x--;)
                    {
                        p0 = DoG[oct][i+offsetI].ptr<uchar>(x+1);
                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1);
                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1);

                        for(int y=DoG[oct][i+offsetI].cols-offsetC;y--;)
                        {


                            if(p0[y+1]>p1[y+1] && p0[y+1]>p_1[y+1])
                            {
                                /*check for Maxima*/

                                for(int xv=-1; xv<=1;xv++)
                                {
                                    if(xv==1)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1+xv);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1+xv);
                                    }
                                    else if(xv==0)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x);
                                    }

                                    if(p0[y+1]>p1[y] && p0[y+1]>p_1[y] && p0[y+1]>p0[y])
                                    {
                                        if(p0[y+1]<=p1[y+2] && p0[y+1]<=p_1[y+2] && p0[y+1]<=p0[y+2])
                                        {
                                            minMax = false;
                                            xv=2;
                                        }
                                    }
                                    else
                                    {
                                        minMax = false;
                                        xv=2;
                                    }
                                }

                            }
                            else if(p0[y+1]<p1[y+1] && p0[y+1]<p_1[y+1])
                            {
                                /*check for Minima*/

                                for(int xv=-1; xv<=1;xv++)
                                {
                                    if(xv==1)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x+1+xv);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x+1+xv);
                                    }
                                    else if(xv==0)
                                    {
                                        p1 = DoG[oct][i+offsetI+1].ptr<uchar>(x);
                                        p_1 = DoG[oct][i+offsetI-1].ptr<uchar>(x);
                                    }

                                    if(p0[y+1]<p1[y] && p0[y+1]<p_1[y] && p0[y+1]<p0[y])
                                    {
                                        if(p0[y+1]>=p1[y+2] && p0[y+1]>=p_1[y+2] && p0[y+1]>=p0[y+2])
                                        {
                                            minMax = false;
                                            xv=2;
                                        }
                                        else
                                        {
                                            /*
                                            cv::Mat test;
                                            cv::resize( extractCV( DoG[oct][i+offsetI], x+1, y+1, 3), test, cv::Size(0,0), (double)100, (double)100);
                                            afficher(&test);
                                            */
                                        }
                                    }
                                    else
                                    {
                                        minMax = false;
                                        xv=2;
                                    }
                                }

                            }
                            else
                                minMax = false;

                            if(minMax)
                            {
                                landmarks[oct].set(oct+1, x+1,y+1);
                            }

                            minMax = true;
                        }
                    }


                }
            }
            break;

        }


    }

    void extractCorner(int tresh = 100)
    {
        /*works on landmarks and compute the gradient everywhere there is a interesting point in order to see if it is a corner or not.*/
        for(int oct=octave;oct--;)
        {
            vector<cv::Mat> grad( computeGradient( DoG[oct][0]) );

            for(int i=landmarks[oct].getLine();i--;)
            {
                for(int j=landmarks[oct].getColumn();j--;)
                {
                    if(landmarks[oct].get(i+1,j+1) != 0)
                    {
                        /*check for a corner*/
                        //vector<cv::Mat> grad( computeGradient( extractCV(DoG[oct][0], i-1, j-1, 3) ) );

                        if(grad[0].at<uchar>(i,j) >= tresh && grad[1].at<uchar>(i,j) >= tresh)
                        //if(grad[0].at<uchar>(1,1) >= tresh && grad[1].at<uchar>(1,1) >= tresh)
                        {
                            /*corner spotted*/
                            features.set((int)255, i*pow(2,4-oct-1), j*pow(2,4-oct-1) );
                        }
                    }
                }
            }
        }
    }

    static vector<cv::Mat> computeGradient(cv::Mat im)
    {
        int ddepth = CV_16S;
        int scale = 1;
        int delta = 0;

        if(im.channels() != 1)
            cv::cvtColor(im,im,CV_BGR2GRAY);
        //cv::GaussianBlur(im,im,cv::Size(0,0),(double)1.0/sqrt(2),(double)1.0/sqrt(2));

        vector<cv::Mat> grad;
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im.rows,im.cols), im.type()) );
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im.rows,im.cols), im.type()) );

        // Gradient X
        Sobel( im, grad[0], ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[0], grad[0] );

        // Gradient Y
        Sobel( im, grad[1], ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[1], grad[1] );

        return grad;
    }

    /*ACCESSEURS*/
    inline vector<Mat<int> > getLandmarks() const
    {
        return landmarks;
    }

    inline Mat<int> getFeatures()  const
    {
        return features;
    }


protected :
    int octave;
    int imPerOct;
    double scale;
    double k;
    cv::Mat Im;
    vector<vector<cv::Mat> > Images;
    vector<vector<cv::Mat> > DoG;
    vector<Mat<int> > landmarks;
    Mat<int> features;

};

class FAST
{
public :
    FAST()
    {
        features = Mat<int>(0,1,1);
    }


    FAST(cv::Mat Im)
    {
        cv::cvtColor(Im,this->Im,CV_BGR2GRAY);
        gradients = computeGradient(this->Im);
        //this->Im = gradients[0];
        //this->Im += gradients[1];

        //afficher(&gradients[0], &gradients[1],&(this->Im),false, 10);
        //cv::GaussianBlur(this->Im,this->Im, cv::Size(0,0),1.0/4,1.0/4);
        //afficher(&(this->Im),NULL,NULL,true,5.0);

        features = Mat<int>((int)0, Im.rows,Im.cols);
        nbrlandmarksfound = 0;
    }

    ~FAST()
    {

    }

    void run(int tresh = 25)
    {
        extractCorner(tresh);
    }

    void extractCorner(int tresh)
    {

        for(int i=Im.rows;i--;)
        {
            for(int j=Im.cols;j--;)
            {
                /*check for a corner*/

                if(check15913(Im, i,j, tresh))
                {
                    /*corner spotted*/
                    features.set((int)255, i+1, j+1);
                    nbrlandmarksfound++;
                }
            }
        }
    }

    inline static vector<cv::Mat> computeGradient(cv::Mat im)
    {
        int ddepth = CV_16S;
        int scale = 1;
        int delta = 0;

        if(im.channels() != 1)
            cv::cvtColor(im,im,CV_BGR2GRAY);
        //cv::GaussianBlur(im,im,cv::Size(0,0),(double)1.0/sqrt(2),(double)1.0/sqrt(2));

        vector<cv::Mat> grad;
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im.rows,im.cols), im.type()) );
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im.rows,im.cols), im.type()) );

        // Gradient X
        Sobel( im, grad[0], ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[0], grad[0] );

        // Gradient Y
        Sobel( im, grad[1], ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[1], grad[1] );

        return grad;
    }

    /*ACCESSEURS*/
    inline Mat<int> getFeatures()  const
    {
        return features;
    }

    inline bool check15913( cv::Mat im, const int x, const int y,int treshold = 25)
    {
        bool ret = false;
        int count = 0;
        uchar *p;
        uchar *o = im.ptr<uchar>(x);

        if(x+3>=im.rows || y+3>=im.cols || x-3<=0 || y-3<=0)
            return false;

        for( int theta=0;theta<=270;theta+=90)
        {
            //p = im.ptr<uchar>(x+3*cos((double)(PI*(((double)theta)/360))));
            p = im.ptr<uchar>(x+3*cos((double)(2*PI*(((double)theta)/360))));

            //if( im.ptr<uchar>( x+1*cos((double)(PI*(((double)theta)/360))), y+1*sin((double)(PI*(((double)theta)/360))) ) > im.ptr<uchar>(x,y) + treshold || im.ptr<uchar>( x+1*cos((double)(PI*(((double)theta)/360))), y+1*sin((double)(PI*(((double)theta)/360))) ) < im.ptr<uchar>(x,y)-treshold )
            //if( p[y+3*(int)sin((double)(PI*(((double)theta)/360)))] > o[y] + treshold || p[y+3*(int)sin((double)(PI*(((double)theta)/360)))] < o[y] -treshold)
            if( p[y+3*(int)sin((double)(2*PI*(((double)theta)/360)))] > o[y] + treshold || p[y+3*(int)sin((double)(2*PI*(((double)theta)/360)))] < o[y] -treshold)
            {
                count++;
                //cout << im.ptr<uchar>( x+1*cos((double)(PI*(((double)theta)/360))), y+1*sin((double)(PI*(((double)theta)/360))) ) << " > " << im.ptr<uchar>(x,y) + treshold << endl;
                //cout <<  p[y+1*(int)sin((double)(PI*(((double)theta)/360)))] << endl; //, y+3*sin((double)(PI*(((double)theta)/360))) ) << " < " << im.ptr<uchar>(x,y)-treshold << endl;
            }

        }

        if( count >= 3)
        {
            //todo : full segment search...

            ret = true;
        }

        return ret;
    }


protected :
    cv::Mat Im;
    vector<cv::Mat> gradients;
    Mat<int> features;
    int nbrlandmarksfound;

};

class BRIEFSIFT : public SIFT
{
public :

    BRIEFSIFT(int octave = 4) : SIFT(octave)
    {
        S = 15;
        n = 512;
        SamplingMode = 1;
        factor = 1.0;

    }

    BRIEFSIFT(cv::Mat Im, cv::Mat ImageOriginale, int n = 512, int SamplingMode = 1, int S = 15, int octave = 4) : SIFT(Im, octave)
    {
        cv::cvtColor(ImageOriginale, this->ImOr, CV_BGR2GRAY);
        factor = (double)(Im.rows)/ImOr.rows;
        this->n = n;
        this->SamplingMode = SamplingMode;
        this->S = (this->Im.rows > S && this->Im.cols > S ? S : 2);
    }
    
    ~BRIEFSIFT()
    {
    
    }

    void generatePatchs(int treshold, int mode)
    {
        SIFT::run(treshold,mode);

        for(int i=SIFT::features.getLine();i--;)
        {
            for(int j=SIFT::features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    Mat<double> temp(2,1);
                    temp.set((double)i, 1,1);
                    temp.set((double)j, 2,1);
                    keypoints.insert(keypoints.end(), temp);
                    cv::Mat patchimage(extractCV(ImOr, i*1.0/factor,j*1.0/factor, S));
                    patchs.insert(patchs.end(),patchimage);
                    cv::Mat rotatedpatchimage(extractCV(ImOr, i*1.0/factor, j*1.0/factor, (int)(sqrt(S*S+S*S)+1)) );
                    rotatedPatchs.insert( rotatedPatchs.end(), rotatedpatchimage);
                }

            }
        }
    }

    void computeOrientationORB(bool rotationInvariance = false, int indice = 0)
    {
        uchar* p;
        uchar* pr;

        if(!indice)
        {
            for(int ind=keypoints.size();ind--;)
            {
                Mat<double> temp( centroid( cv2Matp<double>(patchs[ind]) ) );
                double theta = atan21(temp.get(1,1), temp.get(2,1) );                
                //c.insert(c.end(), operatorC(temp, Mat<double>(theta,1,1) ) );
                c.insert(c.end(), operatorC( keypoints[ind], Mat<double>(theta,1,1) ) );

                /*rotation*/
                if(rotationInvariance)
                {
                    if(theta != (double)0)
                    {
                        for(int i=patchs[ind].rows;i--;)
                        {
                            p = patchs[ind].ptr<uchar>(i);

                            for(int j=patchs[ind].cols;j--;)
                            {
                                Mat<double> vec( (double)i-patchs[ind].rows+1, 2,1);
                                vec.set( (double)j-patchs[ind].cols+1, 2,1);

                                vec = computeSO<double>(-theta)*vec;

                                int x =(int)vec.get(1,1) + (int)patchs[ind].rows-1;
                                int y = (int)vec.get(2,1) + (int)patchs[ind].cols-1;

                                pr = rotatedPatchs[ind].ptr<uchar>(x );
                                pr[y ] = p[j];

                            }
                        }
                    }
                    /*-----------------*/
                    /*
                    cv::Mat aff1,aff2;
                    cv::resize(patchs[ind],aff1,cv::Size(0,0),1.0/factor,1.0/factor);
                    cv::resize(rotatedPatchs[ind],aff2,cv::Size(0,0),1.0/factor,1.0/factor);
                    afficher(&aff1,&aff2,NULL,true);
                    */
                }
            }
        }
    }

    void computeBinaryDescriptorORB(bool rotationInvariance = false)
    {
        uchar* p;
        int radius = S/10+2;
        double theta = 0.0;
        Mat<int> bin(n,1);
        int indice = 1;
        int off = 1;
        int nbrPointPerRadius = (int)((double)n/(S/2-off-radius+1) );

        for(int k=keypoints.size();k--;)
        {            
            //cv::Mat temp(patchs[k]);
            if(rotationInvariance)
                p = rotatedPatchs[k].ptr<uchar>(rotatedPatchs[k].rows/2);
            else
                p = patchs[k].ptr<uchar>(S/2);

            indice = 1;

            //Mat<int> bin( (p[S/2]>=p[S/2 + radius] ? 1 : 0), n,1);

            while(radius <= S/2-2)
            {
                while(theta <= 2*PI)
                {
                      indice++;
                      if(rotationInvariance)
                      {
                          p = rotatedPatchs[k].ptr<uchar>( rotatedPatchs[k].rows/2 + (int)(radius*sin(theta)) );
                          bin.set( (p[rotatedPatchs[k].cols/2] >= p[ rotatedPatchs[k].cols/2 + (int)(radius*cos(theta))] ? 1 : 0 ) ,indice,1);
                      }
                      else
                      {
                          p = patchs[k].ptr<uchar>( S/2 + (int)(radius*sin(theta)) );
                          bin.set( (p[S/2] >= p[S/2 + (int)(radius*cos(theta))] ? 1 : 0 ) ,indice,1);
                      }

                      theta+= (double)(2*PI)/(nbrPointPerRadius);

                      /*--------*/
                      //p[S/2 + (int)(radius*cos(theta))] = 255;
                }

                radius++;
                theta = 0.0;
            }

            /*---------*/
            /*
            cv::resize(patchs[k],patchs[k],cv::Size(0,0),1.0/factor,1.0/factor);
            cv::resize(temp,temp,cv::Size(0,0),1.0/factor,1.0/factor);
            afficher(&patchs[k],&temp);
            cv::resize(patchs[k],patchs[k],cv::Size(0,0),factor,factor);
            */
            bstr.insert(bstr.begin(),bin);            
        }

    }

    void run(int tresh = 255/10, int mode = 0, bool rotationInvariance = false)
    {        
        generatePatchs(tresh,mode);        
        computeOrientationORB(rotationInvariance);

        if(SamplingMode==1)
            computeBinaryDescriptorORB(rotationInvariance);
    }


    /*---------ACCESSEUR---------------*/
    vector<Mat<int> > getBinaryString()
    {
        return bstr;
    }

    vector<cv::Mat> getPatchs()
    {
        return patchs;
    }

    vector<cv::Mat> getRotatedPatchs()
    {
        return rotatedPatchs;
    }


private :

    double factor;
    cv::Mat ImOr;
    int S; //patch size SxS
    int n; //length of the binary descriptor
    int SamplingMode; // 0 : random ; 1 : X=(0,0) Y = all possible value on a coarse polar grid ;
                        // 2 : random from discrete location of a coarse polar grid

    vector<Mat<double> > keypoints;
    vector<cv::Mat> patchs;
    vector<cv::Mat> rotatedPatchs;
    vector<Mat<double> > c;     // coordinate of the centroid of keypoints and orientation of the keypoint.
    vector<Mat<int> > bstr;    // binary string for each keypoints

};

class BRIEFFAST : public FAST
{
public :

    BRIEFFAST() : FAST()
    {
        S = 15;
        n = 512;
        SamplingMode = 1;
        factor = 1.0;

    }

    BRIEFFAST(cv::Mat Im, cv::Mat ImageOriginale, int n = 512, int SamplingMode = 1, int S = 15, int nbrFeatures_ = 25, bool showAll = false) : FAST(Im)
    {
        cv::cvtColor(ImageOriginale, this->ImOr, CV_BGR2GRAY);
        factor = (double)(Im.rows)/ImOr.rows;
        this->n = n;
        this->SamplingMode = SamplingMode;
        this->S = (this->Im.rows > S && this->Im.cols > S ? S : 2);
        this->nbrFeatures = nbrFeatures_;
        this->show = showAll;
    }

    void generatePatchs(int treshold, int mode, bool rotationInvariance = false)
    {                                        

        FAST::run(treshold);

        //cout << nbrlandmarksfound << " / " << 2*nbrFeatures << endl;
        
        if(show)
            allFeatures = features;

        /*if(nbrlandmarksfound < nbrFeatures-15 || nbrlandmarksfound > nbrFeatures+15)
            features = Mat<int>(0, features.getLine(),features.getColumn());*/
        if(nbrlandmarksfound > 2*nbrFeatures)
        {
            int nbrdeletion = nbrlandmarksfound-nbrFeatures;
            for(int i=features.getLine();i--;)
            {
                for(int j=features.getColumn();j--;)
                {
                    if(features.get(i+1,j+1) != 0)
                    {
                        features.set(0, i+1,j+1);
                        nbrdeletion--;

                        if(nbrdeletion == 0)
                        {
                            j=0;
                            i=0;
                            break;
                        }
                    }
                }
            }
        }


        for(int i=FAST::features.getLine();i--;)
        {
            for(int j=FAST::features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    Mat<double> temp(2,1);
                    temp.set((double)i, 1,1);
                    temp.set((double)j, 2,1);
                    keypoints.insert(keypoints.end(), temp);
                    cv::Mat patchimage(extractCV(ImOr, i*1.0/factor,j*1.0/factor, S));
                    patchs.insert(patchs.end(),patchimage);

                    if(rotationInvariance)
                    {
                        cv::Mat rotatedpatchimage(extractCV(ImOr, i*1.0/factor, j*1.0/factor, (int)(sqrt(S*S+S*S)+1)) );
                        rotatedPatchs.insert( rotatedPatchs.end(), rotatedpatchimage);
                    }
                }

            }
        }
    }

    void computeOrientationORB(bool rotationInvariance = false, int indice = 0)
    {
        uchar* p;
        uchar* pr;

        if(!indice)
        {
            for(int ind=keypoints.size();ind--;)
            {
                Mat<double> temp( centroid( cv2Matp<double>(patchs[ind]) ) );
                double theta = atan21(temp.get(1,1), temp.get(2,1) );
                //c.insert(c.end(), operatorC(temp, Mat<double>(theta,1,1) ) );
                c.insert(c.end(), operatorC( keypoints[ind], Mat<double>(theta,1,1) ) );

                /*rotation*/
                if(rotationInvariance)
                {
                    if(theta != (double)0)
                    {
                        for(int i=patchs[ind].rows;i--;)
                        {
                            p = patchs[ind].ptr<uchar>(i);

                            for(int j=patchs[ind].cols;j--;)
                            {
                                Mat<double> vec( (double)i-patchs[ind].rows+1, 2,1);
                                vec.set( (double)j-patchs[ind].cols+1, 2,1);

                                vec = computeSO<double>(-theta)*vec;

                                int x =(int)vec.get(1,1) + (int)patchs[ind].rows-1;
                                int y = (int)vec.get(2,1) + (int)patchs[ind].cols-1;

                                pr = rotatedPatchs[ind].ptr<uchar>(x );
                                pr[y ] = p[j];

                            }
                        }
                    }
                    /*-----------------*/
                    /*
                    cv::Mat aff1,aff2;
                    cv::resize(patchs[ind],aff1,cv::Size(0,0),1.0/factor,1.0/factor);
                    cv::resize(rotatedPatchs[ind],aff2,cv::Size(0,0),1.0/factor,1.0/factor);
                    afficher(&aff1,&aff2,NULL,true);
                    */
                }
            }
        }
    }

    void computeBinaryDescriptorORB(bool rotationInvariance = false)
    {
        uchar* p;
        int radius = S/10+2;
        double theta = 0.0;
        Mat<int> bin(n,1);
        int indice = 1;
        int off = 1;
        int nbrPointPerRadius = (int)((double)n/(S/2-off-radius+1) );

        for(int k=keypoints.size();k--;)
        {
            //cv::Mat temp(patchs[k]);
            if(rotationInvariance)
                p = rotatedPatchs[k].ptr<uchar>(rotatedPatchs[k].rows/2);
            else
                p = patchs[k].ptr<uchar>(S/2);

            indice = 1;

            //Mat<int> bin( (p[S/2]>=p[S/2 + radius] ? 1 : 0), n,1);

            while(radius <= S/2-2)
            {
                while(theta <= 2*PI)
                {
                      indice++;
                      if(rotationInvariance)
                      {
                          p = rotatedPatchs[k].ptr<uchar>( rotatedPatchs[k].rows/2 + (int)(radius*sin(theta)) );
                          bin.set( (p[rotatedPatchs[k].cols/2] >= p[ rotatedPatchs[k].cols/2 + (int)(radius*cos(theta))] ? 1 : 0 ) ,indice,1);
                      }
                      else
                      {
                          p = patchs[k].ptr<uchar>( S/2 + (int)(radius*sin(theta)) );
                          bin.set( (p[S/2] >= p[S/2 + (int)(radius*cos(theta))] ? 1 : 0 ) ,indice,1);
                      }

                      theta+= (double)(2*PI)/(nbrPointPerRadius);

                      /*--------*/
                      //p[S/2 + (int)(radius*cos(theta))] = 255;
                }

                radius++;
                theta = 0.0;
            }

            /*---------*/
            /*
            cv::resize(patchs[k],patchs[k],cv::Size(0,0),1.0/factor,1.0/factor);
            cv::resize(temp,temp,cv::Size(0,0),1.0/factor,1.0/factor);
            afficher(&patchs[k],&temp);
            cv::resize(patchs[k],patchs[k],cv::Size(0,0),factor,factor);
            */
            bstr.insert(bstr.begin(),bin);
        }

    }

    void run(int tresh = 100, int mode = 0, bool rotationInvariance = false)
    {
        generatePatchs(tresh,mode, rotationInvariance);
        if(rotationInvariance)
            computeOrientationORB(rotationInvariance);

        if(SamplingMode==1)
            computeBinaryDescriptorORB(rotationInvariance);
    }


    /*---------ACCESSEUR---------------*/
    vector<Mat<int> > getBinaryString()
    {
        return bstr;
    }

    vector<cv::Mat> getPatchs()
    {
        return patchs;
    }

    vector<cv::Mat> getRotatedPatchs()
    {
        return rotatedPatchs;
    }

    int getNbrLandmarksFound()
    {
        return nbrlandmarksfound;
    }

    Mat<int> getAllFeatures()
    {
        return allFeatures;
    }


private :

    double factor;
    cv::Mat ImOr;
    int S; //patch size SxS
    int n; //length of the binary descriptor
    int SamplingMode; // 0 : random ; 1 : X=(0,0) Y = all possible value on a coarse polar grid ;
                        // 2 : random from discrete location of a coarse polar grid
    int nbrFeatures;

    vector<Mat<double> > keypoints;
    vector<cv::Mat> patchs;
    vector<cv::Mat> rotatedPatchs;
    vector<Mat<double> > c;     // coordinate of the centroid of keypoints and orientation of the keypoint.
    vector<Mat<int> > bstr;    // binary string for each keypoints
    bool show;
    Mat<int> allFeatures;

};

/*
int po(int a, int p)
{
    int r = 1;
    for(int k=1;k<=p;k++)	r *= a;

    return r;
}


float po(float a, float p)
{
    float r = 1;
    //for(int k=1;k<=p;k++)	r *= a;

    if( p!= (float)0)
    {
        if(a != 0)
        {
            r = exp(p*log(a));
        }
        else
            r = 0;
    }


    return r;
}


float po(float a, int p)
{
    float r = 1;
    for(int k=1;k<=p;k++)	r *= a;
    //r = exp(p*log(a));

    return r;
}
*/

template<typename T1,typename T2>
T1 po(T1 a, T2 p)
{
    T1 r = (T1)1;
    //for(int k=1;k<=p;k++)	r *= a;

    if( p!= (T2)0)
    {
        if(a != (T1)0)
        {
            r = (T1)exp(p*log(a));
        }
        else
            r = (T1)0;
    }


    return r;
}

template<typename T>
T RAWmoment(Mat<T> I, int p, int q)
{
    T r = 0;

    for(int i=I.getLine()+1;i--;)
    {
        for(int j=I.getColumn()+1;j--;)
        {
            r += I.get(i+1,j+1)*(po(i+1,p)*po(j+1,q));
        }
    }

    return r;
}

template<typename T>
Mat<T> centroid(Mat<T> I)
{
    Mat<T> r((T)1, 2,1);

    r.set( (T)RAWmoment(I,1,0)/RAWmoment(I,0,0), 1,1);
    r.set( (T)RAWmoment(I,0,1)/RAWmoment(I,0,0), 2,1);

    return r;
}

template<typename T>
Mat<T> computeSO(double theta, int dimension, int axis)
{
    if(dimension==2)
    {
        Mat<T> ret((T)cos(theta),2,2);
        ret.set( (T)-sin(theta), 1,2);
        ret.set( (T)sin(theta),2,1);

        return ret;
    }
    else if(dimension==3)
    {
        Mat<T> ret((T)cos(theta),3,3);

        switch(axis)
        {
            case 1:
            ret.set((T)1,1,1);
            ret.set((T)0,1,2);
            ret.set((T)0,1,3);
            ret.set((T)0,2,1);
            ret.set((T)0,2,3);
            ret.set((T)-sin(theta),2,3);
            ret.set((T)sin(theta),3,2);
            break;

            case 2:
            ret.set((T)1,2,2);
            ret.set((T)0,1,2);
            ret.set((T)0,2,1);
            ret.set((T)0,3,2);
            ret.set((T)0,2,3);
            ret.set((T)-sin(theta),1,3);
            ret.set((T)sin(theta),3,1);
            break;

            case 3:
            ret.set((T)1,3,3);
            ret.set((T)0,1,3);
            ret.set((T)0,2,3);
            ret.set((T)0,3,1);
            ret.set((T)0,3,2);
            ret.set((T)-sin(theta),1,2);
            ret.set((T)sin(theta),2,1);
            break;

            default :
            ret = Mat<T>((T)0,3,3);
            break;

        }

        return ret;
    }
    else
        return Mat<T>((T)0,2,2);
}

template<typename T>
Mat<T> computeK(const Mat<T>& P)
{
    Mat<T> K( extract(P,1,1,3,3) );
    double temp = 0.0;

    temp = sqrt((double)(K.get(3,2)*K.get(3,2) + K.get(3,3)*K.get(3,3)) );
    K = K*computeSO<T>((double)atan21<double>((double)(-(double)K.get(3,2)/temp), (double)((double)K.get(3,3)/temp)), 3,1);

    temp = sqrt((double)(K.get(3,1)*K.get(3,1) + K.get(3,3)*K.get(3,3)) );
    K = K*computeSO<T>((double)atan21<double>((double)(-(double)K.get(3,1)/temp), (double)((double)K.get(3,3)/temp)), 3,2);

    temp = sqrt((double)(K.get(2,2)*K.get(2,2) + K.get(2,1)*K.get(2,1)) );
    K = K*computeSO<T>((double)atan21<double>((double)(-(double)K.get(2,2)/temp), (double)((double)K.get(2,1)/temp)), 3,3);

    return K;

}
