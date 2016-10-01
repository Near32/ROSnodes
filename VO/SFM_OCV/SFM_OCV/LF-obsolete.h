#include "Mat.h"


template<typename T>
class LF 
{
	private :
	
	Mat<T>* ImageR;		/*image to be computed.*/
	Mat<T>* ImageG;
	Mat<T>* ImageB;
	Mat<T>* Image;		/*gray scale image*/
	Mat<T>* Color;
    Mat<T> grad_Image;	/*computed gradient image of Image*/
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
	
        //grad_Image = NULL;
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
	
        //grad_Image = NULL;
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
	
        //grad_Image = NULL;
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
	
        //if(grad_Image != NULL)
        //	delete grad_Image;
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
            Landmarks = new Mat<T>((T)0, (int)3, (int)1);
		}
	}
	
	
	void computeGradient()
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

        //grad_Image = new Mat<T>( correlation(*Image, gradKernel) );

        Mat<T> kX((T)0, 3,3);
        for(int i=1;i<=3;i++)   kX.set( (T)(-1.0/2), i, 1);
        for(int i=1;i<=3;i++)   kX.set( (T)(1.0/2), i, 3);
        Mat<T> kY((T)0, 3,3);
        for(int i=1;i<=3;i++)   kY.set( (T)(-1.0/2), 1, i);
        for(int i=1;i<=3;i++)   kY.set( (T)(1.0/2), 3, i);       

        cout << "Correlation : in progress..." << endl;
        grad_Image = absM( correlation(*Image, kX)) + absM( correlation(*Image,kY));
	}
	
	
	
	void search(int method)	
/*method : 
 * 1 :: Great-Gradient variation Detection ; 
 * 2 :: Harris CornerDetector
 * 3 :: colorimetrique */
 	{
		resetLM();
	
	
		switch(method)
		{
			case 1:
			{
			computeGradient();
            T moyenne = (T)mean( grad_Image);
            int count = 0;

            for(int i=1;i<=grad_Image.getLine();i++)
			{
                for(int j=1;j<=grad_Image.getColumn();j++)
				{
                    if( (T)(grad_Image.get(i,j)) >= (T)(((T)15)*moyenne))
					{
                        cout << grad_Image.get(i,j) << " >= " << 15*moyenne << endl;

						Mat<T> t( 3, 1);
						t.set((T)i, 1,1);
						t.set((T)j, 2,1);
						t.set((T)Image->get(i,j), 3,1);
			
						*Landmarks = operatorL(*Landmarks, t);

                        cout << count++ << endl;
					}
				}
			}
			}
			break;
		
			case 2 :
		
			/*TODO*/
		
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
		
			default:
		
			break;
		}
	}
 
 	
	bool colorMatch(Mat<T> color, T r, T g, T b)
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
	
	
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------*/
	
	Mat<T> getGrad() const
	{
        return grad_Image;
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





