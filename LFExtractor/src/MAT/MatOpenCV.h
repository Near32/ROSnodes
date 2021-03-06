
namespace mat{

#include "./Mat.h"


void afficher( string s, cv::Mat* im, cv::Mat* im1, cv::Mat* im2, bool cont, float factor);
template<typename T>
void afficherMat( string s, Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor);
template<typename T>
void afficherMat( Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor);


/*---------------------------------------------*/

/*
template<typename T>	//pas de point virgule en fin de ligne...
//renvoi le nombre de matrice contenue dans tab,
//correspondant au nombre de channel de mat.

int cv2Mat(const cv::Mat mat, Mat<T>* tab)
{
    int nbr_chan = 0;
    int r = 0;
    //int c = 0;

    if(tab != NULL)
    {
        nbr_chan = 3; //(int)mat.channels();
        r = mat.rows;
        //c = mat.cols;

        cout << "Conversion d'une CvMat<T> avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat<T>." << endl;

        //tab = (Mat<T>**)malloc(sizeof(Mat<T>*)*nbr_chan);
        //tab = new Mat<T>*[nbr_chan];
        //tab = new Mat<T>(FLOAT, (T)0, r,c);



        //for(int k=0;k<=nbr_chan-1;k++)
        //{
        //	cout << "begin " << k << endl;
        //	tab[k] =  new Mat<T>(FLOAT, (T)0, r, c);
        //}


        cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

        int i = 0;
        int j = 0;


        for( ; it != itend; ++it)
        {
            //cout << i << "/" << r << " & " << j << "/" << c << endl;
            //tab[0]->set( (*it)[0], i+1,j+1);
            //tab[1]->set( (*it)[1], i+1,j+1);
            //tab[2]->set( (*it)[2], i+1,j+1);
            tab->set( (*it)[0], i+1,j+1);


            i++;
            if( (i+1)%r == 0 )
            {
                i=0;
                j++;

            }
        }

    }
    else
    {
        cout << "ERREUR : le tableau mis en argument n'est pas NULL. Aucune operation effectuee..." << endl;
    }

    cout << "end" << endl;
    cout << tab << endl;

    return nbr_chan;
}



//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T>* tab)
{
    int nbr_chan = 3;
    int r = tab->getLine();
    int c = tab->getColumn();
    cv::Mat mat( r, c, CV_8UC3);

    if(tab != NULL)
    {

        cout << "Conversion d'une CvMat avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat." << endl;
        cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

        int i = 0;
        int j = 0;

        for( ; it != itend; ++it)
        {
            *it= cv::Vec3b( tab->get(i+1,j+1), tab->get(i+1,j+1), tab->get(i+1,j+1));

            i++;
            if( !(i+1)%r)
            {
                i=0;
                j++;
            }
        }

    }
    else
    {
        cout << "ERREUR : le tableau mis en argument est NULL. Aucune operation effectuee..." << endl;
    }

    return mat;
}





//---------------------------
// renvoi les image ou matrice
//--------------------------



//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Mat(const cv::Mat mat)
{
    int r = mat.rows;
    int c = mat.cols;
    Mat<T> rim( (T)0, r,c);


    //time_t timer;
    //time_t timerp;
    //time(&timer);
    //time(&timerp);
    //cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
    //cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

    //time(&timer);
    //cout << timer-timerp << endl;


    //int i = 0;
    //int j = 0;



    //for( ; it != itend; ++it)
    //{
    //	rim.set( (*it)[0], i+1,j+1);
    //
    //	i++;
    //	if( (i+1)%r == 0 )
    //	{
    //		i=0;
    //		j++;
    //
    //	}
    //}
    //

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    rim.set( mat.data[mat.step*y+x+1], x+1, y+1);
            }
    }

    return rim;
}




//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T> tab)
{
    int r = tab.getLine();
    int c = tab.getColumn();
    cv::Mat mat( r, c, CV_8UC3);

    cv::Mat_<cv::Vec3b>::iterator it = mat.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

    int i = 0;
    int j = 0;

    for( ; it != itend; ++it)
    {
        *it= cv::Vec3b( tab.get(i+1,j+1), tab.get(i+1,j+1), tab.get(i+1,j+1));

        i++;
        if( !(i+1)%r)
        {
            i=0;
            j++;
        }
    }
    //
    //for(int x=0;x<=r;x++)
    //{
    //		for(int y=0;y<=c;y++)
    //		{
    //				mat.data[mat.step*y+x+1] = (unsigned char)(tab.get(x+1, y+1));
    //
    //		}
    //}
    //
    return mat;
}





*/
/*
//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
int CvMat2Mat(const CvMat& mat, Mat<T>** tab)
{
    int nbr_chan = 0;
    int r = 0;
    int c = 0;

    if(tab == NULL)
    {
        nbr_chan = (int)mat.channels();
        r = mat.rows;
        c = mat.cols;

        cout << "Conversion d'une CvMat avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat." << endl;

        tab = (Mat<T>**)malloc(sizeof(Mat<T>*)*nbr_chan);

        for(int k=0;k<=nbr_chan-1;k++)
        {
            tab[k] = (Mat<T>*)malloc(sizeof(Mat<T>));
            *tab[k] = Mat<T>( r, c);;

            for(int i=1;i<=r;i++)
            {
                for(int j=1;j<=c;j++)
                {
                    tab[k]->set((T)(mat.at<T>(i-1,j-1)), i, j);
                }
            }
        }
    }
    else
    {
        cout << "ERREUR : le tableau mis en argument n'est pas NULL. Aucune operation effectuee..." << endl;
    }

    return nbr_chan;
}
*/

/*----------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------*/

cv::Mat absM(const cv::Mat& a)
{
    cv::Mat r(a.rows,a.cols,CV_8UC3);

    for(int i=r.rows;i--;)
    {
        for(int j=r.cols;j--;)
        {

            r.at<char>(i,j) =  (char)fabs_( r.at<char>(i,j));
        }
    }

    return r;

}

cv::Mat absMp(cv::Mat mat)
{
    int c = mat.cols;
    int r = mat.rows;    
    cv::Mat ret(r,c,CV_8UC1);

    /*
    if(channels == 3)
        ret = cv::Mat(mat.rows,mat.cols,CV_8UC3);
        */

    uchar* p;
    uchar* pm;


    for(int i=r;i--;)
    {
        p = mat.ptr<uchar>(i);
        pm = ret.ptr<uchar>(i);

        //for(int j=c*channels;j-=channels;)
        for(int j=c;j--;)
        {
            pm[j] = p[j];

            /*
            if(channels == 3)
            {
                pm[j+1] = p[j+1];
                pm[j+2] = p[j+2];
            }
            */

        }

    }

        return ret;
}


cv::Mat absMAt(cv::Mat mat)
{
    int r = mat.rows;
    int c = mat.cols;
    cv::Mat rim(r,c,CV_8UC1);

    for(int x=r;x--;)
    {
            for(int y=c;y--;)
            {
                    rim.at<uchar>(x,y) = mat.at<uchar>(x,y);
            }
    }

    return rim;
}

inline cv::Mat sum(const cv::Mat& a)
{

    if(a.rows != 1)
    {
        /*sum on columns*/
        cv::Mat r( 1, a.cols, CV_8UC1 );

        for(int j=0;j<a.cols;j++)
        {
            char temp = 0;
            for(int i=0;i<a.rows;i++)
                temp += a.at<char>(i,j);

            r.at<char>( 0,j) = temp;
        }        

        return r;
    }
    else
    {
        /*sum on the only column*/
        cv::Mat r( 1, 1, CV_8UC1);

        for(int i=0;i<a.cols;i++)
        {
            r.at<char>(0,0) = r.at<char>(0,0) + a.at<char>(0,i);
        }


        return r;
    }
}



/*---------------------------------------------*/


template<typename T>	//pas de point virgule en fin de ligne...
int cv2Mat(const cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int r = mat.rows;
    int c = mat.cols;
    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    red->set( mat.data[mat.step*y+x+0], x+1, y+1);
                    green->set( mat.data[mat.step*y+x+1], x+1, y+1);
                    blue->set( mat.data[mat.step*y+x+2], x+1, y+1);
            }
    }

    return 1;
}

template<typename T>	//pas de point virgule en fin de ligne...
int cv2MatAt(const cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int r = mat.rows;
    int c = mat.cols;
    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    red->set( mat.at<cv::Vec3b>(x,y)[0], x+1, y+1);
                    green->set( mat.at<cv::Vec3b>(x,y)[1], x+1, y+1);
                    blue->set( mat.at<cv::Vec3b>(x,y)[2], x+1, y+1);
            }
    }

    return 1;
}

template<typename T>	//pas de point virgule en fin de ligne...
inline int cv2Matp( cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int channels = mat.channels();
    int r = mat.rows;
    int c = mat.cols;

    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    uchar* p;

    //for(int i=0;i<r;i++)
    for(int i=r;i--;)
    {
        p = mat.ptr<uchar>(i);
        //for(int j=0;j<c*channels;j+=3)
        for(int j=c*channels;j-=3;)
        {
            red->set( (T)p[j], i+1, (int)((float)(j)/channels)+1);
            //red->mat[i][(int)((float)j/channels)] = (T)p[j];
            //red->mat[i*(int)((float)j/channels)] = (T)p[j];
            green->set( (T)p[j+1], i+1, (int)((float)(j)/channels)+1);
            //green->mat[i][(int)((float)j/channels)] = (T)p[j+1];
            //green->mat[i*(int)((float)j/channels)] = (T)p[j+1];
            blue->set( (T)p[j+2], i+1, (int)((float)(j)/channels)+1);
            //blue->mat[i][(int)((float)j/channels)] = (T)p[j+2];
            //blue->mat[i*(int)((float)j/channels)] = (T)p[j+2];
        }

    }

    return 1;
}


template<typename T>	//pas de point virgule en fin de ligne...
inline Mat<T> cv2Matp( const cv::Mat& mat)
{
    int channels = mat.channels();
    int r = mat.rows;
    int c = mat.cols;

    Mat<T> ret(r,c,channels);

    //for(int i=0;i<r;i++)
    for(int i=r;i--;)
    {
        const uchar* p = mat.ptr<uchar>(i);
        for(int j=c*channels;j-=3;)
        {
            for(int k=channels;k--;)
            {
            	ret(i+1,(int)((float)(j)/channels)+1,k+1)  = (T)( p[j+k] );
            }
        }

    }

    return ret;
}


/*
template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Matp( const cv::Mat& mat)
{    
	cv::Mat copy;
	mat.copyTo(copy);
    if( copy.channels() != 1)
        cv::cvtColor(copy,copy,CV_BGR2GRAY);

    int r = copy.rows;
    int c = copy.cols;

    Mat<T> rim( r,c);

    uchar* p;

    for(int i=r;i--;)
    {

        p = copy.ptr<uchar>(i);
        for(int j=c;j--;)
            rim.set((T)p[j+c], i+1,j+1);//rim.mat[i][j] = (T)p[j];

    }

    return rim;
}
*/




template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cv(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        for(int x=0;x<=rl-1;x++)
        {
                for(int y=0;y<=rc-1;y++)
                {
                        rim.data[rim.step*y+x+0] = (uchar)r.get( x+1, y+1);
                        rim.data[rim.step*y+x+1] = (uchar)g.get( x+1, y+1);
                        rim.data[rim.step*y+x+2] = (uchar)b.get( x+1, y+1);
                }
        }
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}



template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvVec(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        cv::Mat_<cv::Vec3b>::const_iterator it = rim.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = rim.end<cv::Vec3b>();

        int i = 0;
        int j = 0;

        for( ; it != itend; ++it)
        {
            *it= cv::Vec3b( r.get(i+1,j+1), g.get(i+1,j+1), b.get(i+1,j+1));

            i++;
            if( !(i+1)%rl)
            {
                i=0;
                j++;
            }
        }
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvAt(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        for(int x=0;x<=rl-1;x++)
        {
                for(int y=0;y<=rc-1;y++)
                {
                        rim.at<cv::Vec3b>(x,y) = cv::Vec3b( r.get(x+1,y+1), g.get(x+1,y+1), b.get(x+1,y+1));
                }
        }
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvAt(const Mat<T>& img)
{
    int l=img.getLine();
    int c=img.getColumn();
    int channel=img.getDepth();

	
    cv::Mat rim;
    switch(channel)
    {
    	case 1:
    	rim = cv::Mat( l, c, CV_8UC1);
    	break;
    	
    	case 3:
    	rim = cv::Mat( l, c, CV_8UC3);
    	break;
    }
    for(int x=0;x<=l-1;x++)
    {
            for(int y=0;y<=c-1;y++)
            {
                    switch(channel)
                    {
                    	case 1:
                    	rim.at<uchar>(x,y) = img(x+1,y+1);
                    	break;
                    	
                    	case 3:
                    	rim.at<cv::Vec3b>(x,y) = cv::Vec3b(img(x+1,y+1,1),img(x+1,y+1,2),img(x+1,y+1,3));
                    	break;
                    }
            }
    }

   
    return rim;

}

template<typename T>	//pas de point virgule en fin de ligne...
inline cv::Mat Mat2cvp(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);
    uchar* p;
    int channels = 3;

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        //cout << "Creating cv::Mat..." << endl;
        //for(int x=0;x<rl;x++)
        for(int x=rl;x--;)
        {
                p = rim.ptr<uchar>(x);
                //for(int y=0;y<rc;y++)
                for(int y=rc;y--;)
                {
                        p[y*channels] = (uchar)r.get(x+1,y+1);//r.mat[x][y];
                        p[y*channels+1] = (uchar)g.get(x+1,y+1);//g.mat[x][y];
                        p[y*channels+2] = (uchar)b.get(x+1,y+1);//b.mat[x][y];
                }
        }
        //cout << "Creation cv::Mat : DONE." << endl;
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvp(const Mat<T>& img)
{
    int l=img.getLine();
    int c=img.getColumn();
    int channels=img.getDepth();
    
    cv::Mat rim( l, c, CV_8UC3);
    uchar* p;

    for(int x=l;x--;)
    {
            p = rim.ptr<uchar>(x);
            for(int y=c;y--;)
            {
                    p[y*3] = (uchar) img(x+1,y+1,1);//r.mat[x][y];
                    p[y*3+1] = (uchar) img(x+1,y+1,2);//g.mat[x][y];
                    p[y*3+2] = (uchar) img(x+1,y+1,3);//b.mat[x][y];
            }
    }
    
    return rim;

}
/*************************************************/

cv::Mat extractCV(cv::Mat im,int x, int y, int nneigh)
{    

    cv::Mat rim( (nneigh >=1 ? nneigh : 1), (nneigh >=1 ? nneigh : 1), im.type());
    uchar *p, *pi;
    int channels = im.channels();

    for(int xi=nneigh;xi--;)
    {
        if(x-nneigh/2 +xi > 0 && x-nneigh/2 + xi < im.rows)
        {
            p = rim.ptr<uchar>(xi);
            pi = im.ptr<uchar>(x-nneigh/2 + xi);

            for(int yi=nneigh;yi--;)
            {
                for(int c=channels;c>=1;c--)
                {
                    if(yi*channels+(c-1)<rim.cols && (y-nneigh/2+yi)*channels+(c-1)<im.cols)
                    {
                        p[yi*channels + (c-1)] = pi[(y-nneigh/2+yi)*channels + (c-1)];
                    }
                }
            }
        }
    }

    return rim;

}

void afficher( cv::Mat* im, cv::Mat* im1, cv::Mat* im2, bool cont, float factor)
{
    bool continuer = true;
   /*CAMERA */
    /*
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            continuer = false;
    }
    */

    //cv::destroyAllWindows();
    cv::namedWindow("OUTPUT");

    if(im1 != NULL)
        cv::namedWindow("OUTPUT 1");
    if(im2!=NULL)
        cv::namedWindow("OUTPUT 2");

    /*--------------------*/

    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),factor,factor);
        if(im1!=NULL)
            cv::resize(*im1,*im1,cv::Size(0,0),factor,factor);
        if(im2!=NULL)
            cv::resize(*im2,*im2,cv::Size(0,0),factor,factor);
    }



    while(continuer)
    {
        cv::imshow("OUTPUT", *im);

        if(im1 != NULL)
            cv::imshow("OUTPUT 1", *im1);
        if(im2!=NULL)
            cv::imshow("OUTPUT 2", *im2);


        if(cv::waitKey(30) >= 0)
            continuer = false;
        if(cont)
            continuer = false;
    }

    //cv::destroyAllWindows();
    //cap.release();
    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),1.0/factor,1.0/factor);
    }

    /*--------------------------*/
}


template<typename T>
void afficherMat( Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor)
{
    if(im != NULL)
    {
        cv::Mat imcv(Mat2cvp(*im,*im,*im));

        if(im1 != NULL)
        {
            cv::Mat im1cv( Mat2cvp(*im1,*im1,*im1));

            if(im2 != NULL)
            {
                cv::Mat im2cv(Mat2cvp(*im2,*im2,*im2));

                afficher(&imcv,&im1cv,&im2cv, cont, factor);
            }
            else
                afficher(&imcv,&im1cv,NULL, cont, factor);
        }
        else
            afficher(&imcv,NULL,NULL, cont, factor);
    }
}

template<typename T>
void displayInverseCodedFrame( string s, Mat<T>* frame, Mat<T>* invDepthMap, bool cont, float factor)
{
    if(invDepthMap != NULL)
    {
    	T idmean = (T)mean(*invDepthMap);
    	
        cv::Mat imcv( Mat2cvp( *frame, *frame, *frame));
        
        
        for(int i=invDepthMap->getLine();i--;)
        {

            for(int j=invDepthMap->getColumn();j--;)
            {
				T id = invDepthMap->get(i+1,j+1)/idmean;
				
                if( id != 0.0f )
                {
                	cv::Scalar color(0,0,0);
                	if(id<(T)0)
                		id = -id;
 
                	if(id >= (T)0)
		        	{
						// rainbow between 0 and 4
						float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;
						float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
						float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;

						uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
						uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
						uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);
				
						//rc = 0;
						//gc = 0;
						//bc = id*255;
						color = cv::Scalar(255-bc, 255-gc, 255-rc);
						//color = cv::Scalar(bc, gc, rc);
					}
                	
                	cv::line(imcv,
                                 cv::Point(j,i),
                                 cv::Point(j,i),
                                 color);
                }
            }
            
        }
        
        ostringstream tmean;
        tmean << "MEAN = " << idmean;
        cv::putText(imcv, tmean.str(), cv::Point(5,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.0f, cv::Scalar(255,0,255));

        afficher(s,&imcv,NULL,NULL, cont, factor);        
    }
}


void afficher( string s, cv::Mat* im, cv::Mat* im1, cv::Mat* im2, bool cont, float factor)
{
    bool continuer = true;
   /*CAMERA */
    /*
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            continuer = false;
    }
    */

    //cv::destroyAllWindows();
    cv::namedWindow(s.c_str());

    if(im1 != NULL)
        cv::namedWindow( (s+string(" 1")).c_str() );
    if(im2!=NULL)
        cv::namedWindow( (s+string(" 2")).c_str() );

    /*--------------------*/

    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),factor,factor);
        if(im1!=NULL)
            cv::resize(*im1,*im1,cv::Size(0,0),factor,factor);
        if(im2!=NULL)
            cv::resize(*im2,*im2,cv::Size(0,0),factor,factor);
    }



    while(continuer)
    {
        cv::imshow(s.c_str(), *im);

        if(im1 != NULL)
            cv::imshow((s+string(" 1")).c_str(), *im1);
        if(im2!=NULL)
            cv::imshow( (s+string(" 2")).c_str(), *im2);


        if(cv::waitKey(30) >= 0)
            continuer = false;
        if(cont)
            continuer = false;
    }

    //cv::destroyAllWindows();
    //cap.release();
    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),1.0/factor,1.0/factor);
    }

    /*--------------------------*/
}


template<typename T>
void afficherMat( string s, Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor)
{
    if(im != NULL)
    {
        cv::Mat imcv(Mat2cvp(*im,*im,*im));

        if(im1 != NULL)
        {
            cv::Mat im1cv( Mat2cvp(*im1,*im1,*im1));

            if(im2 != NULL)
            {
                cv::Mat im2cv(Mat2cvp(*im2,*im2,*im2));

                afficher(s, &imcv,&im1cv,&im2cv, cont, factor);
            }
            else
                afficher( s, &imcv,&im1cv,NULL, cont, factor);
        }
        else
            afficher( s, &imcv,NULL,NULL, cont, factor);
    }
}


};
