//#define verbose
#define OPENCV_USE
#include "SFM.h"



#include "../../RAND/rand.h"
#include "../../ONSC/ONSC.h"
#include "../../ONSC/ONSCNewton.h"
#include "../../MVG/MVG.h"
#include "../../MVG/Frame.h"

//#include <QApplication>
//#include "myWindow.h"
#include "../../ONSC/LS3wSD.h"


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
//void afficher( cv::Mat* im, cv::Mat* im1 = NULL, cv::Mat* im2 = NULL);
int main0();	//SFM TX TY
int main1();	//FT test
int main2();    //Live FT+SFM test
int main3();    //test FPS BRIEF SIFT
int matching(cv::Mat* im1_,cv::Mat* im2_, int* treshold_= NULL, int* count_=NULL, bool continuer = true,bool rotationInvariance = false);
int main4();    //test FT
int main6();
int matchingFAST(cv::Mat* im1_, cv::Mat* im2_, int* treshold_= NULL, int* count_=NULL, bool continuer = true,bool rotationInvariance = false);
int main7();
int main8(bool fast = false);
template<typename T>
inline bool notIn(vector<T> set, T elem);
inline bool optim_test(Mat<int> m1, Mat<int> m2);
cv::Mat process(cv::Mat* im_, Mat<int> *features_, vector<Mat<int> > *binStrings_, vector<cv::Mat> *patchs_, int *treshold_ = NULL, int *count_ = NULL, bool rotationInvariance = false);
cv::Mat processFAST(cv::Mat* im_, Mat<int> *features_, vector<Mat<int> > *binStrings_, vector<cv::Mat> *patchs_, int *treshold_ = NULL, int *count_ = NULL, bool rotationInvariance = false);
void seekMatchs(const vector<vector<cv::Mat> > patchs_ct, Mat<int> *matchs);

int main9();
//given two images and the depthmap of the first one and an intrinsic calibration matrix,
//it computes the rigid-body motion that relates one to the other.
Mat<float> poseEstimation(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Mat<float>* K, Mat<float>* invK);
Mat<float> initializeInvDM(Frame<float>* frame,float gradIntTreshold = 255/10, float variance = 0.5, float mean = 0.9);
inline cv::Mat computeGradientXY(cv::Mat* im, cv::Mat* imgradX, cv::Mat* imgradY);
inline cv::Mat computeGradient(cv::Mat* im);
void computeEpipolarLine(Mat<float>* l, Mat<float>* x, Mat<float>* R, Mat<float>* t, Mat<float>* K, Mat<float>* invK);
void computeEpipolarLine2(Mat<float>* l, Mat<float>* invDepthMap1, Mat<float>* x, Mat<float>* R, Mat<float>* t, Mat<float>* K, Mat<float>* invK);
float exhaustiveSearchEpipolarLine(int nbrHypSearch, Frame<float>* f1, Frame<float>* f2, Mat<float>* invDepthMap1, Mat<float>* x1, Mat<float>* x2, Mat<float>* R, Mat<float>* t, float var_invDepthMap, Mat<float>* K, Mat<float>* invK, float factor = 1.0/20);

int main10(int argc, char* argv[]);
int main11(int argc, char* argv[]);

void debugPose(Frame<float>* f1, Frame<float>* f2, const Mat<float>& dm1, const Mat<float>& R, const Mat<float>& t, const Mat<float>& K, const Mat<float>& invK, float factor = 1.0/20);
int main12();
Mat<float> poseEstimationLS(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI);
Mat<float> poseEstimationLSGRAD(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Frame<float>* gradI1X, Frame<float>* gradI1Y, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI, int pyramidDepth = 3);
template<typename T>
Mat<T> propagateDepthMap( Mat<T>* K, Mat<T>* invK, Mat<T>* deltaPose, Mat<T>* invdm);


int main(int argc, char* argv[])
{
    int mode = 12;
    cv::Mat im1 = cv::imread("../SFM_OCV/data1.pgm");
    cv::Mat im2 = cv::imread("../SFM_OCV/data2.pgm");

    if(argc>1)
        mode = atoi(argv[1]);

    switch(mode)
    {
        case 0:
        main0();
        break;

        case 1:
        main1();
        break;

        case 2:
        main2();
        break;

        case 3:
        main3();
        break;

        case 4:
        main4();
        break;

        case 5:
        matching(&im1,&im2,NULL,NULL,true,true);
        break;

        case 6:
        main6();
        break;

        case 7:
        matchingFAST(&im1,&im2);
        break;

        case 8:
        main7();
        break;

        case 9:
        main8();
        break;

        case 10:
        main8(true);
        break;

        case 11:
        main9();
        break;

        case 12:
        main10(argc,argv);
        break;

        case 13:
        main11(argc,argv);
        break;

        case 14:
        main12();
        break;
    }

    return 0;
}

int main0()
{
    int dimx = 64;
    int dimy = 48;
    int F = 10;
    int P = 10;	// 2*F > P, mandatory.

    Mat<float> U( (float)0, F,P);
    Mat<float> V( (float)0, F,P);    

    //frame 1:
    U.set( dimx/2, 1,1);
    V.set( dimy/2, 1,1);    

    U.set( dimx/2+10, 1,2);
    V.set( dimy/2, 1,2);

    U.set( dimx/2+10, 1,3);
    V.set( dimy/2+10, 1,3);

    U.set( dimx/2, 1,4);
    V.set( dimy/2+10, 1,4);

    U.set( dimx/2+10, 1,5);
    V.set( dimy/2-10, 1,5);

    U.set( dimx/2, 1,6);
    V.set( dimy/2-10, 1,6);


    for(int j=7;j<=P-1;j++)
    {
        U.set( dimx/4, 1,j);
        V.set( dimy/8+j, 1,j);
    }

    U.set( mean(Line(U,1)), 1,P);
    V.set( mean(Line(V,1)), 1,P);

    float offset = 0;
    //next frames : translation en x.
    for(int fr=2;fr<= (int)(F/2.);fr++)
    {
        U.set( dimx/2+fr, fr,1);
        V.set( dimy/2, fr,1);

        U.set( dimx/2+10+fr, fr,2);
        V.set( dimy/2, fr,2);

        U.set( dimx/2+10+fr, fr,3);
        V.set( dimy/2+10, fr,3);

        U.set( dimx/2+fr, fr,4);
        V.set( dimy/2+10, fr,4);

        U.set( dimx/2+10+fr, fr,5);
        V.set( dimy/2-10, fr,5);

        U.set( dimx/2+fr, fr,6);
        V.set( dimy/2-10, fr,6);

        for(int j=7;j<P;j++)
        {
            U.set( U.get(1,j)+fr, fr,j);
            V.set( V.get(1,j), fr,j);
        }

        offset = fr;
    }
    //------------------------------

    //next frames : translation en y.

    for(int fr=(int)(F/2.)+1;fr<=F;fr++)
    {
        U.set( dimx/2+offset, fr,1);
        V.set( dimy/2+fr, fr,1);

        U.set( dimx/2+10+offset, fr,2);
        V.set( dimy/2+fr, fr,2);

        U.set( dimx/2+10+offset, fr,3);
        V.set( dimy/2+10+fr, fr,3);

        U.set( dimx/2+offset, fr,4);
        V.set( dimy/2+10+fr, fr,4);

        U.set( dimx/2+10+offset, fr,5);
        V.set( dimy/2-10+fr, fr,5);

        U.set( dimx/2+offset, fr,6);
        V.set( dimy/2-10+fr, fr,6);

        for(int j=7;j<P;j++)
        {
            U.set( U.get(1,j)+offset, fr,j);
            V.set( V.get(1,j)+fr, fr,j);
        }

    }
    //------------------------------

    clock_t timer1 = clock();
    SFM<float> instance(U,V, 0);		// 0: no GNa ; 1 : GNa
    cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
    Mat<float> R(instance.getR()), S(instance.getS());	//rotation and shape matrix.
    R.afficher();
    transpose(S).afficher();

    return 1;
}


int main1()
{
    clock_t timertotal;
    /*FRAME */
    cv::Mat F1,F2;//(640,480, CV_16UC3);
    cv::Mat rF1,rF2;//(640,480,CV_16UC3);
    float factor = 1.0/10.0;
    cv::Size size(0,0); // then the output image size is computed thx to factor. round(fx*src.cols), round(fy*src.rows)

    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    //cv::namedWindow("Output 1");
    cv::namedWindow("Entry 1");    
    /*--------------------*/

    /*FRAME to MAT*/
    Mat<float> iR1, iG1, iB1;
    Mat<float> iR2, iG2, iB2;
    bool continuer = true;
    clock_t timer1 = clock();

    while(continuer)
    {

        //FRAME 1 :
        cap >> F1;
        cv::imshow("Entry 1", F1);

        //FRAME 2 :
        cap >> F2;

        //timer1 = clock();
        cv::resize(F1, rF1, size, factor, factor);
        //cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
        //cv::imshow("Output 1", rF1);

        //timer1 = clock();
        //dst.convertTo(dst, CV_8UC3);
        //cv2MatAt( rF1, &iR1,&iG1,&iB1);
        cv2Matp( rF1, &iR1,&iG1,&iB1);
        //cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


        /*
        if(cv::waitKey(30) >= 0)
            continuer = false;
        */
        //cv::waitKey(100);
        continuer = false;


    }


    //cv::namedWindow("Output 2");
    cv::namedWindow("Entry 2");

    continuer = true;
    while(continuer)
    {
    /*-----------------------------------------*/

        cv::imshow("Entry 2", F2);

        //timer1 = clock();
        cv::resize(F2, rF2, size, factor, factor);
        //cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
        //cv::imshow("Output 2", rF2);

        //timer1 = clock();
        //cv2MatAt( rF2, &iR2,&iG2,&iB2);
        cv2Matp( rF2, &iR2,&iG2,&iB2);
        //cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


        /*
        if(cv::waitKey(30) >= 0)
            continuer = false;
        */
        //cv::waitKey(10);
        continuer = false;

    }

     timertotal = clock();
     cap.release();
     cv::destroyAllWindows();
    /*--------------------------*/

    Mat<float> gF1( ((float)(1.0/3))*(iR1+iG1+iB1));
    Mat<float> gF2( ((float)(1.0/3))*(iR2+iG2+iB2));

    cout << "Initialization : ..." << endl;
    Mat<float> landmarks;
    LF<float> ld( gF1);

    timer1 = clock();
    ld.search(4, /*step*/ 1);	//FAST  //warning : the landmarks could not be found if the step is too large, from one frame to the next..
    cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


    landmarks = ld.getLandmarks();    

    /*
    landmarks = c2pointsMouseClick();
    for(int i=2;i<=5-1;i++)
        landmarks = operatorL(landmarks, c2pointsMouseClick() );
    */

    cout << "Initialization : DONE." << endl;

    for(int i=1;i<=landmarks.getColumn();i++)
    {
        DrawCircle( &iR1, landmarks.get( 1,i), landmarks.get(2,i));
        DrawCircle( &iB1, landmarks.get( 1,i), landmarks.get(2,i));
        DrawCircle( &iG1, landmarks.get( 1,i), landmarks.get(2,i));
    }

    /*FRAME 1 + FAST :*/
    //rF1 = Mat2cvAt(iR1, iG1, iB1);
    rF1 = Mat2cvp(iR1, iG1, iB1);
    cv::resize(rF1, rF1, size, 1.0/factor, 1.0/factor);
    //afficher(&rF1);

    /*--------------------------*/

    cout << "Initialization : ..." << endl;
    Mat<float> landmarks2;
    LF<float> ld2( gF2);

    timer1 = clock();
    ld2.search(4, /*step*/ 10);	//FAST  //warning : the landmarks could not be found if the step is too large, from one frame to the next..
    cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


    landmarks2 = ld2.getLandmarks();

    /*
    landmarks = c2pointsMouseClick();
    for(int i=2;i<=5-1;i++)
        landmarks = operatorL(landmarks, c2pointsMouseClick() );
    */

    cout << "Initialization : DONE." << endl;

    for(int i=1;i<=landmarks2.getColumn();i++)
    {
        DrawCircle( &iR2, landmarks2.get( 1,i), landmarks2.get(2,i));
        DrawCircle( &iB2, landmarks2.get( 1,i), landmarks2.get(2,i));
        DrawCircle( &iG2, landmarks2.get( 1,i), landmarks2.get(2,i));
    }

    /*FRAME 1 + FAST :*/
    //rF2 = Mat2cvAt(iR2, iG2, iB2);
    rF2 = Mat2cvp(iR2, iG2, iB2);
    cv::resize(rF2, rF2, size, 1.0/factor, 1.0/factor);
    //afficher(&rF2);

    /*----------------------*/

    cout << " Feature tracking : ..." << endl;

    timer1 = clock();
    FT<float> instanceFT( gF1, gF2, landmarks);
    cout << "L'execution a pris : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;

    cout << " Feature tracking : DONE." << endl;

    vector<Mat<float> > displacement =  instanceFT.getD();
    vector<Mat<float> > wd = instanceFT.getWindow();
    int nw = instanceFT.getnw();
    int mw = instanceFT.getmw();
    int nbrWpn = (int)(((float)gF2.getLine())/(float)nw +1);
    int nbrWpm = (int)(((float)gF2.getLine())/(float)mw +1);

    /*for(int i=0;i<=(int)displacement.size()-(int)1;i++)
        displacement[i].afficher();*/

    cout << "L'execution totale a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;

    /*--------------------------------------*/
    /*Windows */
    Mat<float> d(displacement[0]);
    for(int i=1;i<=displacement.size()-1;i++)
            d = operatorL( d, displacement[i]);
    d.afficher();
    float maxd = max(d);
    cout << "valeur max : "<<  maxd << endl;
    Mat<float> imW((float)0, gF2.getLine(), gF2.getColumn());
    Mat<float> imF12(gF1);

    for(int i=1;i<=nbrWpn;i++)
    {
        for(int j=1;j<=nbrWpm;j++)
        {
            float color = ((float)255)*((float)norme2( Cola(d, (i-1)*nbrWpn+j))/(float)maxd);
            //cout << norme2( Cola(d, (i-1)*nbrWpn+j)) << " " << maxd << endl;
            //cout << color << endl;
            for(int ii=(i-1)*nw+1;ii<=i*nw;ii++)
            {
                for(int jj=(j-1)*mw+1;jj<=j*mw;jj++)
                {
                    imW.set( color, ii, jj);
                    imF12.set( gF1.get(ii,jj), ii+d.get(1,1), jj+d.get(2,1));
                }
            }
        }
    }

    cv::Mat dMap;
    //cv::resize( Mat2cvAt(imW,imW,imW), dMap, size, 1.0/factor, 1.0/factor);
    cv::resize( Mat2cvp(imW,imW,imW), dMap, size, 1.0/factor, 1.0/factor);
    cv::Mat imF12Map;
    //cv::resize( Mat2cvAt(imF12,imF12,imF12), imF12Map, size, 1.0/factor, 1.0/factor);
    cv::resize( Mat2cvp(imF12,imF12,imF12), imF12Map, size, 1.0/factor, 1.0/factor);
    //afficher( &dMap, &imF12Map, &F2);


    /*----------------------------------*/    
    instanceFT.PointDeplacement();
    Mat<float> outputPoints( instanceFT.getOutputPoints());
    for(int i=1;i<=outputPoints.getColumn();i++)
    {
        DrawCircle( &iR2, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
        DrawCircle( &iB2, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
        DrawCircle( &iG2, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
    }

    //rF2 = Mat2cvAt(iR2, iG2, iB2);
    rF2 = Mat2cvp(iR2, iG2, iB2);
    cv::resize(rF2, rF2, size, 1.0/factor, 1.0/factor);

    cout << "L'execution totale avec deplacement et dessin a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
    afficher(&rF2);



    return 1;
}


int main2()
{
    /*FRAME */
    cv::Mat frame;    
    vector<cv::Mat> F;//(640,480, CV_16UC3);
    vector<cv::Mat> rF;//(640,480,CV_16UC3);
    vector<Mat<float> > gF;

    unsigned int nbuff = 3;
    float factor = 1.0/2.0;
    cv::Size size(0,0); // then the output image size is computed thx to factor. round(fx*src.cols), round(fy*src.rows)

    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    float fps = cap.get(CV_CAP_PROP_FPS);
    cap.set(CV_CAP_PROP_FPS, 1);
    cout << "FPS  = " << fps << endl;
    fps = cap.get(CV_CAP_PROP_FPS);
    cout << "FPS  = " << fps << endl;

    cv::namedWindow("Output");
    cv::namedWindow("OutputFT");
    cv::namedWindow("Entry");
    /*--------------------*/

    /*FRAME to MAT*/
    Mat<float> iR, iG, iB;

    vector<Mat<float> > landmarks;
    bool continuer = true;


    clock_t timer1 = clock();
    clock_t timertotal = clock();

    cout << "Initialization : DONE." << endl;

    while(continuer)
    {
        cout << "L'execution totale avec deplacement et dessin a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
        timertotal=clock();
//FRAME :
        cap >> frame;
        F.insert(F.begin(), frame);        
        cv::resize(frame, frame, size, factor, factor);
        rF.insert( rF.begin(), frame);

        if( F.size() >= nbuff)
            F.pop_back();

        if(rF.size() >= nbuff)
            rF.pop_back();

        if(gF.size() >= nbuff)
            gF.pop_back();

        cv::imshow("Entry", F[0]);


        timer1 = clock();
        cv2MatAt( rF[0], &iR,&iG,&iB);
        gF.insert(gF.begin(), ((float)1.0/3)*(iR+iG+iB));
        cout << "L'execution de conversion-images a prise : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
/*-----------------------------------------------*/

        cout << "Initialization : ..." << endl;
        LF<float> ld(gF[0]);

        timer1 = clock();
        ld.search(4, /*step*/ 10);	//FAST  //warning : the landmarks could not be found if the step is too large, from one frame to the next..
        cout << "L'execution de landmark.search a prise : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


        landmarks.insert( landmarks.begin(), ld.getLandmarks());

        cout << "Initialization : DONE." << endl;

        for(int i=1;i<=landmarks[0].getColumn();i++)
        {            
            DrawCircle( &iR, landmarks[0].get( 1,i), landmarks[0].get(2,i));
            DrawCircle( &iB, landmarks[0].get( 1,i), landmarks[0].get(2,i));
            DrawCircle( &iG, landmarks[0].get( 1,i), landmarks[0].get(2,i));
        }
        cout << endl << landmarks[0].getColumn() << " landmarks found.\n" << endl;
        landmarks[0].afficher();

        /*FRAME 1 + FAST :*/
        rF[0] = Mat2cvAt(iR, iG, iB);
        cv::resize(rF[0], rF[0], size, 1/factor, 1/factor);

        cv::imshow("Output", rF[0]);
/*TRACKING*/

        if( F.size() >= 2 && false)
        {
            cout << " Feature tracking : ..." << endl;

            timer1 = clock();
            FT<float> instanceFT( gF[1], gF[0], landmarks[1]);
            cout << "Feature tracking : " << (float)((float)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;


            /*----------------------------------------------------------*/

            vector<Mat<float> > displacement =  instanceFT.getD();
            vector<Mat<float> > wd = instanceFT.getWindow();
            int nw = instanceFT.getnw();
            int mw = instanceFT.getmw();
            int nbrWpn = (int)(((float)gF[0].getLine())/(float)nw +1);
            int nbrWpm = (int)(((float)gF[0].getLine())/(float)mw +1);

            cout << "L'execution totale a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;

            /*--------------------------------------*/
            /*Windows */
            Mat<float> d(displacement[0]);
            for(int i=1;i<=displacement.size()-1;i++)
                    d = operatorL( d, displacement[i]);
            //d.afficher();
            float maxd = max(d);
            //cout << "valeur max : "<<  maxd << endl;
            Mat<float> imW((float)0, gF[0].getLine(), gF[0].getColumn());
            Mat<float> imF12(gF[1]);

            for(int i=1;i<=nbrWpn;i++)
            {
                for(int j=1;j<=nbrWpm;j++)
                {
                    float color = ((float)255)*((float)norme2( Cola(d, (i-1)*nbrWpn+j))/(float)maxd);

                    for(int ii=(i-1)*nw+1;ii<=i*nw;ii++)
                    {
                        for(int jj=(j-1)*mw+1;jj<=j*mw;jj++)
                        {
                            imW.set( color, ii, jj);
                            imF12.set( gF[1].get(ii,jj), ii+d.get(1,1), jj+d.get(2,1));
                        }
                    }
                }
            }

            cv::Mat dMap;
            cv::resize( Mat2cvAt(imW,imW,imW), dMap, size, 1.0/factor, 1.0/factor);
            cv::Mat imF12Map;
            cv::resize( Mat2cvAt(imF12,imF12,imF12), imF12Map, size, 1.0/factor, 1.0/factor);
            //afficher( &dMap, &imF12Map, &F2);


            /*----------------------------------*/


            instanceFT.PointDeplacement();
            Mat<float> outputPoints( instanceFT.getOutputPoints());
            for(int i=1;i<=outputPoints.getColumn();i++)
            {
                DrawCircle( &iR, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
                DrawCircle( &iB, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
                DrawCircle( &iG, outputPoints.get( 1,i), outputPoints.get(2,i), (int)3, (float)50);
            }

            cv::Mat image = Mat2cvAt(iR, iG, iB);
            cv::resize(image, image, size, 1.0/factor, 1.0/factor);

            cout << "L'execution totale avec deplacement et dessin a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
            cv::imshow("OutputFT", image);


        }
/*------------------------------------------------*/



/*EVENT HANDLING */
        if(cv::waitKey(30) >= 0)
            continuer = false;


    }

    return 0;
}


int main3()
{
    float factor = 1.0/2.0;
    cv::Size size(0,0); // then the output image size is computed thx to factor. round(fx*src.cols), round(fy*src.rows)
    cv::Mat frame, frame2H, frame2S;
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    cap >> frame;

    /*
    float fps = cap.get(CV_CAP_PROP_FPS);
    cap.set(CV_CAP_PROP_FPS, 8);
    cout << "FPS  = " << fps << endl;
    fps = cap.get(CV_CAP_PROP_FPS);
    cout << "FPS  = " << fps << endl;
    */


    cv::namedWindow("Entry");
    cv::namedWindow("Output");
    //cv::namedWindow("Gradient");
    //cv::namedWindow("Harris Detector");
    cv::namedWindow("SIFT");
    /*
    string name = "ld";
    for(int i=3;i--;)
    {
        ostringstream num;
        num << i;
        cv::namedWindow(name+num.str());
    }
    */

    cv::imshow("Entry", frame);
    /*--------------------*/

    /*FRAME to MAT*/
    clock_t timertotal = clock();

    /*
    int channels = frame.channels();
    int n = frame.rows;
    int m = frame.cols;
    uchar* p;
    */

    Mat<float> iR,iG,iB;//((float)0,n,m), iG((float)0,n,m), iB((float)0,n,m);

    cout << "Initialization : DONE." << endl;

    bool continuer = true;
    int count = 1000;
    while(continuer && count--)
    {
        cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        //FRAME :
        cap.read(frame);

        cv::resize(frame, frame2H, size, factor, factor);
        cv::GaussianBlur(frame2H,frame2H, cv::Size(0,0), (float)factor,(float)factor);


        cv::imshow("Entry",  frame);

        /*
        for(int i=0;i<n;i++)
        {
            p = frame.ptr<uchar>(i);
            for(int j=0;j<m*channels;j+=3)
            {
                iR.mat[i][(int)((float)j/channels)] = (1.0/3.0)*((float)p[j] + (float)p[j+1] + (float)p[j+2] );
            }

        }

        cout << iR.mat[0][0] << endl;
        */               

        /*
        cv2Matp(frame2H, &iR,&iG,&iB);

        LF<float> instance((1.0/3)*(iR+iG+iB));
        instance.computeGradient((int)3);
        Mat<float> grad(instance.getGrad());
        cv::resize(Mat2cvp(grad,grad,grad), frame2H, size, 1.0/factor, 1.0/factor);
        cv::imshow("Gradient",  frame2H);

        //cv::resize( Mat2cvp(iR,iG,iB), frame2, size, 0.25/factor, 0.25/factor);
        //cv::imshow("Output", frame2);
        */


        /*----------------------------*/
        /*
        int tresh = 150;
        cv::cvtColor(frame2H,frame2H, CV_BGR2GRAY);
        cv::Mat dst, dst_norm, dst_norm_scaled;
        dst = cv::Mat::zeros(frame2H.size(), CV_32FC1);
        int blocksize = 2;
        int aperture = 3;
        float k = 0.04;

        cv::cornerHarris(frame2H, dst,blocksize,aperture,k);

        normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
        convertScaleAbs( dst_norm, dst_norm_scaled );

        for( int j = 0; j < dst_norm.rows ; j++ )
             { for( int i = 0; i < dst_norm.cols; i++ )
                  {
                    if( (int) dst_norm.at<float>(j,i) > tresh )
                      {
                       circle( dst_norm_scaled, cv::Point( i, j ), 3,  cv::Scalar(0), 2, 8, 0 );
                       circle(frame, cv::Point(i*1.0/factor,j*1.0/factor), 5, cv::Scalar(255), 3, 8, 0);
                      }
                  }
             }

        cv::imshow("Harris Detector", dst_norm_scaled);
        cv::imshow("Output", frame);
        */


        /*------------------------------*/
        frame2S = frame2H;
        //SIFT detector(frame2S);
        BRIEFSIFT detector(frame2S,frame);
        int treshold = (factor <= 1.0/10 ? 10*(15.0*factor) : 20);
        int mode = 0; /* not equal ; 1 =*/
        detector.run(treshold, mode);
        int octave = 4;
        vector<Mat<int> > landmarks(detector.getLandmarks());
        Mat<int> features(detector.getFeatures());

        // Add results to image and save.
        cv::Mat output;
        cv::resize(frame2S,output,size,1.0/factor,1.0/factor);
        /*
        for(int oct=octave;oct--;)
        {
            for( int i=landmarks[oct].getLine()+1;i--; )
            {
                for(int j=landmarks[oct].getColumn()+1;j--;)
                {
                    if(landmarks[oct].get(i+1,j+1) != 0)
                    {
                        //circle(output, cv::Point((j+1)*1.0/(2*factor), (i+1)*1.0/(2*factor)), 5, cv::Scalar(255), 3, 8, 0);
                        circle(frame, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 1, 8, 0);
                    }
                }
            }
        }
        */

        //cv::Mat test( Mat2cvp(features, features, features));
        //cv::resize(test,test,size,1.0/factor,1.0/factor);
        //afficher(&test, NULL, NULL, true);
        cv::cvtColor(frame2H,frame2H,CV_BGR2GRAY);

        int count = 0;
        for(int i=features.getLine();i--;)
        {
            for(int j=features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    count++;
                    int max = 0;
                    for(int m=octave;m--;)
                    {
                        int value = landmarks[m].get(i+1,j+1);
                        if( value !=0)
                        {
                            if(landmarks[max].get(i+1,j+1)<value)
                                max = m;
                        }

                    }

                    circle(frame, cv::Point((j+2)*1.0/factor,(i+2)*1.0/factor), 3*(max+1), cv::Scalar(255,0,0), 1, 8, 0);
                    circle(frame2H, cv::Point((j+2),(i+2)), 5, cv::Scalar(255,0,0), 1, 8, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);
                }

            }
        }


        ostringstream text;
        text << count;
        cv::putText(frame, text.str() +string(" Features"), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(0,255));
        cv::imshow("SIFT", frame);
        cv::resize(frame2H,frame2H,size,1.0/factor,1.0/factor);
        cv::imshow("Output", frame2H);

        //cv::imshow("Output", output);

        /*for(int i=4;i--;)
        {
            cv::Mat ldm( Mat2cvp(255*landmarks[i], 255*landmarks[i], 255*landmarks[i]) );
            cv::resize(ldm, ldm,cv::Size(0,0), 1.0/factor,1.0/factor);
            ostringstream num;
            num << i;
            cv::imshow(name+num.str(), ldm);
            if(i!=3)
                i=0;
        }
        */

        /*------------------------------------------------*/
        /*EVENT HANDLING */
                if(cv::waitKey(30) >= 0)
                    continuer = false;


    }
    return 0;
}



int main4()
{
    float factor = 1.0/2.0;
    cv::Size size(0,0); // then the output image size is computed thx to factor. round(fx*src.cols), round(fy*src.rows)
    cv::Mat frame, frame2H, frame2S;
    cv::VideoCapture cap(1);
    int treshold = (factor <= 1.0/10 ? 10*(15.0*factor) : 20);
    int nbrfeatp = 0;
    int nbrfeat = 0;
    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    cap >> frame;

    cv::namedWindow("Entry");
    cv::namedWindow("Output");
    cv::namedWindow("SIFT");

    cv::imshow("Entry", frame);
    /*--------------------*/

    /*FRAME to MAT*/
    clock_t timertotal = clock();
    Mat<float> iR,iG,iB;//((float)0,n,m), iG((float)0,n,m), iB((float)0,n,m);

    cout << "Initialization : DONE." << endl;

    /*---------------------------------------*/

    bool continuer = true;
    int counter = 1000;
    while(continuer && counter--)
    {
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        //timertotal=clock();

        //FRAME :
        cap.read(frame);

        cv::resize(frame, frame2H, size, factor, factor);
        cv::GaussianBlur(frame2H,frame2H, cv::Size(0,0), (float)1.0/2,(float)1.0/2);


        cv::imshow("Entry",  frame);


        /*------------------------------*/
        frame2S = frame2H;
        BRIEFSIFT detector(frame2S,frame);
        int mode = 0; /* not equal ; 1 =*/
        detector.run(treshold, mode);
        int octave = 4;
        vector<Mat<int> > landmarks(detector.getLandmarks());
        Mat<int> features(detector.getFeatures());

        // Add results to image and save.
        cv::Mat output;
        cv::resize(frame2S,output,size,1.0/factor,1.0/factor);
        cv::cvtColor(frame2H,frame2H,CV_BGR2GRAY);

        int count = 0;
        for(int i=features.getLine();i--;)
        {
            for(int j=features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    count++;
                    int max = 0;
                    for(int m=octave;m--;)
                    {
                        int value = landmarks[m].get(i+1,j+1);
                        if( value !=0)
                        {
                            if(landmarks[max].get(i+1,j+1)<value)
                                max = m;
                        }

                    }


                    circle(frame, cv::Point((j+2)*1.0/factor,(i+2)*1.0/factor), 3*(max+1), cv::Scalar(255,0,0), 5, 8, 0);
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);
                }

            }
        }

        nbrfeatp=nbrfeat;
        nbrfeat=count;
        if(nbrfeat <= 10)
            treshold-=2;
        else if(nbrfeat >= 15)
            treshold+=4;


        ostringstream tfps;
        ostringstream tfeatures;
        ostringstream ttresh;
        tfeatures << "Features : " << count;
        ttresh << "Treshold : " << treshold;
        cv::putText(frame, tfeatures.str() , cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,255));
        tfps << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS.";
        cv::putText(frame, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,255));
        cv::putText(frame, ttresh.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,255));
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("SIFT", frame);
        cv::resize(frame2H,frame2H,size,1.0/factor,1.0/factor);
        cv::imshow("Output", frame2H);

        /*------------------------------------------------*/
        /*EVENT HANDLING */
                if(cv::waitKey(30) >= 0)
                    continuer = false;


    }
    return 0;
}

int matchingFAST(cv::Mat* im1_, cv::Mat* im2_, int* treshold_, int* count_, bool continuer,bool rotationInvariance)
{
    //float factor = 8;
    float factor = 2;
    //float vfactor = 1;
    float vfactor = 1.0/2;
    int treshold = (treshold_ != NULL ? *treshold_ : 150);
    cv::Mat im1,im2,im1r,im2r;
    im1 = *im1_; //cv::imread("../SFM_OCV/data1.pgm");
    im2 = *im2_; //cv::imread("../SFM_OCV/data2.pgm");
    //cv::cvtColor(im1,im1,CV_BGR2GRAY);
    //cv::cvtColor(im2,im2,CV_BGR2GRAY);
    cv::resize(im1,im1r,cv::Size(0,0),1.0/factor,1.0/factor);
    cv::resize(im2,im2r,cv::Size(0,0),1.0/factor,1.0/factor);

    //cv::namedWindow("Entry1");
    //cv::namedWindow("Output1");
    if(continuer)
    {
        cv::namedWindow("FAST1");
        //cv::namedWindow("Entry2");
        //cv::namedWindow("Output2");
        cv::namedWindow("FAST2");
    }


    int counter = 1000;


        /*------------------------------*/
        BRIEFFAST detector1(im1r,im1,256,1,15, 20);
        BRIEFFAST detector2(im2r,im2,256,1,15, 20);
        int mode = 0; /* not equal ; 1 =*/
        detector1.run(treshold, mode, rotationInvariance);
        detector2.run(treshold, mode, rotationInvariance);
        Mat<int> features1(detector1.getFeatures());
        Mat<int> features2(detector2.getFeatures());

        // Add results to image and save.
        cv::resize(im1,im1,cv::Size(0,0),(float)vfactor,(float)vfactor);

        /*----------------------------*/
        //Matching
        /*----------------------------*/
        vector<Mat<int> > bstr1 = detector1.getBinaryString();
        vector<Mat<int> > bstr2 = detector2.getBinaryString();
        vector<int> match;
        int matchingTresh = 0;

        if(bstr1.size()!=0 && bstr1.size() != 0)
        {
            matchingTresh = bstr1[0].getColumn()-100;

            for(int k1=bstr1.size();k1--;)
            {
                match.insert(match.begin(), -1);

                for(int k2=bstr2.size();k2--;)
                {
                    if(notIn(match, k2))
                    {
                        if( optim_test(bstr1[k1], bstr2[k2]) )//sum(bstr1[k1]*transpose(bstr2[k2]) ).get(1,1) >= matchingTresh)
                        {
                            match[0] = k2;
                            k2 = 0;
                        }
                    }
                }
            }
        }
        /*----------------------------*/

        int count1 = 0;
        for(int i=features1.getLine();i--;)
        {
            for(int j=features1.getColumn();j--;)
            {
                if(features1.get(i+1,j+1) != 0)
                {
                    count1++;

                    circle(im1, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 5, cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count1;
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);

                    /*match*/
                    if(bstr1.size()!=0 && bstr1.size() != 0)
                    {
                        if(match[count1] > -1)
                        {
                            nbr << "=";
                            nbr << match[count1];
                            cv::putText(im1, nbr.str(), cv::Point((j)*factor*vfactor,(i)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(255));
                        }
                        else
                            cv::putText(im1, nbr.str(), cv::Point((j)*factor*vfactor,(i)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(0,255));
                    }
                }

            }
        }


        *im1_ = im1;

        /*------------------------------------------------*/



        // Add results to image and save.
        cv::resize(im2,im2,cv::Size(0,0),(float)vfactor,(float)vfactor);

        int count2 = 0;
        for(int i=features2.getLine();i--;)
        {
            for(int j=features2.getColumn();j--;)
            {
                if(features2.get(i+1,j+1) != 0)
                {
                    count2++;
                    circle(im2, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 5, cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count2;
                    cv::putText(im2, nbr.str(), cv::Point((j)*factor*vfactor,(i)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)0.4, cv::Scalar(0,255));
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);
                }

            }
        }

        *im2_=im2;




        /*EVENT HANDLING */
        while(continuer && counter--)
        {
            cv::imshow("FAST1", im1);
            cv::imshow("FAST2", im2);

                    if(cv::waitKey(30) >= 0)
                        continuer = false;


        }

        if(count_ != NULL)
            *count_ = (count1>count2?count1:count2);

    return 0;
}

int matching(cv::Mat* im1_, cv::Mat* im2_, int* treshold_, int* count_,bool continuer, bool rotationInvariance)
{    
    //float factor = 2;
    float factor = 2;
    //float vfactor = 2;
    float vfactor = 1;
    int treshold = (treshold_ != NULL ? *treshold_:20);
    cv::Mat im1,im2,im1r,im2r;
    im1 = *im1_; //cv::imread("../SFM_OCV/data1.pgm");
    im2 = *im2_; //cv::imread("../SFM_OCV/data2.pgm");
    //cv::cvtColor(im1,im1,CV_BGR2GRAY);
    //cv::cvtColor(im2,im2,CV_BGR2GRAY);
    cv::resize(im1,im1r,cv::Size(0,0),1.0/factor,1.0/factor);
    cv::resize(im2,im2r,cv::Size(0,0),1.0/factor,1.0/factor);

    //cv::namedWindow("Entry1");
    //cv::namedWindow("Output1");
    if(continuer)
    {
        cv::namedWindow("SIFT1");
        //cv::namedWindow("Entry2");
        //cv::namedWindow("Output2");
        cv::namedWindow("SIFT2");
    }


    int counter = 1000;


        /*------------------------------*/
        int octave = 4;
        BRIEFSIFT detector1(im1r,im1,32,1,8,octave);
        BRIEFSIFT detector2(im2r,im2,32,1,8,octave);
        int mode = 0; /* not equal ; 1 =*/        
        detector1.run(treshold, mode, rotationInvariance);
        detector2.run(treshold, mode, rotationInvariance);
        vector<Mat<int> > landmarks1(detector1.getLandmarks());
        Mat<int> features1(detector1.getFeatures());
        vector<Mat<int> > landmarks2(detector2.getLandmarks());
        Mat<int> features2(detector2.getFeatures());

        // Add results to image and save.
        cv::resize(im1,im1,cv::Size(0,0),(float)vfactor,(float)vfactor);

        /*----------------------------*/
        //Matching
        /*----------------------------*/
        vector<Mat<int> > bstr1 = detector1.getBinaryString();
        vector<Mat<int> > bstr2 = detector2.getBinaryString();
        vector<int> match;
        int matchingTresh = 0;

        if(bstr1.size()!=0 && bstr1.size() != 0)
        {
            matchingTresh = bstr1[0].getColumn()-100;

            for(int k1=bstr1.size();k1--;)
            {
                match.insert(match.begin(), -1);

                for(int k2=bstr2.size();k2--;)
                {
                    if(notIn(match, k2))
                    {
                        if( optim_test(bstr1[k1], bstr2[k2]) )//sum(bstr1[k1]*transpose(bstr2[k2]) ).get(1,1) >= matchingTresh)
                        {
                            match[0] = k2;
                            k2 = 0;
                        }
                    }
                }
            }
        }        
        /*----------------------------*/

        int count1 = 0;
        for(int i=features1.getLine();i--;)
        {
            for(int j=features1.getColumn();j--;)
            {
                if(features1.get(i+1,j+1) != 0)
                {
                    count1++;
                    int max = 0;
                    for(int m=octave;m--;)
                    {
                        int value = landmarks1[m].get(i+1,j+1);
                        if( value !=0)
                        {
                            if(landmarks1[max].get(i+1,j+1)<value)
                                max = m;
                        }

                    }


                    circle(im1, cv::Point((j+1)*factor*vfactor,(i+2)*factor*vfactor), 3*(max+1), cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count1;                    
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);

                    /*match*/
                    if(bstr1.size()!=0 && bstr1.size() != 0)
                    {
                        if(match[count1] > -1)
                        {
                            nbr << "=";
                            nbr << match[count1];
                            cv::putText(im1, nbr.str(), cv::Point((j+1)*factor*vfactor,(i+2)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(255));
                        }
                        else
                            cv::putText(im1, nbr.str(), cv::Point((j+1)*factor*vfactor,(i+2)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(0,255));
                    }
                }

            }
        }


        *im1_ = im1;

        /*------------------------------------------------*/



        // Add results to image and save.
        cv::resize(im2,im2,cv::Size(0,0),(float)vfactor,(float)vfactor);

        int count2 = 0;
        for(int i=features2.getLine();i--;)
        {
            for(int j=features2.getColumn();j--;)
            {
                if(features2.get(i+1,j+1) != 0)
                {
                    count2++;
                    int max = 0;
                    for(int m=octave;m--;)
                    {
                        int value = landmarks2[m].get(i+1,j+1);
                        if( value !=0)
                        {
                            if(landmarks2[max].get(i+1,j+1)<value)
                                max = m;
                        }

                    }


                    circle(im2, cv::Point((j+1)*factor*vfactor,(i+2)*factor*vfactor), 3*(max+1), cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count2;
                    cv::putText(im2, nbr.str(), cv::Point((j+1)*factor*vfactor,(i+2)*factor*vfactor), cv::FONT_HERSHEY_SIMPLEX, (float)1, cv::Scalar(0,255));
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);
                }

            }
        }

        *im2_=im2;




        /*EVENT HANDLING */
        while(continuer && counter--)
        {
            cv::imshow("SIFT1", im1);
            cv::imshow("SIFT2", im2);

                    if(cv::waitKey(30) >= 0)
                        continuer = false;


        }

        if(count_ != NULL)
            *count_ = (count1>count2?count1:count2);

    return 0;
}

int main6()
{
    cv::Mat frame1, frame2;
    vector<cv::Mat> frame;
    cv::VideoCapture cap(1);
    //cv::VideoCapture cap0(0);
    int count = 0;
    int treshold = 20;
    int nbrfeatp = 0;
    int nbrfeat = 0;

    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    /*if(!cap0.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video zero." << endl;
    cap0.open(0);
    if(!cap0.isOpened())
        return -1;
    }*/

    cap >> frame1;
    //cap0 >> frame2;

    cv::namedWindow("Entry");
    cv::namedWindow("Output");

    cv::imshow("Entry", frame1);
    /*--------------------*/
    clock_t timertotal = clock();

    while(frame.size() <= 10)
    {
        cap >> frame1;
        frame.insert( frame.begin(), frame1);
    }
    cout << "Initialization : DONE." << endl;

    /*---------------------------------------*/

    bool continuer = true;
    int counter = 1000;
    while(continuer && counter--)
    {
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        //timertotal=clock();
        //cout << counter << endl;

        //FRAME :
        cap >> frame1;
        //cap0 >> frame2;


        frame.insert(frame.begin(), frame1);

        if(frame.size() >= 20)
               frame.pop_back();
        //cap >> frame2;

        matching(&frame[0],&frame[8], &treshold, &count,false, true);
        frame1 = frame[0];
        frame2 = frame[8];
        //matching(&frame[8],&frame2, &treshold, &count,false, true);
        //frame1 = frame[8];

        cv::imshow("Output",frame2);


        /*------------------------------*/

        nbrfeatp=nbrfeat;
        nbrfeat=count;
        if(nbrfeat <= 20)
            treshold-=2;
        else if(nbrfeat >= 25)
            treshold+=2;


        ostringstream tfps;
        ostringstream tfeatures;
        ostringstream ttresh;
        tfeatures << "Features : " << count;
        ttresh << "Treshold : " << treshold;
        cv::putText(frame1, tfeatures.str() , cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        tfps << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS.";
        cv::putText(frame1, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        cv::putText(frame1, ttresh.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("Entry", frame1);

        /*------------------------------------------------*/
        /*EVENT HANDLING */
                if(cv::waitKey(30) >= 0)
                    continuer = false;


    }

    cout << counter << endl;

    return 0;
}


int main7()
{
    cv::Mat frame1, frame2;
    vector<cv::Mat> frame;
    vector<cv::Mat> frame02;
    cv::VideoCapture cap(1);
    int count = 0;
    int treshold = 200;
    int nbrfeatp = 0;
    int nbrfeat = 0;

    if(!cap.isOpened())
    {
    cerr << "Erreur : Impossible de démarrer la capture video." << endl;
    cap.open(0);
    if(!cap.isOpened())
        return -1;
    }

    cap >> frame1;

    cv::namedWindow("Entry");
    cv::namedWindow("Output");

    cv::imshow("Entry", frame1);
    /*--------------------*/
    clock_t timertotal = clock();

    while(frame.size() <= 10)
    {
        cap >> frame1;
        frame.insert( frame.begin(), frame1);
    }
    cout << "Initialization : DONE." << endl;

    /*---------------------------------------*/

    bool continuer = true;
    int counter = 1000;
    while(continuer && counter--)
    {
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        //timertotal=clock();
        //cout << counter << endl;

        //FRAME :
        cap >> frame1;

        frame.insert(frame.begin(), frame1);
        frame02.insert(frame02.begin(),frame1);

        if(frame.size() >= 10)
               frame.pop_back();
        if(frame02.size() >= 10)
               frame.pop_back();
        //cap >> frame2;

        matchingFAST(&frame[0],&frame02[0], &treshold, &count,false, false);
        frame1 = frame[0];
        frame2 = frame02[0];

        cv::imshow("Output",frame2);


        /*------------------------------*/

        nbrfeatp=nbrfeat;
        nbrfeat=count;
        if(nbrfeat <= 25)
            treshold-=abs(nbrfeatp-nbrfeat);
        else if(nbrfeat >= 30)
            treshold+=abs(nbrfeatp-nbrfeat);

        /*if(nbrfeat <= 3)
            treshold = 100;
            */


        ostringstream tfps;
        ostringstream tfeatures;
        ostringstream ttresh;
        tfeatures << "Features : " << count;
        ttresh << "Treshold : " << treshold;
        cv::putText(frame1, tfeatures.str() , cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        tfps << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS.";
        cv::putText(frame1, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        cv::putText(frame1, ttresh.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("Entry", frame1);

        /*------------------------------------------------*/
        /*EVENT HANDLING */
                if(cv::waitKey(30) >= 0)
                    continuer = false;


    }

    cout << counter << endl;

    return 0;
}

Mat<float> c2pointsMouseClick()
{
   /*FRAME */
    cv::Mat frame;//(640,480, CV_16UC3);
    Mat<float> res(Mouse);
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            return Mat<float>((float)0,1,1);
    }

    cv::namedWindow("Entry");
    cv::setMouseCallback("Entry", CallBackMouseFunc, NULL);
    /*--------------------*/

    bool continuer = true;

    while(continuer)
    {
        cap >> frame;
        cv::imshow("Entry", frame);

        if(cv::waitKey(30) >= 0)
            continuer = false;

        if(update)
        {
            update = false;
            res = Mouse;
        }
    }

    cap.release();

    /*--------------------------*/

    return res;
}

template<typename T>
inline bool notIn(vector<T> set, T elem)
{

    for(int i=set.size();i--;)
    {
        if(set[i] == elem)
            return false;
    }

    return true;
}

bool optim_test(Mat<int> m1, Mat<int> m2)
{
    int tresh = m1.getLine()/1.45;
    bool ret = true;
    int count = 0;

    for(uchar i=m1.getLine();i--;)
    {
        if(m1.get(i+1,1) != m2.get(i+1,1))
            count++;

        if(count > tresh)
        {
            ret = false;
            i=0;
        }
    }

    return ret;
}

cv::Mat process(cv::Mat* im_, Mat<int> *features_, vector<Mat<int> > *binStrings_, vector<cv::Mat> *patchs_, int *treshold_, int *count_, bool rotationInvariance)
{
    //float factor = 2;
    float factor = 4;//18 fps
    //float factor = 8;//10 fps
    //float vfactor = 2;
    float vfactor = 1;
    int treshold = (treshold_ != NULL ? *treshold_:20);
    cv::Mat im,imr,rim;
    im = *im_; //cv::imread("../SFM_OCV/data1.pgm");
    //cv::cvtColor(im1,im1,CV_BGR2GRAY);
    //cv::cvtColor(im2,im2,CV_BGR2GRAY);
    cv::resize(im,imr,cv::Size(0,0),1.0/factor,1.0/factor);    


        /*------------------------------*/
        int octave = 4;
        int patchSize = 10;
        int binLength = 4;
        BRIEFSIFT detector(imr,im,binLength,1,patchSize,octave);
        int mode = 0; /* not equal ; 1 =*/
        detector.run(treshold, mode, rotationInvariance);
        vector<Mat<int> > landmarks(detector.getLandmarks());
        Mat<int> features(detector.getFeatures());
        *patchs_ = detector.getPatchs();
        vector<Mat<int> > bstr = detector.getBinaryString();

        // Add results to image and save.
        cv::resize(im,rim,cv::Size(0,0),(float)vfactor,(float)vfactor);

        int count = 0;
        for(int i=features.getLine();i--;)
        {
            for(int j=features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    count++;
                    int max = 0;
                    for(int m=octave;m--;)
                    {
                        int value = landmarks[m].get(i+1,j+1);
                        if( value !=0)
                        {
                            if(landmarks[max].get(i+1,j+1)<value)
                                max = m;
                        }

                    }


                    circle(rim, cv::Point((j+2)*factor*vfactor,(i+2)*factor*vfactor), 3*(max+1), cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count;
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);

                }

            }
        }


        *im_ = im;


        if(count_ != NULL)
            *count_ = count;

        *binStrings_ = bstr;
        *features_ = features;

        return rim;

}

cv::Mat processFAST(cv::Mat* im_, Mat<int> *features_, vector<Mat<int> > *binStrings_, vector<cv::Mat> *patchs_, int *treshold_, int *count_, bool rotationInvariance)
{
    //float factor = 2;
    float factor = 4;
    //float vfactor = 2;
    float vfactor = 1;
    int treshold = (treshold_ != NULL ? *treshold_:20);
    cv::Mat im,imr,rim;
    im = *im_; //cv::imread("../SFM_OCV/data1.pgm");
    //cv::cvtColor(im1,im1,CV_BGR2GRAY);
    //cv::cvtColor(im2,im2,CV_BGR2GRAY);
    cv::resize(im,imr,cv::Size(0,0),1.0/factor,1.0/factor);


        /*------------------------------*/
    bool show = true;
    int nbrFeatures = 20;
    int patchSize = 100;
    int binLength = 4;
        BRIEFFAST detector(imr,im,binLength,1,patchSize, nbrFeatures, show);
        int mode = 0; /* not equal ; 1 =*/
        detector.run(treshold, mode, rotationInvariance);
        Mat<int> features(detector.getFeatures());
        Mat<int> allFeatures(detector.getAllFeatures());
        *patchs_ = detector.getPatchs();
        vector<Mat<int> > bstr = detector.getBinaryString();

        // Add results to image and save.
        cv::resize(im,rim,cv::Size(0,0),(float)vfactor,(float)vfactor);

        /*----------------------------*/

        int count = 0;
        for(int i=features.getLine();i--;)
        {
            for(int j=features.getColumn();j--;)
            {
                if(features.get(i+1,j+1) != 0)
                {
                    count++;

                    circle(rim, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 5, cv::Scalar(255,0,0), 1, 8, 0);
                    ostringstream nbr;
                    nbr << count;
                    //circle(frame2H, cv::Point((j+2),(i+2)), 1, cv::Scalar(255,0,0), 1, 5, 0);
                    //circle(output, cv::Point((j+2)*1.0/(1*factor)*pow(2,4-oct-1), (i+2)*1.0/(1*factor)*pow(2,4-oct-1)), 3*landmarks[oct].get(i+1,j+1)+1, cv::Scalar((oct-1)*(oct-2)*255,(oct-3)*(oct-1)*255,(oct-3)*(oct-2)*255), 2, 8, 0);

                }

                if(allFeatures.get(i+1,j+1) != 0)
                {
                    circle(rim, cv::Point((j)*factor*vfactor,(i)*factor*vfactor), 2, cv::Scalar(0,255,0), 1, 8, 0);
                }

            }
        }


        *im_ = im;


        if(count_ != NULL)
            *count_ = detector.getNbrLandmarksFound();

        *binStrings_ = bstr;
        *features_ = features;

        return rim;

}

int main8(bool fast)
{
    clock_t timertotal = clock();
    cv::Mat frame,frame1;
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            return -1;
    }

    cv::namedWindow("Entry");
    cv::namedWindow("Output");
    /*--------------------*/

    //containers
    vector<Mat<int> > features_ct;
    vector<vector<Mat<int> > > binStrings_ct;
    vector<vector<cv::Mat> > patchs_ct;


    bool continuer = true;
    Mat<int> features;
    vector<Mat<int> > binStrings;
    vector<cv::Mat> patchs;
    int treshold = (fast ? 100 : 50);
    int count = 0;
    bool invariance = false;

    int nbrfeat = 0;
    int nbrfeatp = 0;

    while(continuer)
    {                
        cap >> frame;

        frame1 = (fast ? processFAST(&frame, &features, &binStrings, &patchs, &treshold, &count, invariance) : process(&frame, &features, &binStrings, &patchs, &treshold, &count, invariance) );

        //features_ct.insert(features_ct.begin(), features);
        //binStrings_ct.insert(binStrings_ct.begin(), binStrings);
        patchs_ct.insert(patchs_ct.begin(), patchs);

        Mat<int> matchs((int)0,2,count);
        if(patchs_ct.size() >= 2)
        {
            seekMatchs(patchs_ct, &matchs);
        }
        afficher(&(patchs_ct[0][0]),NULL,NULL,true,(float)30);

        cv::imshow("Entry", frame);
        if(cv::waitKey(30)>=0)
            continuer = false;

        nbrfeatp=nbrfeat;
        nbrfeat=count;
        if(!fast)
        {
            if(nbrfeat <= 30)
                treshold-=abs(nbrfeatp-nbrfeat)/100+2;
            else if(nbrfeat >= 40)
                treshold+=abs(nbrfeatp-nbrfeat)/100+2;
        }
        else
        {
            if(nbrfeat <= 30)
                treshold-=abs(nbrfeatp-nbrfeat)/100+2;
            else if(nbrfeat >= 40)
                treshold+=abs(nbrfeatp-nbrfeat)/100+2;

        }


        ostringstream tfps;
        ostringstream tfeatures;
        ostringstream ttresh;
        tfeatures << "Features : " << count;
        ttresh << "Treshold : " << treshold;
        cv::putText(frame1, tfeatures.str() , cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        tfps << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS.";
        cv::putText(frame1, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        cv::putText(frame1, ttresh.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("Output", frame1);        

    }

    cap.release();


    return 0;
}


void seekMatchs(const vector<vector<cv::Mat> > patchs_ct, Mat<int> *matchs)
{
    //only between the last two frames.
    int nbr_patchF1 = patchs_ct[0].size();
    int nbr_patchF2 = patchs_ct[1].size();

    Mat<float> patch_matching_error((float)0, nbr_patchF1, nbr_patchF2);

    for(int i=0;i<nbr_patchF1;i++)
    {
        for(int j=0;j<nbr_patchF2;j++)
        {            
            patch_matching_error.set( (float)sum( sum( absM(patchs_ct[0][i]-patchs_ct[1][j]) ) ).at<float>(1,1), i+1,j+1);            
        }

        int idmin = 0;
        int errmin = patch_matching_error.get(i,1);
        for(int j=0;j<nbr_patchF2;j++)
        {
            if(patch_matching_error.get(i,j) < errmin)
            {
                idmin = j;
                errmin = patch_matching_error.get(i,j);
            }
        }

        matchs->set(i,1,i+1);
        matchs->set(idmin, 2,i+1);
        //attention, décalage entre indice de colonne et indice de patch.
    }

    //patch_matching_error.afficher();
    //patch_matching_error = (255.0/max(patch_matching_error) )*patch_matching_error;
    //cv::Mat show = Mat2cvp( patch_matching_error, patch_matching_error, patch_matching_error);
    //afficher(&show,NULL,NULL,true,(float)10);

    //matchs->afficher();


}

//#define v_bench
int main9()
{
    int h = 120;
    int w = 160;
    int delta = 3;
    Mat<float> im1(0.0,h,w);
    Mat<float> im2(im1);
    Mat<float> dm1(0.0,h,w);
    Mat<float> var_idm1(0.5,h,w);

    //Benchmark creation
    for(int i=h/10;i<=h-h/10;i++)
    {
        im1.set(100.0, i, 4*w/10);
        im1.set(100.0, i, 4*w/10+1);
        im1.set(100.0, i, 4*w/10+2);
        im1.set(100.0, i, 4*w/10+3);
    }
    for(int i=h/10;i<=h-h/3;i++)
    {
        im1.set(100.0, i, w/10);
        im1.set(100.0, i, w/10+1);
        im1.set(100.0, i, w/10+2);
        im1.set(100.0, i, w/10+3);
    }
    /*
    for(int j=w/10;j<=w-w/10;j++)
    {
        im1.set(100.0, 8*h/10, j);
        im1.set(100.0, 8*h/10+1, j);
        im1.set(100.0, 8*h/10+2, j);
        im1.set(100.0, 8*h/10+3, j);
    }
    */
    for(int i=h/10;i<=h-h/10;i++)
    {
        im2.set(100.0, i, 4*w/10+delta);
        im2.set(100.0, i, 4*w/10+1+delta);
        im2.set(100.0, i, 4*w/10+2+delta);
        im2.set(100.0, i, 4*w/10+3+delta);
    }    
    for(int i=h/10;i<=h-h/3;i++)
    {
        im2.set(100.0, i, w/10+delta);
        im2.set(100.0, i, w/10+1+delta);
        im2.set(100.0, i, w/10+2+delta);
        im2.set(100.0, i, w/10+3+delta);
    }
    /*
    for(int j=w/10;j<=w-w/10;j++)
    {
        im2.set(100.0, 8*h/10+delta, j);
        im2.set(100.0, 8*h/10+1+delta, j);
        im2.set(100.0, 8*h/10+2+delta, j);
        im2.set(100.0, 8*h/10+3+delta, j);
    }
    */

    for(int i=h/10;i<=h-h/10;i++)
    {
        dm1.set(1.0/110.0, i, 4*w/10);
        dm1.set(1.0/110.0, i, 4*w/10+1);
        dm1.set(1.0/110.0, i, 4*w/10+2);
        dm1.set(1.0/110.0, i, 4*w/10+3);
    }    
    for(int i=h/10;i<=h-h/3;i++)
    {
        dm1.set(1.0/110.0, i, w/10);
        dm1.set(1.0/110.0, i, w/10+1);
        dm1.set(1.0/110.0, i, w/10+2);
        dm1.set(1.0/110.0, i, w/10+3);
    }

    /*
    for(int j=w/10;j<=w-w/10;j++)
    {
        dm1.set(1.0/110.0, 8*h/10, j);
        dm1.set(1.0/110.0, 8*h/10+1, j);
        dm1.set(1.0/110.0, 8*h/10+2, j);
        dm1.set(1.0/110.0, 8*h/10+3, j);
    }
    */
    //------------------

    Frame<float> f1(im1,1);
    Frame<float> f2(im2,1);

    Mat<float> K(0.0,3,3);
    K.set(500,1,1); //fx
    K.set(500,2,2); //fy
    K.set((float)h/2,1,3); //cx
    K.set((float)w/2,2,3); //cy
    K.set(1.0,3,3);
    Mat<float> invK(invGJ(K));

#ifdef v_bench
    cout << "Intrinsic Matrix : " << endl;
    K.afficher();
    cout << "Inverse Intrinsic Matrix : " << endl;
    invK.afficher();

    afficherMat<float>(&im1,&im2, &dm1, false, 2.0);
#endif


    clock_t timer = clock();
    int it = 5;

    //OODirectSlamSIM3<float> instanceOO(&f1,&f2,&K,&invK,&dm1);
    OODirectSlamSE3<float> instanceOO(&f1,&f2,&K,&invK,&dm1, &var_idm1);
    //bool approxHessian = true;
    //ONSCNewton<float> instanceONSC( &instanceOO, it, approxHessian);
    bool optimstep = true;
    ONSC<float> instanceONSC( &instanceOO, it,optimstep);
    cout << " L'execution a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;

    if(it <= 30)
    {
        //regardons l'evolution de notre variable :
        Mat<float> stack(instanceONSC.getX(it));
        for(int i=it;i--;)
            stack = operatorL(stack, instanceONSC.getX(i));
        stack.afficher();
    }

    Mat<float> X( instanceONSC.getX(it));
    //float s = X.get(7,1);
    //Mat<float> R(s*expW( extract(X,1,1,3,1) ) );
    Mat<float> R(expW( extract(X,1,1,3,1) ) );
    //cout << "rotation matrix : " << s << endl;
    R.afficher();
    Mat<float> t(extract(X,4,1,6,1) );
    cout << "translation vector : " << endl;
    //t.set( 5.0, 3,1);
    //t.set( 0.5,2,1);
    //t.set( 0.2,1,1);
    t.afficher();

    Mat<float> pos(0.0,3,1);
    Mat<float> im(0.0,10*h,10*w);

    for(int i=1;i<=h;i++)
    {
        for(int j=1;j<=w;j++)
        {
            if(dm1.get(i,j) != 0.0)
            {
                pos.set(i,1,1);
                pos.set(j,2,1);
                pos.set(1.0,3,1);

                //pos.afficher();

                pos = invK*pos;

                //cout << " Renvoye vers : " << endl;
                //pos.afficher();                

                pos = (R*pos+t);

                float norme = norme2(pos);
                pos = ((float)1.0/norme)*pos;
                pos = ((float)1.0/dm1.get(i,j))*pos;

                pos = K*pos;

                //homogeneousNormalization(&pos);
                if(pos.get(3,1) != (float)0.0 && pos.get(3,1) != (float)1.0)
                    pos = ((float)1.0/pos.get(3,1))*pos;

                //pos.afficher();
                //cout << "////////////////////////////////" << endl;


                im.set( im1.get(i,j), 10*h/2+pos.get(1,1), 10*w/2+pos.get(2,1) );
            }
        }

    }

    cout << "/////////////////////////////////////////" << endl;
    cout << "Pose estimee : " << endl;
    expM(X).afficher();
    cout << "/////////////////////////////////////////" << endl;
    afficherMat(&im,&f1,&f2,false,2.0);

    Mat<float> imRef2(extract(im,10*h/2+1,10*w/2+1,10*h/2+h,10*w/2+w)-f2);
    afficherMat(&imRef2, &f1,&f2,false,2.0);

    /*Pyramidal computation test*/
    /*
    Mat<float> kernel(1.0,4,4);
    Mat<float> pIm( pooling(im,kernel,2) );
    afficherMat(&pIm,&im,&imRef2,false,5.0);
    */

    /*Epipolar Line Computation test*/    
    cout << "Epipolar Line Computation test..." << endl;
    float numlim = 1000.0/numeric_limits<float>::epsilon();
    Mat<float> epi;
    Mat<float> epi1;
    Mat<float> x(1.0,3,1);
    x.set((float)h/10,1,1);
    x.set(4*(float)w/10,2,1);
    computeEpipolarLine2(&epi1,&dm1, &x, &R, &t, &K, &invK );
    computeEpipolarLine(&epi, &x, &R, &t, &K, &invK );
    epi.afficher();
    epi1.afficher();


    cv::Scalar color(255,255,255);
    cv::Scalar color1(100,100,100);
    cv::Mat imEpi( Mat2cvp( imRef2, imRef2, imRef2) );
    //cv::line(imEpi,cv::Point(x.get(1,1)-numlim*epi.get(1,1), x.get(2,1)-numlim*epi.get(2,1)), cv::Point(x.get(1,1)+ numlim*epi.get(1,1),x.get(1,1)+numlim*epi.get(2,1)), color);
    cv::line(imEpi,cv::Point(x.get(1,1)-numlim*epi1.get(1,1), x.get(2,1)-numlim*epi1.get(2,1)), cv::Point(x.get(1,1)+ numlim*epi1.get(1,1),x.get(1,1)+numlim*epi1.get(2,1)), color1);
    //cv::line(imEpi,cv::Point(x.get(1,1), x.get(2,1)), cv::Point(x.get(1,1)+10,x.get(1,1)+20), color);
    afficher(&imEpi,NULL,NULL,false,2.0);



    return 0;
}


Mat<float> poseEstimation(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Mat<float>* K, Mat<float>* invK)
{
    clock_t timer = clock();
    int it = 5;

    OODirectSlamSIM3<float> instanceOO(f1,f2,K,invK,dm1);
    //OODirectSlamSE3<float> instanceOO(f1,f2,K,invK,dm1);
    bool optimstep = true;
    ONSC<float> instanceONSC( &instanceOO, it, optimstep);
    //bool approxHessian = true;
    //ONSCNewton<float> instanceONSC( &instanceOO, it, approxHessian);
    cout << " L'execution a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;

    Mat<float> X( instanceONSC.getX(it));

#ifdef verbose
    float s = X.get(7,1);
    Mat<float> R(s*expW( extract(X,1,1,3,1) ) );
    Mat<float> t(extract(X,4,1,6,1) );


    cout << "rotation matrix : " << s << endl;
    R.afficher();
    cout << "translation vector : " << endl;
    t.afficher();
#endif

    return expMSIM3(X);
    //return expM(X);
}



Mat<float> initializeInvDM(Frame<float>* frame, float gradIntTreshold, float variance, float mean)
{
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
                idm.set( (float)mean+(float)gen.dev(), i+1,j+1);
                //initialized inverse depth between -0.9 and 1.1.
            }

        }
    }

    //TODO : regularization...

    return idm;
}


inline cv::Mat computeGradient(cv::Mat* im)
{
    int ddepth = CV_16S;
    int scale = 1;
    int delta = 0;

    if(im->channels() != 1)
        cv::cvtColor(*im,*im,CV_BGR2GRAY);
    cv::GaussianBlur(*im,*im,cv::Size(0,0),(float)20.0,(float)20.0);


    vector<cv::Mat> grad;
    grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type()) );
    grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type()) );

    // Gradient X
    Sobel( *im, grad[0], ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
    convertScaleAbs( grad[0], grad[0] );

    // Gradient Y
    Sobel( *im, grad[1], ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
    convertScaleAbs( grad[1], grad[1] );

    return ((float)(0.5))*(grad[0]+grad[1]);
    /*
    cv::Mat grad;
    Sobel(*im,grad,ddepth,1,1,3,scale,delta,cv::BORDER_DEFAULT);
    //cv::GaussianBlur(grad,grad,cv::Size(0,0),(float)1.0/sqrt(2),(float)1.0/sqrt(2));
    convertScaleAbs(grad,grad);
    return absMAt(grad);
    */
}

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



inline void computeEpipolarLine(Mat<float>* l, Mat<float>* x, Mat<float>* R, Mat<float>* t, Mat<float>* K, Mat<float>* invK)
{
    *l = (crossProduct((*K)*(((float)(1.0))*(*t))) * ((*K)* ((*R)* ((*invK) * (*x)))) );
    homogeneousNormalization(l);
}

inline void computeEpipolarLine2(Mat<float>* l, Mat<float>* invDepthMap1, Mat<float>* x, Mat<float>* R, Mat<float>* t, Mat<float>* K, Mat<float>* invK)
{
    float lambda1 = (float)1.0/invDepthMap1->get(x->get(1,1),x->get(2,1));
    float lambda2 = (float)1.0/invDepthMap1->get(x->get(1,1),x->get(2,1)) + 100;//1.0/numeric_limits<float>::epsilon();
    Mat<float> retroprojx = (*invK)*(*x);
    //here is hC = RCW * hW
    // let us  have h features in the frame of the second camera :
    Mat<float> Xlambda1 = (*R)*retroprojx + (*t);
    Mat<float> Xlambda2 = (*R)*retroprojx + (*t);

    //TEST :    
    //the direction is not normalized...??
    float norme = norme2(Xlambda1);
    Xlambda1 = ((float)1.0/norme)*Xlambda1;
    Xlambda2 = ((float)1.0/norme)*Xlambda2;
    
    //now we just have to it the correct distance : X features in the second camera frame :
    Xlambda1 = lambda1*Xlambda1;
    Xlambda2 = lambda2*Xlambda2;
    //let us project them again :
    Mat<float> x1_2 = (*K)*Xlambda1;
    Mat<float> x2_2 = (*K)*Xlambda2;
    // and finally we have to extract the pixel coordinate out of that hC :
    homogeneousNormalization(&x1_2);
    homogeneousNormalization(&x2_2);
    // here is our line joining the two points within the pixel frame :
    *l = x1_2-x2_2;
    l->set((float)1.0,3,1);
}


inline float exhaustiveSearchEpipolarLine(int nbrHypSearch, Frame<float>* f1, Frame<float>* f2, Mat<float>* invDepthMap1, Mat<float>* x1, Mat<float>* x2, Mat<float>* R, Mat<float>* t, float var_invDepthMap, Mat<float>* K, Mat<float>* invK, float factor)
{
    float lambda;
    Mat<float> Xlambda(3,1);
    Mat<float> x_2(3,1);
    int nbrHyp = nbrHypSearch;
    float varStep = ((float)10.0/nbrHyp)*var_invDepthMap;
    Mat<float> photoDist(1,1);
    Mat<float> minphotoDist( absM(f1->get(*x1)-f2->get(*x2)) );
    float minlambda = (float)1.0/invDepthMap1->get(x1->get(1,1),x1->get(2,1));
    //cv::Scalar colorLine1(255,0,0);
    //cv::Scalar colorLine2(0,0,255);
    //cv::Scalar colorPoint(0,255,0);
    //cv::Mat debugIm( Mat2cvp(*f2,*f2,*f2));

    for(int i=nbrHyp+1;i--;)
    {
        lambda = (float)1.0/(invDepthMap1->get(x1->get(1,1),x1->get(2,1)) - 2.0*var_invDepthMap + i*varStep );
        //here is hW = RWC * hC + rWC
        // let us  have h features in the frame of the second camera :
        Xlambda.set( lambda * (invK->get(1,1)*x1->get(1,1) + invK->get(1,3) ),1,1);
        Xlambda.set( lambda * (invK->get(2,2)*x1->get(2,1) + invK->get(2,3) ),2,1);
        Xlambda.set((float)lambda,3,1);
        
        Xlambda = (*R)*Xlambda + (*t);

	//TEST :
        //the direction is not normalized...???
	//float norme = norme2(Xlambda);
        //Xlambda = ((float)1.0/norme)*Xlambda;
        
        
        //now we just have to it the correct distance : X features in the second camera frame :
        //Xlambda = lambda*Xlambda;

        //let us project them again :
        x_2 = (*K)*Xlambda;
        // and finally we have to extract the pixel coordinate out of that hC :
        //homogeneousNormalization(&x_2);
        x_2.mat[0][0] = x_2.mat[0][0]/x_2.mat[0][2];
        x_2.mat[0][1] = x_2.mat[0][1]/x_2.mat[0][2];
        x_2.mat[0][2] = (float)1.0;


        photoDist = absM( f1->get(*x1)-f2->get(x_2) );

        if(photoDist < minphotoDist)
        {
            minlambda = lambda;
            minphotoDist = photoDist;
        }
    }



    /*
    Xlambda = (*R)*((*invK)*(*x1))+(*t) ;
    Xlambda = (float)(1.0/norme2(Xlambda))*Xlambda;
    Xlambda = minlambda*Xlambda;
    x_2 = (*K)*Xlambda;
    homogeneousNormalization(&x_2);
    Mat<float> line;
    computeEpipolarLine2(&line, invDepthMap1, x1, R, t, K, invK );
    cv::line(debugIm,
             cv::Point((1.0/factor)*x_2.get(1,1)-10000*line.get(1,1), (1.0/factor)*x_2.get(2,1)-10000*line.get(2,1)),
             cv::Point((1.0/factor)*x_2.get(1,1)+ 10000*line.get(1,1), (1.0/factor)*x_2.get(1,1)+10000*line.get(2,1)),
             colorLine2);
    computeEpipolarLine(&line, x1, R, t, K, invK );
    cv::line(debugIm,
             cv::Point((1.0/factor)*x_2.get(1,1)-10000*line.get(1,1), (1.0/factor)*x_2.get(2,1)-10000*line.get(2,1)),
             cv::Point((1.0/factor)*x_2.get(1,1)+ 10000*line.get(1,1), (1.0/factor)*x_2.get(1,1)+10000*line.get(2,1)),
             colorLine1);
    cv::circle(debugIm,cv::Point(x_2.get(1,1),x_2.get(2,1)),(int)5,colorPoint);

    afficher(&debugIm,NULL,NULL,true,1.0/ factor);
    */


    return minlambda;
}

float exhaustiveSearchEpipolarLine2( Frame<float>* f1, Frame<float>* f2, Mat<float>* invDepthMap1, Mat<float>* x1, Mat<float>* x2, Mat<float>* R, Mat<float>* t, float var_invDepthMap, Mat<float>* K, Mat<float>* invK, float factor)
{

    float lambda;
    Mat<float> Xlambda;
    Mat<float> x_2;
    int nbrHyp = 20;
    Mat<float> photoDist;
    Mat<float> minphotoDist( absM(f1->get(*x1)-f2->get(*x2)) );
    float minlambda = (float)1.0/invDepthMap1->get(x1->get(1,1),x1->get(2,1));
    cv::Scalar colorLine1(255,0,0);
    cv::Scalar colorLine2(0,0,255);
    cv::Scalar colorPoint(0,255,0);
    cv::Mat debugIm( Mat2cvp(*f2,*f2,*f2));

    Mat<float> epiline(3,1);
    computeEpipolarLine(&epiline,x1,R,t,K,invK);
    float h = f1->getLine();
    float w = f1->getColumn();
    float varStepPixel = sqrt(h*h+w*w)/(2*nbrHyp);


    /*
    for(int i=0;i<=nbrHyp;i++)
    {
        lambda =1.0/(invDepthMap1->get(x1->get(1,1),x1->get(2,1)) - 2.0*var_invDepthMap + i*varStep );
        Xlambda = (*R)*((*invK)*(*x1) + lambda*(*t) );
        x_2 = (*K)*Xlambda;
        homogeneousNormalization(&x_2);

        photoDist = absM( f1->get(*x1)-f2->get(x_2) );

        if(photoDist < minphotoDist)
        {
            minlambda = lambda;
            minphotoDist = photoDist;
        }
    }
*/

    /*
    Xlambda = (*R)*((*invK)*(*x1)+minlambda*(*t) );
    x_2 = (*K)*Xlambda;
    homogeneousNormalization(&x_2);
    Mat<float> line;
    computeEpipolarLine2(&line, invDepthMap1, x1, R, t, K, invK );
    cv::line(debugIm,
             cv::Point((1.0/factor)*x_2.get(1,1)-10000*line.get(1,1), (1.0/factor)*x_2.get(2,1)-10000*line.get(2,1)),
             cv::Point((1.0/factor)*x_2.get(1,1)+ 10000*line.get(1,1), (1.0/factor)*x_2.get(1,1)+10000*line.get(2,1)),
             colorLine2);
    computeEpipolarLine(&line, x1, R, t, K, invK );
    cv::line(debugIm,
             cv::Point((1.0/factor)*x_2.get(1,1)-10000*line.get(1,1), (1.0/factor)*x_2.get(2,1)-10000*line.get(2,1)),
             cv::Point((1.0/factor)*x_2.get(1,1)+ 10000*line.get(1,1), (1.0/factor)*x_2.get(1,1)+10000*line.get(2,1)),
             colorLine1);
    cv::circle(debugIm,cv::Point(x_2.get(1,1),x_2.get(2,1)),(int)5,colorPoint);
    */
    //afficher(&debugIm,NULL,NULL,false,1.0/ factor);


    return minlambda;
}



int main10(int argc, char* argv[])
{
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
    	
    int count = 0;
    float fps = (float)4.0;
    float goalfps = (float)2.0;
    float coeff = (float)1.0/factor/2;
    float fpserr = (float)0.0;
    bool fpsoptimization = false;
    bool debug =true;
    bool debugrotonly = true;
    bool grad = true;
    //Mat<float> khi(numeric_limits<float>::epsilon(),6,1);
    Mat<float> khi(numeric_limits<float>::epsilon(),7,1);
    khi.set((float)1,7,1);

    Mat<float> deltaPose( expMSIM3(khi) );
    //Mat<float> pose( expM(khi) );
    vector<Mat<float> > pose;
    Mat<float> globalPose(deltaPose);
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
    		
    cout << "Usage : " << argv[0] << " mode=12 gradTresh=50 invfactor=4 pyramidDepth=3 nbrHypSearch=10 withMultipleKF=0:false;1:true meanDepth=0.9 initVarDepth=0.5" << endl; 		
    int nbrFramePerKF = 4;//ceil(1.0/ceil(fps))*4;
    int countFramePerKF = 0;
    
    cv::Mat debugIm;
    cv::Scalar colorbleue(255,0,0);
    cv::Scalar colorvert(0,255,0);


    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            return -1;
    }

    
    cv::namedWindow("Entry");
    if(debug)
	cv::namedWindow("DEBUG");    
    //--------------------

    //containers



    bool continuer = true;
    for(int i=0 ; i<= 20; i++)  cap >> frame1;
    cap >> frame1;
    for(int i=0 ; i<= 20; i++)  cap >> frame2;

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
    if(grad)
    {
    	f1 = cv2Matp<float>( frame1 );
    	f1Grad = cv2Matp<float>( computeGradientXY( &frame1, &f1gradX, &f1gradY) );
    	f1GradX = cv2Matp<float>( f1gradX);
    	f1GradY = cv2Matp<float>( f1gradY);
    }
    else
    {
    	f1 = cv2Matp<float>( computeGradient( &frame1) );
    }
    	    
    Frame<float> f2(f1);
    Frame<float> f2GradX, f2GradY, f2Grad;
    cv::Mat f2gradX;
    cv::Mat f2gradY;    
    
    int h = f1.getLine();
    int w = f1.getColumn();


    float mean = 0.9;
    float initVariance = 1.0;
    
    if(argc>7)
    	mean = atof(argv[7]);
    if(argc>8)
    	initVariance = atof(argv[8]);
    	
    Mat<float> invDepthMap1;
    if(grad)
    	invDepthMap1 =  initializeInvDM(&f1Grad, gradIntTreshold, initVariance, mean);
    else
    	invDepthMap1 = initializeInvDM(&f1, gradIntTreshold, initVariance, mean);
    	
    Mat<float> invDepthMap2(invDepthMap1);
    float depthVar_init = (float)0.5;
    Mat<float> var_invDepthMap1(depthVar_init,h,w);
    Mat<float> var_invDepthMap2(depthVar_init,h,w);

    Mat<float> x1( (float)1.0,3,1);
    Mat<float> x2(x1);
    Mat<float> line2;
    Mat<float> line22;
    float numlim = (float)1.0/pow(numeric_limits<float>::epsilon(),10);


    Mat<float> K((float)0.0,3,3);
    //K.set((float)-0.000876,1,1); //fx
    //K.set((float)-0.000018459,2,2); //fy
    //K.set((float)-0.4445,1,3); //cx
    //K.set((float)0.900336,2,3); //cy
    K.set((float)530*factor,1,1); //fx
    K.set((float)700*factor,2,2); //fy
    K.set((float)w/2,1,3); //cx
    K.set((float)h/2,2,3); //cy
    K.set((float)1.0,3,3);
    Mat<float> invK(invGJ(K));
    Mat<float> rot(1,1);
    Mat<float> invRot(1,1);
    Mat<float> t((float)0,3,1);

    float dist = (float)0;
    float variance = (float)0;
    while(continuer)
    {        
        count = 0;
        cap >> frame;
        countFramePerKF++;
        nbrFrame++;
        
        cap >> debugIm;
        cv::resize(frame,frame2,cv::Size(0,0),factor,factor);
        framelist.insert(framelist.end(), frame2);
        
        if(grad)
	{
	    f2 = cv2Matp<float>( frame2 );
	    f2Grad = cv2Matp<float>( computeGradientXY( &frame2, &f2gradX, &f2gradY) );
	    f2GradX = cv2Matp<float>( f2gradX);
	    f2GradY = cv2Matp<float>( f2gradY);
	}
	else
	{
	    f2 = cv2Matp<float>( computeGradient( &frame2) );
	}


        //deltaPose = poseEstimation(&f1, &invDepthMap1,&f2, &K, &invK);
        
        
        if(grad)
        {
        	deltaPose = poseEstimationLSGRAD(&f1, &invDepthMap1,&f2, &f2GradX, &f2GradY, &K, &invK, &var_invDepthMap1, &ResidualImage, pyramidDepth);
        }
        else
        {
        	deltaPose = poseEstimationLS(&f1, &invDepthMap1,&f2, &K, &invK, &var_invDepthMap1, &ResidualImage);
        }
        
        //pose = pose*deltaPose;
        pose.insert(pose.end(),deltaPose);        
        
        rot = extract(deltaPose,1,1,3,3);
        invRot = transpose(rot);
        // rot € SO(3) !!!
        if(!debugrotonly)
        	t = extract(deltaPose,1,4,3,4);
        
        deltaPose.afficher();
	timerdepth = clock();

        //TODO : depth map propagation...
        for(int i=invDepthMap1.getLine();i--;)
        {
            x1.set(i+1,1,1);

            for(int j=invDepthMap1.getColumn();j--;)
            {

                if(invDepthMap1.get(i+1,j+1) != 0.0 )
                {
                    count++;
                    dist = (float)1.0/invDepthMap1.get(i+1,j+1);
                    variance = var_invDepthMap1.get(i+1,j+1);
                    x1.set(j+1,2,1);

                    x2 = invK*x1;

                    homogeneousNormalization(&x2);
                    x2 = ((float)1.0/invDepthMap1.get(i+1,j+1))*x2;
                    x2 = x2 + t;
                    x2 = rot*x2;
                    x2 = K*x2;

                    /*
                    x2 = rot*x2 + t;
                    float norme = norme2(x2);
                    x2 = ((float)1.0/norme)*x2;
                    x2 = ((float)1.0/invDepthMap1.get(i+1,j+1))*x2;
                    x2 = K*x2;
                    */

                    homogeneousNormalization(&x2);

                    if(f1.get(x1).get(1,1) >= (float)gradIntTreshold)
                    {
                        //i) compute epipolar line
                        //Method 1:                        
                        
                        computeEpipolarLine(&line2, &x2, &rot, &t, &K, &invK );
                        cv::line(debugIm,
                                 cv::Point(((float)1.0/factor)*x2.get(1,1)-((float)1.0/factor)*100000*line2.get(1,1), ((float)1.0/factor)*x2.get(2,1)-((float)1.0/factor)*100000*line2.get(2,1)),
                                 cv::Point(((float)1.0/factor)*x2.get(1,1)+ ((float)1.0/factor)*100000*line2.get(1,1), ((float)1.0/factor)*x2.get(1,1)+((float)1.0/factor)*100000*line2.get(2,1)),
                                 colorbleue);
                        

                        //Method 2:                        
                        
                        computeEpipolarLine2(&line22,&invDepthMap1, &x2, &rot, &t, &K, &invK );                        
                        cv::line(debugIm,
                                 cv::Point(((float)1.0/factor)*x2.get(1,1)-((float)1.0/factor)*numlim*line22.get(1,1), ((float)1.0/factor)*x2.get(2,1)-((float)1.0/factor)*numlim*line22.get(2,1)),
                                 cv::Point(((float)1.0/factor)*x2.get(1,1)+ ((float)1.0/factor)*numlim*line22.get(1,1), ((float)1.0/factor)*x2.get(1,1)+((float)1.0/factor)*numlim*line22.get(2,1)),
                                 colorvert);

                        

                        //ii) exhaustive search of intensity match
                        //iii) depth update with translation along the optical axis (z)
                        float estimatedDepth = (float)1.0/(exhaustiveSearchEpipolarLine(nbrHypSearch,
                        							   &f1,
                                                                                   &f2,
                                                                                   &invDepthMap1,
                                                                                   &x1,
                                                                                   &x2,
                                                                                   &rot,
                                                                                   &t,
                                                                                   var_invDepthMap1.get(i+1,j+1),
                                                                                   &K,
                                                                                   &invK,
                                                                                   factor));
                        float predictedDepth = (float)1.0/((float)1.0/invDepthMap1.get(i+1,j+1) - t.get(3,1) );

                        //mean of the means of the estimation and the prediction.

                        //iv) covariance update :
                        float varpred = var_invDepthMap1.get(i+1,j+1);
                        float varest = depthVar_init;
                        float varOFFSET = abs(predictedDepth-estimatedDepth);
                        //TODO : estimate the initialization variance for each estimation step ?
                        var_invDepthMap1.set(  /*varOFFSET+*/(varpred*varest)/(varpred+varest), x1.get(1,1), x1.get(2,1) );
                        //var_invDepthMap1.set(  (varOFFSET*var_invDepthMap1.get(x1.get(1,1), x1.get(2,1)))/(varOFFSET+var_invDepthMap1.get(x1.get(1,1), x1.get(2,1))), x1.get(1,1), x1.get(2,1) );

                        //fusion of the prior normal distribution variance and the estimation variance constant initialization :

                        predictedDepth = predictedDepth/(varpred+varest);
                        estimatedDepth = estimatedDepth/(varpred+varest);
                        invDepthMap2.set( varest*estimatedDepth + varpred*predictedDepth, x1.get(1,1),x1.get(2,1) );

                    }
                }

            }
        }

	cout << " L'execution depth-map a prise : " << (float)(clock()-timerdepth)/CLOCKS_PER_SEC << " secondes." << endl;
        /*Orientation Sensor test*/
        /*
        Mat<float> omega = transpose(invK)*invK;
        Mat<float> xo1(0.0,3,1);
        xo1.set(h/(2*factor), 1,1);
        xo1.set(w/(2*factor)-200, 2,1);
        xo1.set( 1.0, 3,1);
        Mat<float> xo2(x1);
        xo2.set(h/(2*factor), 1,1);
        xo2.set(w/(2*factor)+200, 2,1);
        float ctheta = ((float)(transpose(xo1)*(omega*xo2)).get(1,1))/(sqrt( (transpose(xo2)*(omega*xo2)).get(1,1)) * sqrt((transpose(xo1)*(omega*xo1)).get(1,1) ) );
        float theta = acos(ctheta);
        cout << "THETA = " << theta << endl;
        cout << " cos THETA = " << ctheta << endl;
        omega.afficher();
        cv::circle(debugIm,cv::Point(xo1.get(2,1),xo1.get(1,1)), 5, cv::Scalar(0,0,255) );
        cv::circle(debugIm,cv::Point(xo2.get(2,1),xo2.get(1,1)), 5, cv::Scalar(0,0,255) );
        afficher(&debugIm,NULL,NULL,false,1.0);        
        */

	if(debug)
	{
        	cv::imshow("DEBUG",debugIm);
	        debugPose( &f1, &f2, invDepthMap2/*depthmap assigned to f1..*/, rot, t, K, invK, coeff);
	}




        if(cv::waitKey(30)>=0)
            continuer = false;


        ostringstream tfps, distance,compteur,var, gradient;
        fps = (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC));
        tfps << fps << " FPS.";
        distance << "distance = " << dist;
        var << "variance = " << variance;
        compteur << "nombre de pixels : " << count;
        gradient << "Valeur du treshold gradient : " << gradIntTreshold;
        cout << "l'execution a prise : " << (float)((float)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
        cv::putText(frame, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (float)1.25, cv::Scalar(0,0,255));
        cv::putText(frame, distance.str(), cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
        cv::putText(frame, var.str(), cv::Point(10,90), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
        cv::putText(frame, compteur.str(), cv::Point(10,120), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
        cv::putText(frame, gradient.str(), cv::Point(10,150), cv::FONT_HERSHEY_SIMPLEX, (float)0.75, cv::Scalar(0,0,255));
        //cout << (float)(1.0/((float)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("Entry", frame);


	if(debug)
	{
		float min = ((float)-1.0*max(((float)-1.0)*invDepthMap1));
		invDepthMap1 = ((float)-1.0*min)*Mat<float>((float)1.0,invDepthMap1.getLine(),invDepthMap1.getColumn()) + invDepthMap1;
		float maxinv = max(invDepthMap1);
		invDepthMap1 = (float)((float)255.0/maxinv)*invDepthMap1;

		var_invDepthMap2 = var_invDepthMap1;
		float varmin = ((float)-1.0*max(((float)-1.0)*var_invDepthMap2));
		var_invDepthMap2 = ((float)-1.0*varmin)*Mat<float>((float)1.0,var_invDepthMap2.getLine(),var_invDepthMap2.getColumn()) + var_invDepthMap2;
		float varmaxinv = max(var_invDepthMap2);
		var_invDepthMap2 = (float)((float)255.0/varmaxinv)*var_invDepthMap2;

		//invDepthMap2 = 0.9*Mat<float>(1.0,invDepthMap2.getLine(),invDepthMap2.getColumn()) + invDepthMap2;
		//invDepthMap2 = (float)(125.0)*invDepthMap2;

		afficherMat(string("f2,varinvD,invD"), &f2,&var_invDepthMap2,&invDepthMap2,true, coeff);
	}

        //invDepthMap2 = (1.0/125)*invDepthMap2;
        //invDepthMap2 = -0.9*Mat<float>(1.0,invDepthMap2.getLine(),invDepthMap2.getColumn()) + invDepthMap2;
        //Depth Map update :
        invDepthMap1 = invDepthMap2;

        //f1 = f2;
        
        //FPS HANDLING :
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
        
        
        //KF Handling :        
        if( withMultipleKF && countFramePerKF >= nbrFramePerKF )
        {		
        	countFramePerKF = 0;
        	cout << "POSE GLOBAL : " << endl;
        	(operatorC(globalPose,deltaPose)).afficher();
        	globalPose = globalPose*deltaPose;
        	globalPose.afficher();
        	cv::Mat newKF = framelist[nbrFrame-1];
        	KFlist.insert(KFlist.end(), newKF);
        	nbrKF++;
        	
        	
        	f1 = f2;
        	invDepthMap1 = initializeInvDM(&f2Grad, gradIntTreshold, initVariance, mean) + propagateDepthMap( &K, &invK, &deltaPose,&invDepthMap1);
        	//invDepthMap1 = propagateDepthMap( &K, &invK, &deltaPose,&invDepthMap1);
        	var_invDepthMap1 = Mat<float>(depthVar_init,h,w);        	
        }
        afficherMat(string("Residual Image "), &ResidualImage, (Mat<float>*)NULL, (Mat<float>*)NULL,true, coeff);        

    }

    cap.release();


    return 0;
}

int main11(int argc,char* argv[])
{
    /*
    QApplication app(argc, argv);
    myWindow instanceWin(30); //Ajout de notre classe myWindow
    instanceWin.show();   //Exécution de notre fenêtre de rendu OpenGL
    return app.exec();
    */
    return 0;
}


void debugPose(Frame<float>* f1, Frame<float>* f2, const Mat<float>& dm1, const Mat<float>& R, const Mat<float>& t, const Mat<float>& K, const Mat<float>& invK, float factor)
{
    int h = f1->getLine();
    int w = f1->getColumn();
    Mat<float> pos(3,1);
    Mat<float> pos1(3,1);
    pos1.set(1.0,3,1);
    Mat<float> im(0.0,10*h,10*w);

    for(int i=1;i<=h;i++)
    {
        pos1.set(i,1,1);

        for(int j=1;j<=w;j++)
        {

            if(dm1.get(i,j) != 0.0)
            {
                pos1.set(j,2,1);

                //pos.afficher();

                pos = invK*pos1;

                //cout << " Renvoye vers : " << endl;
                //pos.afficher();

                pos = (R*pos+t);

                float norme = norme2(pos);
                pos = ((float)1.0/norme)*pos;
                pos = ((float)1.0/dm1.get(i,j))*pos;

                pos = K*pos;

                //homogeneousNormalization(&pos);
                if(pos.get(3,1) != (float)0.0 && pos.get(3,1) != (float)1.0)
                    pos = ((float)1.0/pos.get(3,1))*pos;

                //pos.afficher();
                //cout << "////////////////////////////////" << endl;


                im.set( (f1->get(pos1)).get(1,1), 10*h/2+pos.get(1,1), 10*w/2+pos.get(2,1) );
            }
        }

    }


    Mat<float> imRef2(extract(im,10*h/2+1,10*w/2+1,10*h/2+h,10*w/2+w));
    Mat<float> imDiff(imRef2-(*f2));
    afficherMat( string("debug ref2, f2, imDiff"), &imRef2, f2, &imDiff,true, factor);

}


//#define v_bench
int main12()
{
    int h = 60;
    int w = 90;
    int delta = 1;
    int size = 5;
    Mat<float> im1((float)0.0,h,w);
    Mat<float> im2(im1);
    Mat<float> dm1((float)0.0,h,w);
    Mat<float> var_idm1((float)0.5,h,w);

    //Benchmark creation
    for(int i=h/10;i<=h-h/10;i++)
    {
    	for(int k=size;k--;)	
    		im1.set((float)100.0, i, 4*w/10+k);
    }
    for(int i=h/10;i<=h-h/3;i++)
    {
        for(int k=size;k--;) 
        	im1.set((float)100.0, i, w/10+k);
    }

    for(int j=w/10;j<=w-w/10;j++)
    {
        for(int k=size;k--;)	
        	im1.set((float)100.0, 8*h/10+k, j);
    }


    for(int i=h/10;i<=h-h/10;i++)
    {
        for(int k=size;k--;)	
        	im2.set((float)100.0, i, 4*w/10+delta+k);
    }
    for(int i=h/10;i<=h-h/3;i++)
    {
        for(int k=size;k--;)	
        	im2.set((float)100.0, i, w/10+delta+k);
    }

    for(int j=w/10;j<=w-w/10;j++)
    {
        for(int k=size;k--;)	
        	im2.set((float)100.0, 8*h/10+delta+k, j);
    }


    for(int i=h/10;i<=h-h/10;i++)
    {
        for(int k=size;k--;)	
        	dm1.set((float)1.0/(float)110.0, i, 4*w/10+k);
    }
    for(int i=h/10;i<=h-h/3;i++)
    {
        for(int k=size;k--;)	
        	dm1.set((float)1.0/(float)110.0, i, w/10+k);
    }

    /*
    for(int j=w/10;j<=w-w/10;j++)
    {
        dm1.set(1.0/110.0, 8*h/10, j);
        dm1.set(1.0/110.0, 8*h/10+1, j);
        dm1.set(1.0/110.0, 8*h/10+2, j);
        dm1.set(1.0/110.0, 8*h/10+3, j);
    }
    */
    //------------------

    Frame<float> f1(im1,1);
    Frame<float> f2(im2,1);
    dm1 = initializeInvDM(&f1,(float)20);

    Mat<float> K((float)0.0,3,3);
    K.set((float)500,1,1); //fx
    K.set((float)500,2,2); //fy
    K.set((float)h/2,1,3); //cx
    K.set((float)w/2,2,3); //cy
    K.set((float)1.0,3,3);
    Mat<float> invK(invGJ(K));

#ifdef v_bench
    cout << "Intrinsic Matrix : " << endl;
    K.afficher();
    cout << "Inverse Intrinsic Matrix : " << endl;
    invK.afficher();

    afficherMat<float>(&im1,&im2, &dm1, false, 2.0);
#endif


    clock_t timer = clock();
    int nbrL = 3;

    //OODirectSlamSIM3<float> instanceOO(&f1,&f2,&K,&invK,&dm1);
    LSSE3<float> instanceLS(&f1,&f2,&K,&invK,&dm1, &var_idm1);
    //bool approxHessian = true;
    //ONSCNewton<float> instanceONSC( &instanceOO, it, approxHessian);
    //bool optimstep = true;
    //ONSC<float> instanceONSC( &instanceOO, it,optimstep);
    Mat<float> X( instanceLS.energy( Mat<float>((float)nbrL,1,1) ) );

    cout << " L'execution a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;

    /*if(it <= 30)
    {
        //regardons l'evolution de notre variable :
        Mat<float> stack(instanceLS.getX(it));
        for(int i=it;i--;)
            stack = operatorL(stack, instanceONSC.getX(i));
        stack.afficher();
    }
    */

    //float s = X.get(7,1);
    //Mat<float> R(s*expW( extract(X,1,1,3,1) ) );
    Mat<float> R(expW( extract(X,1,1,3,1) ) );
    //cout << "rotation matrix : " << s << endl;
    R.afficher();
    Mat<float> t(extract(X,4,1,6,1) );
    cout << "translation vector : " << endl;
    //t.set( 5.0, 3,1);
    //t.set( 0.5,2,1);
    //t.set( 0.2,1,1);
    t.afficher();

    Mat<float> pos((float)0.0,3,1);
    Mat<float> im((float)0.0,10*h,10*w);

    for(int i=1;i<=h;i++)
    {
        for(int j=1;j<=w;j++)
        {
            if(dm1.get(i,j) != (float)0.0)
            {
                pos.set((float)i,1,1);
                pos.set((float)j,2,1);
                pos.set((float)1.0,3,1);

                //pos.afficher();

                pos = invK*pos;

                //cout << " Renvoye vers : " << endl;
                //pos.afficher();

                pos = (R*pos+t);

                float norme = norme2(pos);
                pos = ((float)1.0/norme)*pos;
                pos = ((float)1.0/dm1.get(i,j))*pos;

                pos = K*pos;

                //homogeneousNormalization(&pos);
                if(pos.get(3,1) != (float)0.0 && pos.get(3,1) != (float)1.0)
                    pos = ((float)1.0/pos.get(3,1))*pos;

                //pos.afficher();
                //cout << "////////////////////////////////" << endl;


                im.set( im1.get(i,j), 10*h/2+pos.get(1,1), 10*w/2+pos.get(2,1) );
            }
        }

    }

    cout << "/////////////////////////////////////////" << endl;
    cout << "Pose estimee : " << endl;
    expM(X).afficher();
    cout << "/////////////////////////////////////////" << endl;
    afficherMat(&im,&f1,&f2,false,2.0);

    Mat<float> imRef2(extract(im,10*h/2+1,10*w/2+1,10*h/2+h,10*w/2+w)-f2);
    afficherMat(&imRef2, &f1,&f2,false,2.0);

    /*Pyramidal computation test*/
    /*
    Mat<float> kernel(1.0,4,4);
    Mat<float> pIm( pooling(im,kernel,2) );
    afficherMat(&pIm,&im,&imRef2,false,5.0);
    */

    /*Epipolar Line Computation test*/
    cout << "Epipolar Line Computation test..." << endl;
    float numlim = (float)1000.0/numeric_limits<float>::epsilon();
    Mat<float> epi;
    Mat<float> epi1;
    Mat<float> x((float)1.0,3,1);
    x.set((float)h/10,1,1);
    x.set(4*(float)w/10,2,1);
    computeEpipolarLine2(&epi1,&dm1, &x, &R, &t, &K, &invK );
    computeEpipolarLine(&epi, &x, &R, &t, &K, &invK );
    epi.afficher();
    epi1.afficher();


    cv::Scalar color(255,255,255);
    cv::Scalar color1(100,100,100);
    cv::Mat imEpi( Mat2cvp( imRef2, imRef2, imRef2) );
    //cv::line(imEpi,cv::Point(x.get(1,1)-numlim*epi.get(1,1), x.get(2,1)-numlim*epi.get(2,1)), cv::Point(x.get(1,1)+ numlim*epi.get(1,1),x.get(1,1)+numlim*epi.get(2,1)), color);
    cv::line(imEpi,cv::Point(x.get(1,1)-numlim*epi1.get(1,1), x.get(2,1)-numlim*epi1.get(2,1)), cv::Point(x.get(1,1)+ numlim*epi1.get(1,1),x.get(1,1)+numlim*epi1.get(2,1)), color1);
    //cv::line(imEpi,cv::Point(x.get(1,1), x.get(2,1)), cv::Point(x.get(1,1)+10,x.get(1,1)+20), color);
    afficher(&imEpi,NULL,NULL,false,(float)2.0);



    return 0;
}


Mat<float> poseEstimationLS(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI)
{
    clock_t timer = clock();
    int nbrL = 4;


    LSSE3<float> instanceLS(f1,f2,K,invK,dm1, var_invDepthMap1, nbrL);
    Mat<float> X( instanceLS.energy( Mat<float>((float)nbrL,1,1) ) );
    cout << " L'execution de LSSE3 a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;
    X.afficher();
    *RI = instanceLS.getResidual(nbrL);

#ifdef verbose
    float s = X.get(7,1);
    Mat<float> R(s*expW( extract(X,1,1,3,1) ) );
    Mat<float> t(extract(X,4,1,6,1) );


    cout << "rotation matrix : " << s << endl;
    R.afficher();
    cout << "translation vector : " << endl;
    t.afficher();
#endif

    return expM(X);
    //return expMSIM3( operatorC(X,Mat<float>((float)1,1,1) ) );
}

Mat<float> poseEstimationLSGRAD(Frame<float>* f1, Mat<float>* dm1, Frame<float>* f2, Frame<float>* gradI2X, Frame<float>* gradI2Y, Mat<float>* K, Mat<float>* invK, Mat<float>* var_invDepthMap1, Mat<float>* RI, int pyramidDepth)
{
    clock_t timer = clock();
    int nbrL = pyramidDepth;


    LSSE3<float> instanceLS(f1,f2, gradI2X, gradI2Y, K,invK,dm1, var_invDepthMap1, nbrL);
    Mat<float> X( instanceLS.energy( Mat<float>((float)nbrL,1,1) ) );
    cout << " L'execution de LSSE3 a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;
    //X.afficher();
    *RI = absM(instanceLS.getResidual(nbrL));

#ifdef verbose
    float s = X.get(7,1);
    Mat<float> R(s*expW( extract(X,1,1,3,1) ) );
    Mat<float> t(extract(X,4,1,6,1) );


    cout << "rotation matrix : " << s << endl;
    R.afficher();
    cout << "translation vector : " << endl;
    t.afficher();
#endif

    //return expM(X); //if sum of sol
    return X;	// if product of exp sol
    //return expMSIM3( operatorC(X,Mat<float>((float)1,1,1) ) );
}

template<typename T>
Mat<T> propagateDepthMap( Mat<T>* K, Mat<T>* invK, Mat<T>* deltaPose, Mat<T>* invdm)
{
	Mat<T> PI0((T)0,3,4);
	for(int i=3;i--;)	PI0.set((T)1,i+1,i+1);	
	
	Mat<T> rinvdm((T)0,invdm->getLine(),invdm->getColumn());
	Mat<T> x((T)1,3,1);
	Mat<T> rX(4,1),rx(3,1);	
	
	for(int i=rinvdm.getLine();i--;)
	{
		x.set(i+1,1,1);
		
		for(int j=rinvdm.getColumn();j--;)
		{
			if(invdm->get(i+1,j+1) != (T)0)
			{
				x.set(j+1,2,1);
				//Rigid body motion :
				rX = operatorC( ((T)1.0/invdm->get(i+1,j+1))*((*invK)*x), Mat<T>((T)1,1,1) );
				rX = (*deltaPose)*rX;
				
				homogeneousNormalization(&rX);
												
				/*set the depth of the correct point in the new KF :*/
				rx = (*K)*(PI0*rX);
				//set the correct depth to each point of the new KF according the frame variation ://
				T depth = rx.get(3,1);
				homogeneousNormalization(&rx);
				rinvdm.set( depth, rx.get(1,1), rx.get(2,1));						
			}
								
		}
	}
	
	return rinvdm;
}
