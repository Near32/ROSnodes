#ifndef VOWINDOW_H
#define VOWINDOW_H

#include "myWindow.h"
#include "../../MVG/Frame.h"
#include "../../ONSC/ONSC.h"
#include "../../RAND/rand.h"


class VOWindow : public myWindow
{
    Q_OBJECT
public :
    VOWindow(int fps, QWidget* parent = 0);

    ~VOWindow();

    void process();

    Mat<double> poseEstimation(Frame<double>* f1, Mat<double>* dm1, Frame<double>* f2, Mat<double>* K, Mat<double>* invK)
    {
        clock_t timer = clock();
        int it = 10;

        OODirectSlamSIM3<double> instanceOO(f1,f2,K,invK,dm1);
        bool optimstep = true;
        ONSC<double> instanceONSC( &instanceOO, it, optimstep);
        //bool approxHessian = true;
        //ONSCNewton<double> instanceONSC( &instanceOO, it, approxHessian);
        cout << " L'execution a prise : " << (double)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;

        Mat<double> X( instanceONSC.getX(it));

    #ifdef verbose
        double s = X.get(7,1);
        Mat<double> R(s*expW( extract(X,1,1,3,1) ) );
        Mat<double> t(extract(X,4,1,6,1) );


        cout << "rotation matrix : " << s << endl;
        R.afficher();
        cout << "translation vector : " << endl;
        t.afficher();
    #endif

        return expMSIM3(X);
    }


    Mat<double> initializeInvDM(Frame<double>* frame, double gradIntTreshold)
    {
        NormalRand gen(0,1,(long)17);
        Mat<double> idm(0.0, frame->getLine(), frame->getColumn());
        Mat<double> x(0.0,2,1);

        for(int i=frame->getLine();i--;)
        {
            x.set(i+1,1,1);

            for(int j=frame->getColumn();j--;)
            {
                x.set(j+1,2,1);

                //inverse depth value is assigned only if the gradient is sufficient.
                if( fabs_(frame->get(x).get(1,1)) >= (double)gradIntTreshold)
                {
                    idm.set( 0.01+(double)gen.dev(), i+1,j+1);
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
        //cv::GaussianBlur(*im,*im,cv::Size(0,0),(double)1.0/sqrt(2),(double)1.0/sqrt(2));


        vector<cv::Mat> grad;
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type()) );
        grad.insert(grad.begin(), cv::Mat::zeros(cv::Size(im->rows,im->cols), im->type()) );

        // Gradient X
        Sobel( *im, grad[0], ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[0], grad[0] );

        // Gradient Y
        Sobel( *im, grad[1], ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
        convertScaleAbs( grad[1], grad[1] );

        return (double)(0.5)*(grad[0]+grad[1]);
        /*
        cv::Mat grad;
        Sobel(*im,grad,ddepth,1,1,3,scale,delta,cv::BORDER_DEFAULT);
        //cv::GaussianBlur(grad,grad,cv::Size(0,0),(double)1.0/sqrt(2),(double)1.0/sqrt(2));
        convertScaleAbs(grad,grad);
        return absMAt(grad);
        */
    }

    void computeEpipolarLine(Mat<double>* l, Mat<double>* x, Mat<double>* R, Mat<double>* t, Mat<double>* K, Mat<double>* invK)
    {
        *l = (crossProduct((*K)*((-1.0)*(*t))) * ((*K)* ((*R)* ((*invK) * (*x)))) );
        homogeneousNormalization(l);
    }

    void computeEpipolarLine2(Mat<double>* l, Mat<double>* invDepthMap1, Mat<double>* x, Mat<double>* R, Mat<double>* t, Mat<double>* K, Mat<double>* invK)
    {
        double lambda1 = 1.0/invDepthMap1->get(x->get(1,1),x->get(2,1));
        double lambda2 = 1.0/invDepthMap1->get(x->get(1,1),x->get(2,1)) + 100;//1.0/numeric_limits<double>::epsilon();
        Mat<double> retroprojx = (*invK)*(*x);
        //here is hC = RCW * hW
        // let us  have h features in the frame of the second camera :
        Mat<double> Xlambda1 = (*R)*retroprojx + (*t);
        Mat<double> Xlambda2 = (*R)*retroprojx + (*t);
        //the direction is not normalized...
        double norme = norme2(Xlambda1);
        Xlambda1 = (1.0/norme)*Xlambda1;
        Xlambda2 = (1.0/norme)*Xlambda2;
        //now we just have to it the correct distance : X features in the second camera frame :
        Xlambda1 = lambda1*Xlambda1;
        Xlambda2 = lambda2*Xlambda2;
        //let us project them again :
        Mat<double> x1_2 = (*K)*Xlambda1;
        Mat<double> x2_2 = (*K)*Xlambda2;
        // and finally we have to extract the pixel coordinate out of that hC :
        homogeneousNormalization(&x1_2);
        homogeneousNormalization(&x2_2);
        // here is our line joining the two points within the pixel frame :
        *l = x1_2-x2_2;
        l->set(1.0,3,1);
    }


    double exhaustiveSearchEpipolarLine( Frame<double>* f1, Frame<double>* f2, Mat<double>* invDepthMap1, Mat<double>* x1, Mat<double>* x2, Mat<double>* R, Mat<double>* t, double var_invDepthMap, Mat<double>* K, Mat<double>* invK, double factor)
    {
        double lambda;
        Mat<double> Xlambda(3,1);
        Mat<double> x_2(3,1);
        int nbrHyp = 5;
        double varStep = (4.0/nbrHyp)*var_invDepthMap;
        Mat<double> photoDist(1,1);
        Mat<double> minphotoDist( absM(f1->get(*x1)-f2->get(*x2)) );
        double minlambda = 1.0/invDepthMap1->get(x1->get(1,1),x1->get(2,1));
        cv::Scalar colorLine1(255,0,0);
        cv::Scalar colorLine2(0,0,255);
        cv::Scalar colorPoint(0,255,0);
        cv::Mat debugIm( Mat2cvp(*f2,*f2,*f2));

        for(int i=nbrHyp+1;i--;)
        {
            lambda =1.0/(invDepthMap1->get(x1->get(1,1),x1->get(2,1)) - 2.0*var_invDepthMap + i*varStep );
            //here is hW = RWC * hC + rWC
            // let us  have h features in the frame of the second camera :
            Xlambda = (*R)*((*invK)*(*x1)) + (*t);

            //the direction is not normalized...
            double norme = norme2(Xlambda);
            Xlambda = (1.0/norme)*Xlambda;
            //now we just have to it the correct distance : X features in the second camera frame :
            Xlambda = lambda*Xlambda;

            //let us project them again :
            x_2 = (*K)*Xlambda;
            // and finally we have to extract the pixel coordinate out of that hC :
            homogeneousNormalization(&x_2);


            photoDist = absM( f1->get(*x1)-f2->get(x_2) );

            if(photoDist < minphotoDist)
            {
                minlambda = lambda;
                minphotoDist = photoDist;
            }
        }


        Xlambda = (*R)*((*invK)*(*x1))+(*t) ;
        Xlambda = (double)(1.0/norme2(Xlambda))*Xlambda;
        Xlambda = minlambda*Xlambda;
        x_2 = (*K)*Xlambda;
        homogeneousNormalization(&x_2);
        Mat<double> line;
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

        //afficher(&debugIm,NULL,NULL,true,1.0/ factor);


        return minlambda;
    }

    double exhaustiveSearchEpipolarLine2( Frame<double>* f1, Frame<double>* f2, Mat<double>* invDepthMap1, Mat<double>* x1, Mat<double>* x2, Mat<double>* R, Mat<double>* t, double var_invDepthMap, Mat<double>* K, Mat<double>* invK, double factor)
    {

        double lambda;
        Mat<double> Xlambda;
        Mat<double> x_2;
        int nbrHyp = 5;
        Mat<double> photoDist;
        Mat<double> minphotoDist( absM(f1->get(*x1)-f2->get(*x2)) );
        double minlambda = 1.0/invDepthMap1->get(x1->get(1,1),x1->get(2,1));
        cv::Scalar colorLine1(255,0,0);
        cv::Scalar colorLine2(0,0,255);
        cv::Scalar colorPoint(0,255,0);
        cv::Mat debugIm( Mat2cvp(*f2,*f2,*f2));

        Mat<double> epiline(3,1);
        computeEpipolarLine(&epiline,x1,R,t,K,invK);
        double h = f1->getLine();
        double w = f1->getColumn();
        double varStepPixel = sqrt(h*h+w*w)/(2*nbrHyp);


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
        Mat<double> line;
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




protected :
    bool ready;
    double gradIntTreshold;
    clock_t timertotal;
    double factor;
    Mat<double> khi;

    Mat<double> pose;
    Mat<double> deltaPose;
    cv::Mat frame,frame1,frame2;
    cv::Mat debugIm;


    cv::VideoCapture cap;

    Frame<double> f1;
    Frame<double> f2;
    int h;
    int w;


    Mat<double> invDepthMap1;
    Mat<double> invDepthMap2;
    double depthVar_init;
    Mat<double> var_invDepthMap1;
    Mat<double> var_invDepthMap2;

    Mat<double> x1;
    Mat<double> x2;
    Mat<double> line2;
    Mat<double> line22;
    double numlim;


    Mat<double> K;
    Mat<double> invK;
    Mat<double> rot;
    Mat<double> invRot;
    Mat<double> t;


};

#endif // VOWINDOW_H
