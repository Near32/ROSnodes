#ifndef OPENCV_USE
#define OPENCV_USE
#endif

#include "vowindow.h"

VOWindow::VOWindow(int fps, QWidget* parent) : myWindow(fps,parent)
{
    gradIntTreshold = 200;
    timertotal = clock();
    factor = 1.0/5;
    khi = Mat<double>(numeric_limits<double>::epsilon(),7,1);
    khi.set((double)1,7,1);

    pose = Mat<double>( expMSIM3(khi) );
    deltaPose = pose;
    pose.afficher();

    cap = cv::VideoCapture(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            ready = false;
    }
    else
        ready = true;

    cv::namedWindow("Entry");
    cv::namedWindow("DEBUG");
    //--------------------

    //containers
    for(int i=0 ; i<= 20; i++)  cap >> frame1;
    cap >> frame1;
    for(int i=0 ; i<= 20; i++)  cap >> frame2;

    cv::resize(frame1,frame1,cv::Size(0,0),factor,factor);


    f1 = Frame<double>( cv2Matp<double>( computeGradient(&frame1) ) );
    f2 = f1;
    h = f1.getLine();
    w = f1.getColumn();


    invDepthMap1 = Mat<double>( initializeInvDM(&f1, gradIntTreshold) );
    invDepthMap2 = invDepthMap1;
    depthVar_init = 0.5;
    var_invDepthMap1 = Mat<double>(depthVar_init,h,w);
    var_invDepthMap2 = var_invDepthMap1;

    x1 = Mat<double>(1.0,3,1);
    x2 = x1;
    numlim = 1.0/numeric_limits<double>::epsilon();


    K = Mat<double>(0.0,3,3);
    //K.set(-0.000876,1,1); //fx
    //K.set(-0.000018459,2,2); //fy
    //K.set((double)-0.4445,1,3); //cx
    //K.set((double)0.900336,2,3); //cy
    K.set((double)430,1,1); //fx
    K.set((double)430,2,2); //fy
    K.set((double)w/2,1,3); //cx
    K.set((double)h/2,2,3); //cy
    K.set(1.0,3,3);
    invK = invGJ(K);

}

VOWindow::~VOWindow()
{
    cap.release();
}

void VOWindow::process()
{
    bool continuer = true;
    double dist = 0;
    cv::Scalar colorbleue(255,0,0);
    cv::Scalar colorvert(0,255,0);

    while(ready && continuer)
    {
        cap >> frame;
        cap >> debugIm;
        cv::resize(frame,frame2,cv::Size(0,0),factor,factor);
        f2 = Frame<double>( cv2Matp<double>( computeGradient(&frame2) ) );


        deltaPose = poseEstimation(&f1, &invDepthMap1,&f2, &K, &invK);
        //pose = pose*deltaPose;
        pose = deltaPose;
        rot = extract(pose,1,1,3,3);
        invRot = transpose(rot);
        // rot € SO(3) !!!
        t = extract(pose,1,4,3,4);
        pose.afficher();


        //TODO : depth map propagation...

        for(int i=invDepthMap1.getLine();i--;)
        {
            x1.set(i+1,1,1);

            for(int j=invDepthMap1.getColumn();j--;)
            {

                if(invDepthMap1.get(i+1,j+1) != 0.0 )
                {
                    dist = 1.0/invDepthMap1.get(i+1,j+1);
                    x1.set(j+1,2,1);
                    x2 = invK*x1;
                    homogeneousNormalization(&x2);
                    x2 = (1.0/invDepthMap1.get(i+1,j+1))*x2;
                    x2 = x2 + t;
                    x2 = rot*x2;
                    x2 = K*x2;

                    homogeneousNormalization(&x2);

                    if(f1.get(x1).get(1,1) >= gradIntTreshold)
                    {
                        //i) compute epipolar line
                        //Method 1:
                        /*
                        computeEpipolarLine(&line2, &x2, &rot, &t, &K, &invK );
                        cv::line(debugIm,
                                 cv::Point((1.0/factor)*x2.get(1,1)-100000*line2.get(1,1), (1.0/factor)*x2.get(2,1)-100000*line2.get(2,1)),
                                 cv::Point((1.0/factor)*x2.get(1,1)+ 100000*line2.get(1,1), (1.0/factor)*x2.get(1,1)+100000*line2.get(2,1)),
                                 colorbleue);
                        */

                        //Method 2:

                        computeEpipolarLine2(&line22,&invDepthMap1, &x2, &rot, &t, &K, &invK );
                        cv::line(debugIm,
                                 cv::Point((1.0/factor)*x2.get(1,1)-100*line22.get(1,1), (1.0/factor)*x2.get(2,1)-100*line22.get(2,1)),
                                 cv::Point((1.0/factor)*x2.get(1,1)+ 100*line22.get(1,1), (1.0/factor)*x2.get(1,1)+100*line22.get(2,1)),
                                 colorvert);



                        //ii) exhaustive search of intensity match
                        //iii) depth update with translation along the optical axis (z)
                        double estimatedDepth = 1.0/(exhaustiveSearchEpipolarLine( &f1,
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
                        double predictedDepth = 1.0/(1.0/invDepthMap1.get(i+1,j+1) + t.get(3,1) );
                        invDepthMap1.set( (1.0/2)*( 1.0/(t.get(3,1) + 1.0/invDepthMap1.get(i+1,j+1))
                                                    + invDepthMap2.get(x2.get(1,1), x2.get(2,1)))
                                          , x1.get(1,1)
                                          , x1.get(2,1) );
                        //mean of the means of the estimation and the prediction.
                        //iv) covariance update :
                        double var = var_invDepthMap1.get(i+1,j+1);
                        var_invDepthMap1.set(  (var+depthVar_init)/(var*depthVar_init), x1.get(1,1), x1.get(2,1) );
                        //fusion of the prior normal distribution variance and the estimation variance constant initialization :
                        //TODO : estimate the initialization variance for each estimation step ?
                        predictedDepth = predictedDepth/(1.0/var+1.0/depthVar_init);
                        estimatedDepth = estimatedDepth/(1.0/var+1.0/depthVar_init);
                        invDepthMap2.set( (1.0/depthVar_init)*estimatedDepth + (1.0/var)*predictedDepth, x1.get(1,1),x1.get(2,1) );

                    }
                }

            }
        }


        /*Orientation Sensor test*/
        /*
        Mat<double> omega = transpose(invK)*invK;
        Mat<double> xo1(0.0,3,1);
        xo1.set(h/(2*factor), 1,1);
        xo1.set(w/(2*factor)-200, 2,1);
        xo1.set( 1.0, 3,1);
        Mat<double> xo2(x1);
        xo2.set(h/(2*factor), 1,1);
        xo2.set(w/(2*factor)+200, 2,1);
        double ctheta = ((double)(transpose(xo1)*(omega*xo2)).get(1,1))/(sqrt( (transpose(xo2)*(omega*xo2)).get(1,1)) * sqrt((transpose(xo1)*(omega*xo1)).get(1,1) ) );
        double theta = acos(ctheta);
        cout << "THETA = " << theta << endl;
        cout << " cos THETA = " << ctheta << endl;
        omega.afficher();
        cv::circle(debugIm,cv::Point(xo1.get(2,1),xo1.get(1,1)), 5, cv::Scalar(0,0,255) );
        cv::circle(debugIm,cv::Point(xo2.get(2,1),xo2.get(1,1)), 5, cv::Scalar(0,0,255) );
        afficher(&debugIm,NULL,NULL,false,1.0);
        */

        cv::imshow("DEBUG",debugIm);




        if(cv::waitKey(30)>=0)
            continuer = false;


        ostringstream tfps, distance;
        tfps << (double)(1.0/((double)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS.";
        distance << "distance = " << dist;
        cout << "l'execution a prise : " << (double)((double)(clock()-timertotal)/CLOCKS_PER_SEC) << " secondes." << endl;
        cv::putText(frame, tfps.str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, (double)1.25, cv::Scalar(0,0,255));
        cv::putText(frame, distance.str(), cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, (double)0.75, cv::Scalar(0,0,255));
        //cout << (double)(1.0/((double)(clock()-timertotal)/CLOCKS_PER_SEC)) << " FPS." << endl;
        timertotal=clock();

        cv::imshow("Entry", frame);

        double min = (-1.0*max((-1.0)*invDepthMap1));
        invDepthMap1 = (-1.0*min)*Mat<double>(1.0,invDepthMap1.getLine(),invDepthMap1.getColumn()) + invDepthMap1;
        double maxinv = max(invDepthMap1);
        invDepthMap1 = (double)(255.0/maxinv)*invDepthMap1;

        var_invDepthMap2 = var_invDepthMap1;
        double varmin = (-1.0*max((-1.0)*var_invDepthMap2));
        var_invDepthMap2 = (-1.0*varmin)*Mat<double>(1.0,var_invDepthMap2.getLine(),var_invDepthMap2.getColumn()) + var_invDepthMap2;
        double varmaxinv = max(var_invDepthMap2);
        var_invDepthMap2 = (double)(255.0/varmaxinv)*var_invDepthMap2;

        //invDepthMap2 = 0.9*Mat<double>(1.0,invDepthMap2.getLine(),invDepthMap2.getColumn()) + invDepthMap2;
        //invDepthMap2 = (double)(125.0)*invDepthMap2;

        afficherMat(&f2,&var_invDepthMap2,&invDepthMap2,true,5.0);

        //invDepthMap2 = (1.0/125)*invDepthMap2;
        //invDepthMap2 = -0.9*Mat<double>(1.0,invDepthMap2.getLine(),invDepthMap2.getColumn()) + invDepthMap2;
        //Depth Map update :
        invDepthMap1 = invDepthMap2;

        //f1 = f2;

    }

}
