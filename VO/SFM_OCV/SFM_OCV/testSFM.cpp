//#define verbose

#include "Mat.h"
#include "SFM.h"

int main0();	//SFM TX TY
int main1();	//FT test


int main(int argc, char* argv[])
{
	int mode = 0;
	
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
	}
	
	return 0;
}

int main0()
{
	int dimx = 640;
	int dimy = 480;
	int F = 10;
	int P = 10;	// 2*F > P, mandatory.
	Mat<double> U( 0., F,P);
	Mat<double> V( 0., F,P);
	
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
	
	double offset = 0;
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
	SFM<double> instance(U,V, 1);		// 0: no GNa ; 1 : GNa
	cout << "L'execution a pris : " << (double)((double)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
	Mat<double> R(instance.getR()), S(instance.getS());	//rotation and shape matrix.
	R.afficher();
	transpose(S).afficher();
	
	return 1;
}


int main1()
{
	/*FRAME */
	cv::Mat F1,F2;//(640,480, CV_16UC3);
	cv::Mat rF1,rF2;//(640,480,CV_16UC3);
	double factor = 1.0/10.0;
	cv::Size size(0,0); // then the output image size is computed thx to factor. round(fx*src.cols), round(fy*src.rows)

	cv::VideoCapture cap(1);
	if(!cap.isOpened())
	{
	cerr << "Erreur : Impossible de démarrer la capture video." << endl;
	cap.open(0);
	if(!cap.isOpened())
	    return -1;
	}

	cv::namedWindow("Output 1");
	cv::namedWindow("Entry 1");
	cv::namedWindow("Output 2");
	cv::namedWindow("Entry 2");
	/*--------------------*/

	/*FRAME to MAT*/
	Mat<double> iR1, iG1, iB1;
	Mat<double> iR2, iG2, iB2;
	bool continuer = true;
	clock_t timer1 = clock();

	while(continuer)
	{
	
		//FRAME 1 :
		cap >> F1;
		cv::imshow("Entry 1", F1);

		timer1 = clock();
		cv::resize(F1, rF1, size, factor, factor);
		cout << "L'execution a pris : " << (double)((double)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
		cv::imshow("Output 1", rF1);

		timer1 = clock();
		//dst.convertTo(dst, CV_8UC3);
		cv2MatAt( rF1, &iR1,&iG1,&iB1);
		cout << "L'execution a pris : " << (double)((double)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;

	/*-----------------------------------------*/
		//FRAME 2 :
		cap >> F2;
		cv::imshow("Entry 2", F2);

		timer1 = clock();
		cv::resize(F2, rF2, size, factor, factor);
		cout << "L'execution a pris : " << (double)((double)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
		cv::imshow("Output 2", rF2);

		timer1 = clock();
		cv2MatAt( rF2, &iR2,&iG2,&iB2);
		cout << "L'execution a pris : " << (double)((double)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;

		if(cv::waitKey(30) >= 0)
		    continuer = false;
	}

	/*--------------------------*/
	
	Mat<double> gF1( ((double)(1.0/3))*(iR1+iG1+iB1));
	Mat<double> gF2( ((double)(1.0/3))*(iR2+iG2+iB2));
	LF<double> ld( gF1);
	ld.search(1);	//great gradient variation.
	
	Mat<double> landmarks( ld.getLandmarks() );
	
	FT<double> instanceFT( gF1, gF2, landmarks);
	

	return 1;
}
