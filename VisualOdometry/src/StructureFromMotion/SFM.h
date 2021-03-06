#include "../MATv2/Mat.h"
#include "../GaussNewtonAlgorithm/GNA.h"
#include "../LandmarkFinder/LF.h"

#define iteration 10

using namespace std;

template<typename T>
class SFM
{
	private :
	int F;		//nbr of frame.
	int P;		//nbr of points.
	
	Mat<T> U;	//FxP : u(f,p) = value on the x axis of the image of the p-th point in the f-th frame. 
	Mat<T> V;	//FxP : v(f,p) = value on the y axis of the image of the p-th point in the f-th frame. 
	Mat<T> W;	//2FxP :measurement matrix col(U,V)
		
	Mat<T> reg;	//Fx2 : af = mean of u(f,i) Vi ; bf = mean of v(f,i) Vi.
	Mat<T> W_;	//2FxP : registered measurement matrix : col ( (u(f,p) - af), (v(f,p) - bf) )
	
	Mat<T> R;	//2Fx3 ?
	Mat<T> S;	//3xP  ?
	Mat<T> Q;	//3x3 € GL3(R)


	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/

	
	public:
	
	
	
	SFM(Mat<T> U_, Mat<T> V_, int mode = 1) : F(U_.getLine()), P(U_.getColumn()), U(U_), V(V_)
	{
		W = operatorC(U,V);
		
		if(U_.getLine() == V_.getLine() && U_.getColumn() == V_.getColumn() )
		{
			reg = operatorC( Mat<T>(mean( extract(U, 1, 1, 1,P) ), 1,1) , Mat<T>( mean( extract(V, 1,1, 1,P)), 1,1) );
			
			for(int i=2;i<=F;i++)
				reg = operatorL(reg, operatorC( Mat<T>( mean( extract(U, i, 1, i,P) ), 1,1), Mat<T>( mean( extract(V, i,1, i,P)), 1,1) ) );
			
				
			W_ = W;			
			for(int fr=1;fr<=2*F;fr++)
			{
				for(int pt=1;pt<=P;pt++)
				{
					W_.set( W_.get(fr,pt) - reg.get( fr, (fr<=F? 1 : 2) ), fr, pt);
				}
			}
			
			cout << "Factorization : begin." << endl;
			
			SVD<T> instance(W_);
			R = instance.getU();
			S = instance.getV();
			Mat<T> D = instance.getS();
			
			cout << "SVD : Done." << endl;
			
			/*on reordonne :*/
			bool continuer = true;
			bool ech = false;
			int Dl = D.getLine();
			
			if(Dl!=P)
			{				
				Dl = (Dl <= D.getColumn() ? Dl : D.getColumn());
			}
			
			while(continuer)
            {
				for(int i=1;i<=Dl-1;i++)
				{	
					if(D.get(i,i) <= D.get(i+1,i+1))
					{
						/*alors on fait l'echange :*/
						D.swap(i,i, i+1,i+1);
						R.swapC(i,i+1);
						S.swapC(i,i+1);	// test ?						
						ech = true;
					}
				}
				
				if(ech == false)
				{
					cout << "exiting" << endl;
					continuer = false;
				}
					
				
				ech = false;
			}
			/*---------------------------*/
			
			D.afficher();
			/*on prend ce qui nous interesse*/
			D = extract(D, 1,1, 3,3);
			D.set((T)0, 1,2);
			D.set((T)0, 1,3);
			D.set((T)0, 2,1);
			D.set((T)0, 2,3);
			D.set((T)0, 3,1);
			D.set((T)0, 3,2);
			D.afficher();
			
			R = extract(R, 1, 1, 2*F, 3)*sqrt(D);

			transpose(S).afficher();			
			S = sqrt(D) * extract(transpose(S),1,1, 3, P);		
			
			/*------------------------------*/
			/*Determination de Q*/
			cout << "/////////////////////////////\nComputation of Q : ....";			
			computeQ(mode);
			cout << " DONE." << endl;			
			/*------------------------------*/
			R.afficher();
			R = R*Q;
			R.afficher();
			transpose(S).afficher();
			transpose(invGJ(Q)).afficher();
			S = invGJ(Q)*S;
			
			/*--------------------------------*/
			
			cout << "Factorisation : DONE." << endl;
		}
		else
			cerr << "ERREUR : les dimensions des matrices U et V ne coincident pas." << endl;
	}
	
	
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	
	
	~SFM()	{}
	
	
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	
	
	Mat<T> getR()	const
	{
		return R;
	}
	
	Mat<T> getS()	const
	{
		return S;
	}
	
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	
	void computeQ(int mode)
	{
		Q = Mat<T>((T)0, 3,3);
		for(int i=1;i<=3;i++)	Q.set( (T)1, i,i);
		Q.afficher();
		
		
		if(mode)
		{
			vector<Mat<T> > params;
			
		
			for(int fr=1;fr<=F;fr++)
			{
				params.push_back( operatorL( transpose(Line(R, fr)), transpose(Line(R, F+fr)) ) );
			}
		
			Mat<T> rV((T)1, 3,1);
			rV.set( (T)0, 3,1);
			Mat<T> temp(rV);
			
			for(int fr=1;fr<=F-1;fr++)	rV = operatorC(rV, temp);
			
			rV.afficher();
						
			GNA<T> instance( initGLrand, params, rV, SFM_F, SFM_J, iteration);
			Q = instance.getBeta();
		}
		
		
	
	}
	
	
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	/*---------------------------------------------------------------------------------------*/
	

};


template<typename T>
class FT 
{
	private :
    LF<T>* seeker;
	
	Mat<T> F1;
	Mat<T> F2;
	int n;
	int m;
	
	T lambda;
	
	int nW;
	int mW;
	vector<Mat<T> > W;
	vector<Mat<T> > paramW;
	vector<Mat<T> > GW;
	//vector<Mat<T> > e;	//stores in d...
	vector<Mat<T> > d;	//displacement vector for the window W.
	
	
	Mat<T> points;
	Mat<T> outputPoints;
	
	public :
	
    FT( Mat<T> F1_, Mat<T> F2_, Mat<T> trackedPoints, int nW_ =5, int mW_ =5) : F1(F1_), F2(F2_), points(trackedPoints)
	{
		nW = nW_;
		mW = mW_;
		n = F2.getLine();
		m = F2.getColumn();

        cout << "Taille des images : " << endl;
        cout << n << endl;
        cout << m << endl;
		

        process();
	
	}
	
	~FT()
	{
	
	}
	
	
	/*-----------------------------------------------------*/
	/*-----------------------------------------------------*/
	/*-----------------------------------------------------*/
	/*-----------------------------------------------------*/
	
    void process()
    {
        //clock_t timer1 = clock();
        lambdaSelection();
        //cout << "L'execution a pris : " << (T)((T)(clock()-timer1)/CLOCKS_PER_SEC) << " secondes." << endl;
        cout << "Window Selection :" << endl;
        WindowSelection();
        solve();        
    }
	
	Mat<T> computeGrad4W( Mat<T> Wdw)
	{
		Mat<T> kX((T)(1/2), 3,3);
		for(int i=1;i<=3;i++)	kX.set((T)0, i,2);
        for(int i=1;i<=3;i++)	kX.set((T)(-1.0/2), i,3);
		Mat<T> kY((T)(1/2), 3,3);
		for(int i=1;i<=3;i++)	kY.set((T)0, 2,i);
        for(int i=1;i<=3;i++)	kY.set((T)(-1.0/2), 3,i);
		
		return operatorL( correlation(Wdw, kX), correlation(Wdw,kY) );	
	}
	
	Mat<T> computeG4W( Mat<T> Wdw)
	{
		Mat<T> Grad( computeGrad4W(Wdw) );
		Mat<T> G((T)0, 2,2);
		
		int nWdw = Wdw.getLine();
		int mWdw = Wdw.getColumn();
		for(int i=1;i<=nWdw; i++)
		{
			for(int j=1;j<=mWdw;j++)
			{
				Mat<T> g2(2,1);
				g2.set( Grad.get(i,j), 1,1);
				g2.set( Grad.get(i,j+mWdw), 2,1);
				G = G + g2*transpose(g2);
			}
		}
		
		return G;
	}
	
	
	void lambdaSelection()
	{
        cout << "FT : lambda selection : ..." << endl;                
        Mat<T> gradF2( computeGrad4W(F2));
        gradF2 = absM( extract(gradF2, 1,1, n,m)) + absM( extract(gradF2, 1, m+1, n,2*m));
        //gradF2.afficher();

        cout << "FT : lambda selection : convolution..." << endl;
		Mat<T> kernel((T)1, n/10, m/10);
        Mat<T> minimal( convolution(gradF2, kernel));
        Mat<T> id( idmin( extract(minimal, 2,2, n-1,m-1) ) );
		
        cout << "FT : lambda selection : convolution : DONE." << endl;
		//constant brightness window setting :
		minimal = extract( F2, (n/10)*id.get(1,1)-(n/10)/2, (m/10)*id.get(2,1)-(m/10)/2, (n/10)*id.get(1,1)+(n/10)/2, (m/10)*id.get(2,1)+(m/10)/2 );
		
        cout << "FT : lambda selection : compute G..." << endl;
		Mat<T> G( computeG4W( minimal));
        SVD<T> instanceSVD(G, 10);
        lambda = (instanceSVD.getS()).get(1,1);
		
		
		cout << "Lambda = " << lambda << endl;	
	}
	
	void WindowSelection()
	{
        Mat<T> defaultD((T)10,2,1);

		for(int i=1;i<=n-nW;i+=nW)
		{
			for(int j=1;j<=m-mW;j+=mW)
			{                
				Mat<T> window(extract( F2, i,j, i+nW,j+mW) );                
				Mat<T> G(computeG4W(window));
				
                SVD<T> instanceSVD( G , 10);
                T lambdaW = (instanceSVD.getS()).get(2,2);                
				                
                if( 1 || lambdaW>lambda)
				{                    
                    /*
                    cout << "Window : " << i << " - " << j << endl;
                    cout << lambda << " et lambdaW = " << lambdaW << endl;
					cout << "Window : OK" << endl;
                    */
					W.insert(W.end(), window);                    

					Mat<T> coord(2,1);
					coord.set( (T)i, 1,1);
					coord.set( (T)j, 2,1);
					paramW.insert(paramW.end(), coord);
					
					GW.insert(GW.end(), G);
					
					//compute e :
					Mat<T> Grad( computeGrad4W( window) );
					Mat<T> h( extract(F1, i,j, i+nW, j+nW) - window);
					Mat<T> tempe( (T)0, 2,1);					                    

					for(int iw=1;iw<=nW;iw++)
					{
						for(int jw=1;jw<=mW;jw++)
						{
							Mat<T> g2(2,1);
							g2.set( Grad.get(iw, jw), 1, 1);
							g2.set( Grad.get(iw, jw+mW), 2,1);
							tempe = tempe + h.get(iw,jw)*g2;
						}
					}					                    

                    if( lambdaW > lambda)
                    {                        
                        d.insert(d.end(), tempe);                        
                        defaultD = tempe;                        
                        // in order to interpolate the deplacement...
                    }
                    else
                    {
                        d.insert(d.end(), defaultD);
                    }
					/*----------------------*/
				}
				else
					cout << "Window : NG" << endl;
			}
		}
	
	}
	
	void solve()
	{        
		int nbrW = W.size();
		
		for(int nwd=0;nwd<=nbrW-1;nwd++)
		{

            //cout << "Solve : " << nwd << " / " << nbrW-1 << endl;
			gaussj( &(W[nwd]), &(d[nwd]) );
		}
	}
	
	
	
    void PointDeplacement( Mat<T>* points_ = NULL)
	{

        outputPoints = (points_ == NULL ? points : *points_);

        for(int p=1;p<=points.getColumn();p++)
        {
            cout << "Points : " << p << " / " << points.getColumn() << endl;
            for(int i=0;i<= n/nW;i++)
            {
                for(int j=0;j<=m/mW;j++)
                {

                    if(points.get(1,p) <= (i+1)*nW)
                    {
                        if(points.get(1,p) >= i*nW)
                        {
                            if(points.get(2,p) <= (j+1)*mW)
                            {
                                if(points.get(2,p) >= j*mW)
                                {
                                    //Then the points belonged to that window
                                    //update :
                                    outputPoints.set( points.get(1,p)+d[((i*n)/nW + (j*m)/mW < d.size() ? (i*n)/nW + (j*m)/mW : 0)].get(1,1), 1,p);
                                    outputPoints.set( points.get(2,p)+d[((i*n)/nW + (j*m)/mW < d.size() ? (i*n)/nW + (j*m)/mW : 0)].get(2,1), 2,p);
                                    //break :
                                    i = n/nW+1;
                                    j = m/mW+1;
                                }
                            }
                        }
                    }
                }
            }
        }
		
	}
	
		
	/*------------------------------------------------------------------------*/
	/*---------------------------ACCESSEURS-----------------------------------*/
	
	vector<Mat<T> > getWindow()
	{
		return W;
	}
	
	vector<Mat<T> > getD()
	{
		return d;
	}
	
	vector<Mat<T> > getGW()
	{
		return GW;
	}

    void setF1( const Mat<T> F1_)
    {
        F1 = F1_;
    }

    void setF2( const Mat<T> F2_)
    {
        F2 = F2_;
    }

    int getnw() const
    {
        return nW;
    }

    int getmw() const
    {
        return mW;
    }

    Mat<T> getOutputPoints() const
    {
        return outputPoints;
    }



};

