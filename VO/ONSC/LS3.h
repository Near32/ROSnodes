#ifndef LS_H
#define LS_H

#include "OO.h"
//#define verbose_res_debug		    
//#define verbose_mini
//#define verbose_mini_res
//#define verbose_debug
//#define verbose_debug_JR
//#define verbose_debug_A
//#define verbose_debug_y
//#define verbose_debug_W
//#define verbose_debug_mini
//#define verbose_debug_r
//#define verbose_debug_rigidMotion

template<typename T>
class LSSE3 : OODirectSlamSE3<T>
{

protected :

    Mat<T>* res;
    bool initY;
    Mat<T>* y;
    bool initA;
    Mat<T>* A;
    bool initW;
    Mat<T>* W;
    
    Mat<T> x;
    Mat<T>* solutions;
    Mat<T> sol;
    
    int level;	//level of the pyramid scheme
    int itsperlevel;
    bool maxItsLowLevel;
    int nbrL;
    Mat<T> khi_0;

    T h;

public :

    LSSE3( Frame<T>* I1, Frame<T>* I2, Mat<T>* K, Mat<T>* invK, Mat<T>* depthMapI1, Mat<T>* var_depthMapI1, int nbrlevel = 4) :     OODirectSlamSE3<T>( I1, I2, K, invK, depthMapI1, var_depthMapI1, nbrlevel)
    {
    	//pyramid computed
    	level = 0;	// fourth level of the pyramid...
    			// the pyramid is inversed and goes from 0 to DirectSLAMSSE3::pyramidDepth -1 = 3 .
        itsperlevel = 1;
        maxItsLowLevel = true;

        //khi_0 = Mat<T>((T)0,6, 1);
        khi_0 = pow(numeric_limits<T>::epsilon(), (T)1)*Mat<T>(6,1,(char)1);
        sol = khi_0;
    	// result in a near-identity rigid body motion...
    
    	//solutions.insert(solutions.begin(), khi_0);
    	solutions = new Mat<T>[itsperlevel*nbrlevel+1];
	for(int i=itsperlevel*nbrlevel+1;i--;)	solutions[i] = sol;

	/*
	//TODO : initializing everything here...
	res[level] = Mat<T>((T)0,this->pyramidD1[level].getLine(), this->pyramidD1[level].getColumn());
     	Mat<T> xi(2,1);
     	
     	//initializations :
     	A[level*itsperlevel+(it-1)] = Mat<T>((T)0, this->pyramidD1[level].getLine()*this->pyramidD1[level].getColumn(), 6);
     	Mat<T> tempA(1,6);
     	//dim nx6 where n = #Sigma
     	y[level*itsperlevel+(it-1)] = Mat<T>((T)0, this->pyramidD1[level].getLine()*this->pyramidD1[level].getColumn(),1);
     	*/    	
     	
     	
        /*
        for(int i=itsperlevel*nbrlevel;i--;)	res.insert( res.begin(), Mat<T>((T)0, this->pyramidD1[i/itsperlevel].getLine(), this->pyramidD1[i/itsperlevel].getColumn()) );*/
        res = new Mat<T>[itsperlevel*nbrlevel];        
        
        /*
    	for(int i=itsperlevel*nbrlevel;i--;)	A.insert( A.begin(), Mat<T>(1,1) );*/
    	A = new Mat<T>[itsperlevel*nbrlevel];
    	
    	/*
    	for(int i=itsperlevel*nbrlevel;i--;)	y.insert( y.begin(), Mat<T>(1,1) );*/
    	y = new Mat<T>[itsperlevel*nbrlevel];
    	
    	/*
    	for(int i=itsperlevel*nbrlevel;i--;)	W.insert( W.begin(), Mat<T>(1,1) );*/
    	W = new Mat<T>[itsperlevel*nbrlevel];
    	
        initY = false;
        initA = false;
        initW = false;


        //derivation handling :
        h = pow(numeric_limits<T>::epsilon(), (T)0.06125);
    	
    }
    
    ~LSSE3()
    {
    	delete[] res;
    	delete[] y;
    	delete[] A;
    	delete[] W;
    	delete[] solutions;

    }
    
    Mat<T> init()
    {
        //return (T)(1.0/100)*Mat<T>(7,1,(char)1);//
        //return Mat<T>((T)numeric_limits<T>::epsilon(),7,1);
        Mat<T> r(6,1,(char)1);
        r = (T)numeric_limits<T>::epsilon()*r;      
        
        return r;
    }
    
    Mat<T> energy(Mat<T> X)
    {
    	nbrL = X.get(1,1)-1;	//number of level for which the loop will run.
    	
        for(;level<=nbrL;level++)
    	{
    	
    	    for(int i=1;i<=itsperlevel;i++)
    	    {
		    int nbrLA = computePyramidResidualA(i);
		    	
		    	//Least-Square Resolution for this level :

		    while(false && A[level*itsperlevel+(i-1)] == Mat<T>((T)0, A[level*itsperlevel+(i-1)].getLine(), A[level*itsperlevel+(i-1)].getColumn() ) )
		    {
		        cout << "RECOMPUTATION OF A... h = " << h << endl;
		        h = sqrt(h);
		        initA = false;
		        initY = false;
		        initW = false;
		        nbrLA = computePyramidResidualA(i);

		    }

		    
		    if( A[level*itsperlevel+(i-1)]== Mat<T>((T)0,A[level*itsperlevel+(i-1)].getLine(),A[level*itsperlevel+(i-1)].getColumn() )  && level < 3)
		    {
		        //take care of the case where the pyramidD1[level] is empty... :
		        cout << "EMPTY PYRAMIDD1... level = " << level << " nbrLA = " << nbrLA << endl;
		        
		        //afficherMat( string("PYRAMID D1"), (Mat<T>*)(&(this->pyramidD1[level])), (Mat<T>*)NULL, (Mat<T>*)NULL, false, (float)10); 
		        
		        solutions[level*itsperlevel+(i-1)] = khi_0;
		        i++;
		        if(i>itsperlevel)
		        {
		        	i=1;
		        	level++;
		        }
		        initA = false;
		        initY = false;
		        initW = false;
		        nbrLA = computePyramidResidualA(i);
		    }
		    
		    /*
		    while( isnanM(A[level*itsperlevel+(i-1)]) )
		    {
		        //take care of the case where the pyramidD1[level] is empty... :
		        cout << "NAN FOUND in A..." << endl;
		        h = pow(numeric_limits<T>::epsilon(), (T)(1.0+h));
		        solutions[level*itsperlevel+(i-1)] = khi_0;
		        initA = false;
		        initY = false;
		        initW = false;
		        nbrLA = computePyramidResidualA(i);
		    }
		    */

		    A[level*itsperlevel+(i-1)] = extract(A[level*itsperlevel+(i-1)], 1,1, nbrLA, 6);
		    y[level*itsperlevel+(i-1)] =  extract(y[level*itsperlevel+(i-1)], 1,1, nbrLA, 1);
		    Mat<T> tAA( transpose(A[level*itsperlevel+(i-1)]) * W[level*itsperlevel+(i-1)] * A[level*itsperlevel+(i-1)]);
		    //Mat<T> tAA( transpose(tempA) * W[level*itsperlevel+(i-1)] * tempA);
		    Mat<T> itAA( invGJ(tAA) );

		    x = itAA * ( transpose(A[level*itsperlevel+(i-1)]) * W[level*itsperlevel+(i-1)] * y[level*itsperlevel+(i-1)]);
		    //x = itAA * ( transpose(tempA) * W[level*itsperlevel+(i-1)] * tempy);

		    //take care of the case where x= "false"0...
		    if(x == Mat<T>((T)0,x.getLine(),x.getColumn()) )
		    {
		        x = pow(numeric_limits<T>::epsilon(), (T)0.5)*Mat<T>((T)1, 6, 1);
			/*x = x + sol;
			if(level != 0 || i!=1)
				x = ((T)1.0/(level*itsperlevel+(i-1)))*sol;*/
		    }


		    //NAN Handling :
		    if(isnanM(x))
		    {
		        x = pow(numeric_limits<T>::epsilon(), (T)0.5)*Mat<T>((T)1, 6,1);
		        cout << "NAN FOUND ON LEVEL : " << level << " : " << i << endl;
		    }
		    sol = sol + x;
	//            sol.afficher();

	#ifdef verbose_mini_res
		    cout << " tAA : " << endl;
		    tAA.afficher();
		    T det = computeDeterminant(tAA);
		    cout << " Determinant = " << det << endl;
		    cout << " inverse of tAA : " << endl;
		    itAA.afficher();
		    cout << " PRODUCT : " << endl;
		    (tAA*itAA).afficher();            
	#endif

	#ifdef verbose_debug
		    cout << "Matrice A : level = " << level << endl;
		    A[level*itsperlevel+(i-1)].afficher();
		    cout << "Matrice y : level = " << level << endl;
		    y[level*itsperlevel+(i-1)].afficher();
		    //cout << "Matrice W : level = " << level << endl;
		    //W[level*itsperlevel+(i-1)].afficher();
		    cout << "SOLUTION : level = " << level << endl;
		    expM(x).afficher();
		    cout << "SOLUTION sol : level = " << level << endl;
		    expM(sol).afficher();
	#endif


		    //solutions[level*itsperlevel+(i-1)] = x;
	#ifdef verbose_res_debug		    
		    cout << "Solution iteration précédente : level = " << level << " : " << i-1 << endl;
		    expM(solutions[level*itsperlevel+(i-1)]).afficher();
		    cout << "Solution à cette iteration : level = " << level << " : " << i << endl;
		    expM(x).afficher();
	#endif		    
		    solutions[level*itsperlevel+i] = x;
		    
	#ifdef verbose_debug
		    for(int ite=0;ite<nbrL*itsperlevel;ite++)
		    {
		    	cout << "solutions " << ite << endl;
		    	solutions[ite].afficher();		   	
		    }
	#endif
		    // problem solved for this level.

		    initY = false;
		    initA = false;
		    initW = false;
		    
		    if(maxItsLowLevel && (level != 1 ) && itsperlevel >1)
		    {
		    	solutions[level*itsperlevel+i] = solutions[level*itsperlevel+(i-1)];
		    	A[level*itsperlevel+i] = A[level*itsperlevel+(i-1)];
		    	W[level*itsperlevel+i] = W[level*itsperlevel+(i-1)];
		    	y[level*itsperlevel+i] = y[level*itsperlevel+(i-1)];
		    	res[level*itsperlevel+i] = res[level*itsperlevel+(i-1)];
		    	i++;
		    }
		}
        }
	
	//final solution.
	return x;
    }
    
    inline int computePyramidResidualA(int it)
    {
    	//TODO : Watch out the xi positioning with respect to the camera frame and pixel frame ?
    	
     	res[level] = Mat<T>((T)0,this->pyramidD1[level].getLine(), this->pyramidD1[level].getColumn());
     	Mat<T> xi(2,1);
     	
     	/*initializations :*/
     	A[level*itsperlevel+(it-1)] = Mat<T>((T)0, this->pyramidD1[level].getLine()*this->pyramidD1[level].getColumn(), 6);
     	Mat<T> tempA(1,6);
     	//dim nx6 where n = #Sigma
     	y[level*itsperlevel+(it-1)] = Mat<T>((T)0, this->pyramidD1[level].getLine()*this->pyramidD1[level].getColumn(),1);
     	//W[level*itsperlevel+(it-1)] = Mat<T>((T)0, A[level*itsperlevel+(it-1)].getLine(), A[level*itsperlevel+(it-1)].getLine());
     	//Mat<T> tempW((T)0, 1, A[level*itsperlevel+(it-1)].getLine());
     	int count = 0;
     	
        for(int i= this->pyramidD1[level].getLine();i--;)
     	{
            xi.set((T)i+1, 1,1);
     		
            for(int j= this->pyramidD1[level].getColumn();j--;)
     	    {
                xi.set((T)j+1, 2,1);
		
		/*
		//TODO : HANDLE NAN OUTSIDE...
                //NAN handling :
                if( isnan( this->pyramidD1[level].get(i+1,j+1) ) )
                {
                    this->pyramidD1[level].set( (T)1.0/pow( numeric_limits<T>::epsilon(), (T)0.5), i+1, j+1) ;
                }
	     	*/
	     		
                if( (this->pyramidD1[level]).get( i+1, j+1) != (T)0)
     		{
     		    count++;
                    //cout << "inverse distance : " << (this->pyramidD1[level]).get(i+1,j+1) << endl;
                
                
	     	    Jr(&tempA, solutions[level*itsperlevel+(it-1)], xi);
	     	    for(int k=6;k--;)	A[level*itsperlevel+(it-1)].set( tempA.get(1,k+1), count, k+1); 
     					//1x6 lines
#ifdef verbose_debug_A
                        cout << "Matrice A : " << endl;
                        //A[level*itsperlevel+(it-1)].afficher();
                        tempA.afficher();
                        cout << "LEVEL = " << level << " : " << it << endl;
#endif
	     	    	     	    

	     	    y[level*itsperlevel+(it-1)].set( r( solutions[level*itsperlevel+(it-1)], xi).get(1,1), count, 1);
	     				//nx1 vector where n = #Sigma
#ifdef verbose_debug_y
                        cout << "Vector Y : " << endl;
                        y[level*itsperlevel+(it-1)].afficher();
                        cout << "LEVEL = " << level << " :" << it << endl;
#endif
	     			
     		    // What is it ?
                    //res[level*itsperlevel+(it-1)].set( (Line(y[level*itsperlevel+(it-1)], (y[level*itsperlevel+(it-1)]).getLine()) + Line( A[level*itsperlevel+(it-1)], (A[level*itsperlevel+(it-1)]).getLine() ) * solutions[level*itsperlevel+(it-1)] ).get(1,1) , xi.get(1,1), xi.get(2,1) );
                    res[level*itsperlevel+(it-1)].set( (Line(y[level*itsperlevel+(it-1)], count) - Line( A[level*itsperlevel+(it-1)], count ) * solutions[level*itsperlevel+(it-1)] ).get(1,1) , xi.get(1,1), xi.get(2,1) );
     		
     		}
     	    }
     	}    	    
     	
     	int nbrLA = count;
     	int countPoint = 0;
     	W[level*itsperlevel+(it-1)] = Mat<T>((T)0, nbrLA, nbrLA);
     	Mat<T> tempW((T)0, 1, nbrLA);
     	
     	for(int i= this->pyramidD1[level].getLine();i--;)
     	{
            xi.set((T)i+1, 1,1);
     		
            for(int j= this->pyramidD1[level].getColumn();j--;)
     	    {
                xi.set((T)j+1, 2,1);
                
                if( (this->pyramidD1[level]).get( i+1, j+1) != (T)0)
     		{         
     		    countPoint++;

	     	    JrW( &tempW, solutions[level*itsperlevel+(it-1)], xi, countPoint, nbrLA, it);
	     	    for(int k=nbrLA;k--;)
	     	    {
	     	    	W[level*itsperlevel+(it-1)].set( tempW.get(1,k+1), countPoint, k+1);	// nxn matrix where n = #Sigma.
	     	    	tempW.set( (T)0, 1,k+1);
	     	    }
	     	    
#ifdef verbose_debug_W
		        cout << "Matrice W : " << endl;
		        W[level*itsperlevel+(it-1)].afficher();
		        cout << "LEVEL = " << level << " : " << it << endl;
#endif
	        }
	    }
	}
	
	return nbrLA;
                	    
    }
    
    inline Mat<T> r(const Mat<T>& khi,const  Mat<T>& x)
    {
        Mat<T> rim( (this->pyramidI2[level]).get( warp(khi,x) ) - (this->pyramidI1[level]).get(x) );

#ifdef verbose_debug_r
        cout << "Value I2 : " << (this->pyramidI2[level]).get( warp(khi,x) ).get(1,1) << " Value I1 " <<  (this->pyramidI1[level]).get(x).get(1,1) << endl;
        cout << "x1 : " << endl;
        Mat<T> temp(x);
        temp.afficher();
        cout << "x2 : " << endl;
        warp(khi,x).afficher();
        cout << " RIM = " << endl;
        rim.afficher();
        cout << "/////////////////////////" << endl;
#endif

        return rim;
        //1x1 in grayscale.
    	//watch out the level.
    }
    
    inline Mat<T> warp(const Mat<T>& khi, const Mat<T>& x)
    {
    	return proj( rigidMotion( khi, retroproj( x) ) );
    }
    
    inline Mat<T> proj(const Mat<T>& P)
    {
        /*
        Mat<T> rim(2,1);
        rim.set( (T)this->K->get(1,1)*P.get(1,1)/(T)P.get(3,1), 1,1);
        rim.set( (T)this->K->get(2,2)*P.get(2,1)/(T)P.get(3,1), 2,1);
    	
    	return rim;
        */
        
        
        Mat<T> ret( (this->pyramidK[level])*P );
        //Mat<T> ret((*(this->K))*P);
        homogeneousNormalization(&ret);
        return ret;
        
    }
    
    inline Mat<T> retroproj(const Mat<T>& x)
    {
        /*
        Mat<T> rim(3,1);
        rim.set( (T)(x.get(1,1)-this->K->get(1,3))/this->K->get(1,1), 1,1);
        rim.set( (T)(x.get(2,1)-this->K->get(2,3))/this->K->get(2,2), 2,1);
        rim.set( (T)1, 3,1);
    	rim = Z(x)*rim;
    	
    	return rim;    	 
        */
        Mat<T> rim(3,1);
        rim.set(x.get(1,1),1,1);
        rim.set(x.get(2,1), 2,1);
        rim.set((T)1,3,1);
        
        
        //rim = (*(this->invK))*rim;
        rim = (this->pyramidKinv[level])*rim;
        
        //rim = (T)(((float)Z(x)/(float)rim.get(3,1)))*rim;
        //Test.... :
        rim = Z(x)*rim;
        return rim;
    }
    
    inline T Z(const Mat<T>& x)
    {        
        return     (T)((float)1.0/(float)(this->pyramidD1[level].get( x.get(1,1),x.get(2,1) )) );
    	
    	//watch out the level.
    }
    
    inline Mat<T> rigidMotion(const Mat<T>& khi,const Mat<T>& P)
    {
        Mat<T> khi_hat(expMSIM3( operatorC( khi, Mat<T>((T)1,1,1)) ) );
    	Mat<T> rot(extract(khi_hat, 1,1, 3,3) );
    	Mat<T> t( extract(khi_hat, 1,4, 3,4) );
#ifdef verbose_debug_rigidMotion
        cout << "rigidMotion : " << endl;
        cout << "old P : " << endl;
        Mat<T> temp(P);
        temp.afficher();
        cout << "new P : " << endl;
        (rot*P+t).afficher();
        rot.afficher();
        t.afficher();
#endif
    	
    	return rot*P+t;
    }
    
    /*
    inline Mat<T> Jr(const Mat<T>& khi,const Mat<T>& x)
    {
        int n = khi.getLine();
        Mat<T> delta((T)0, n,1);        

        if(isnan(h))
            h = pow(numeric_limits<T>::epsilon(), (T)0.5);

#ifdef verbose_debug_mini_JR
        cout << " h = " << h << endl;
#endif

        delta.set(h, 1,1);

        Mat<T> x1(khi+delta);
        Mat<T> x2(khi-delta);
        Mat<T> temp(r(x1, x) - r(x2, x));
        Mat<T> grad( (T)0, temp.getLine(), n);
        
        for(int i=temp.getLine();i--;) grad.set( temp.get(i+1,1), i+1,1);

        for(int i=2;i<=n;i++)
        {
            delta.set((T)0,i-1,1);
            delta.set(h,i,1);

            x1 = khi+delta;
            x2 = khi-delta;
            //grad = operatorL(grad, r(x1, x) - r(x2, x)    );
            Mat<T> tempr(r(x1, x) - r(x2, x));

            for(int j=tempr.getLine();j--;) grad.set( tempr.get(j+1,1), j+1,i);
            
            

        }

        return ((T)((T)1.0/(2*h)))*grad;
        // 1x6 in grayscale.
    }
    */
    inline void Jr(Mat<T>* tempA, const Mat<T>& khi,const Mat<T>& x)
    {
        int n = khi.getLine();
        Mat<T> delta((T)0, n,1);        
	T pownormekhi = pow(norme1(khi),(T)1);
	h = pow(numeric_limits<T>::epsilon(), (T)0.5)* pownormekhi;//( 1 >= pownormekhi ? 1 : pownormekhi);

        if(isnan(h))
            h = pow(numeric_limits<T>::epsilon(), (T)0.5);

#ifdef verbose_debug_JR
        cout << " h = " << h << endl;
#endif

        delta.set(h, 1,1);

        Mat<T> x1(khi+delta);
        Mat<T> x2(khi-delta);
        volatile T dx = norme2(x1-x2);
        Mat<T> temp(r(x1, x) - r(x2, x));
        
        for(int i=temp.getLine();i--;) tempA->set( temp.get(i+1,1), i+1,1);

        for(int i=2;i<=n;i++)
        {
            delta.set((T)0,i-1,1);
            delta.set(h,i,1);

            x1 = khi+delta;
            x2 = khi-delta;
            //grad = operatorL(grad, r(x1, x) - r(x2, x)    );
            temp = (r(x1, x) - r(x2, x));


            for(int j=1;j<=temp.getLine();j++) 
            	tempA->set( (T)((float)1.0/(dx))*temp.get(j,1), j,i);
            
        }

        // 1x6 in grayscale.
    }

    inline Mat<T> JrCR(const Mat<T>& khi,const Mat<T>& x)
    {
        Mat<T> warp0(warp(khi,x));
        int nkhi = khi.getLine();
        int nwarp = warp0.getLine();
        Mat<T> deltakhi((T)0, nkhi,1);
        Mat<T> deltawarp((T)0, nwarp,1);

        if(isnan(h))
            h = pow(numeric_limits<T>::epsilon(), (T)1);

#ifdef verbose_debug_mini_JR
        cout << " h = " << h << endl;
#endif

        deltakhi.set(h, 1,1);
        deltawarp.set(h,1,1);

        Mat<T> khi1(khi+deltakhi);
        Mat<T> khi2(khi-deltakhi);
        Mat<T> warp1(warp0+deltawarp);
        Mat<T> warp2(warp0-deltawarp);
        Mat<T> gradkhi( warp(khi1, x) - warp(khi2, x) );
        Mat<T> gradwarp( (this->pyramidI2[level]).get( warp1 ) - (this->pyramidI2[level]).get( warp2) );

        for(int i=2;i<=nkhi;i++)
        {
            deltakhi.set((T)0,i-1,1);
            deltakhi.set(h,i,1);

            khi1 = khi+deltakhi;
            khi2 = khi-deltakhi;
            gradkhi = operatorL(gradkhi, warp(khi1, x) - warp(khi2, x) );

        }

        for(int i=2;i<=nwarp;i++)
        {
            deltawarp.set((T)0,i-1,1);
            deltawarp.set(h,i,1);

            warp1 = warp0+deltawarp;
            warp2 = warp0-deltawarp;
            gradwarp = operatorL(gradwarp, (this->pyramidI2[level]).get( warp1 ) - (this->pyramidI2[level]).get( warp2) );

        }

        return ((T)((T)1.0/(2*h)))*(gradwarp*gradkhi);
        // 1x6 in grayscale.
    }
    
    /*
    inline Mat<T> JrW(const Mat<T>& khi,const Mat<T>& x, int numeroPoint, int nbrLA, int it)
    {
    	Mat<T> rres((T)0,1,nbrLA);
    	T val = (T)1.0;//res[level*itsperlevel+(it-1)].get(x.get(1,1),x.get(2,1));
    	val = (T)1.0/this->pyramidvar_depthMapI1[level*itsperlevel+(it-1)].get(x.get(1,1),x.get(2,1));
    	
    	if(fabs_(val) < (T)10)
    		val = (T)1;
    	rres.set( (T)1*val , 1, numeroPoint);
    	return rres;    
    }
    */
    
    inline void JrW( Mat<T>* tempW, const Mat<T>& khi,const Mat<T>& x, int numeroPoint, int nbrLA, int it)
    {
    	T val = (T)1.0/*res[level*itsperlevel+(it-1)].get(x.get(1,1),x.get(2,1))*/;
    	val = (T)1.0/this->pyramidvar_depthMapI1[level*itsperlevel+(it-1)].get(x.get(1,1),x.get(2,1));
    	
    	if(val < (T)10)
    		val = (T)1;
    	tempW->set( (T)val*val , 1, numeroPoint);    
    }
    
    Mat<T> getResidual( int level)
    {
    	if(level <= nbrL && level >= 0)
    	{
    		return res[level*itsperlevel+(itsperlevel-1)];
    	}
    	else
    	{
    		return res[nbrL*itsperlevel+(itsperlevel-1)];
    	}
    }
};


#endif
