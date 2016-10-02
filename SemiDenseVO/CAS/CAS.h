#ifndef CAS_H
#define CAS_H

#include "../../MATv2/Mat.h"

template<typename T>
class QRter
{
	public :
	
	Mat<T> Q;
	Mat<T> R;
	
	QRter(const Mat<T>& A) : Q(Mat<T>((T)0.0f,A.getLine(),A.getLine())), R(Mat<T>(A))
	{
		int m = A.getLine();
		int n = A.getColumn();
		for(int i=1;i<=m;i++)	Q.set( 1.0f, i,i);
		
		for(int j=1;j<=n-1;j++)
		{
			volatile T normx;
			Mat<T> x( extract(R, 1,j,m,j) );
			normx = sqrt((transpose(x)*x).get(1,1));
			
			//let us zero the upper values :
			for(int i=1;i<j;i++)	x.set( 0,i,1);
			
			Mat<T> ej((T)0,m,1);
			ej.set(1,j,1);
			
			Mat<T> dummy(x-(T)normx*ej);
			volatile T uj = sqrt( (transpose(dummy)*dummy).get(1,1) );
			
			
			Mat<T> w( (T)(1.0f/uj)*dummy );
			
			
			Mat<T> H( (-2.0f)*(w*transpose(w)));
			for(int i=1;i<=H.getLine();i++)	H.set(1.0f+H.get(i,i), i,i);
			
			/*
			std::cout << "H symmetry and orthogonality : " << std::endl;
			H.afficher();
			(transpose(H)*H).afficher();
			(H*H).afficher();
			*/
			R = H*R;
			Q = Q*H;
			
			/*
			std::cout << "STEP : " << j << std::endl;
			Q.afficher();
			R.afficher();
			std::cout << "STEP : orthogonality of Q : " << j << std::endl;
			(Q*transpose(Q)).afficher();
			*/
		}
	}
	
	Mat<T> getR()	const
	{
		return R;
	}
	
	Mat<T> getQ()	const
	{
		return Q;
	}
};


#define verbose
template<typename T>
class SVDbis
{

    private :

    Mat<T>* A_;
    QRter<T>* qr1;
    QRter<T>* qr;
    Mat<T>* U;
    Mat<T>* S;
    Mat<T>* V;

    public :

    SVDbis(Mat<T> A, int iteration = 100)
    {
        /* QR decomposition */
        qr1 = new QRter<T>(A);
        Mat<T> Q1(qr1->getQ());
        Mat<T> R1(qr1->getR());
#ifdef verbose
        cout << "//////////////////////////////" << endl;
        cout << "Matrice Q1 : " << endl;
        Q1.afficher();
        //(Q1*transpose(Q1)).afficher();
        cout << "Matrice R1 : " << endl;
        R1.afficher();
        cout << "Matrice PRODUCT : Q1*R1 : " << endl;
        Mat<T> prod(Q1*R1);
        prod.afficher();
        
        /*-------------------*/

        cout << endl << "Decomposition QR : DONE !!!" << endl << endl;
#endif
        /* QR decomposition of R1 */
        qr = new QRter<T>(transpose(R1));
        Mat<T> Q(qr->getQ());
        Mat<T> R(qr->getR());
#ifdef verbose
        cout << "Matrice Q : " << endl;
        Q.afficher();
        cout << "Matrice R : " << endl;
        R.afficher();
        cout << "Matrice R1 : " << endl;
        R1.afficher();
        /*---------------------*/

        cout << endl << "Decomposition QR de R.t : DONE !!!" << endl << endl;

        cout << "Verification : " << endl;
        cout << "Matrice A :" << endl;
        A.afficher();
        cout << "Produit : " << endl;
        (Q1*transpose(R)*transpose(Q)).afficher();
        //cout << "Verification de l'orthogonalité de Q1 : " << endl;
        //(Q1*transpose(Q1)).afficher();
        cout << "Verification de D : " << endl;
        transpose(R).afficher();
        //cout << "Verification de l'orthogonalité de Q : " << endl;
        //(Q*transpose(Q)).afficher();                
        
        /*----------------------------------------*/
        /*----------------------------------------*/
        /*----------------------------------------*/
        cout << "ASSIGNATION ..." << endl;
        R.afficher();
#endif


        Mat<T> D(transpose(R));
        QRter<T>* rec = new QRter<T>(D);
        Mat<T> Qr(rec->getQ());
        Mat<T> rtemp(rec->getR());
        delete rec;
        rec = new QRter<T>(transpose(rtemp));
        Mat<T> tQl(transpose(rec->getQ()));
        D = transpose( rec->getR());

        Mat<T> error(D);
        int dimmax = (error.getLine() > error.getColumn() ? error.getLine() : error.getColumn());
        for(int i=1;i<= dimmax;i++)	error.set((T)0, i,i);
        T eps =  numeric_limits<T>::epsilon();

        while( iteration >= 0 && sum(sum(absM(error))).get(1,1) > eps)
        {
        	iteration--;
        	std::cout << " step : " << iteration << " ; Error : " << sum(sum(absM(error))).get(1,1) << " / " << eps << endl;
#ifdef verbose        
            cout << "Iteration : " << iteration << " : error : " << sum(sum(absM(error))).get(1,1) << " / " << eps << endl;
#endif
            delete rec;
            rec = new QRter<T>(D);
            Qr = Qr * rec->getQ();
            rtemp = rec->getR();
            delete rec;
            rec = new QRter<T>(transpose(rtemp));
            tQl = transpose(rec->getQ()) * tQl;
            D = transpose( rec->getR());
#ifdef verbose
            D.afficher();
#endif

            /////////////////
            error = D;
            dimmax = (error.getLine() > error.getColumn() ? error.getLine() : error.getColumn());
            for(int i=1;i<= dimmax;i++)	error.set((T)0, i,i);


        }
        delete rec;

        Qr = Q1*Qr;
        tQl = tQl*transpose(Q);


#define verbose
#ifdef verbose
        cout << "Methode itérative : " << endl;
        cout << "Matrice originale :" << endl;
        A.afficher();
        cout << "Produit : " << endl;
        (Qr*D*tQl).afficher();
        cout << " U :" << endl;
        Qr.afficher();
        cout << " S :" << endl;
        D.afficher();
        cout << " V :" << endl;
        transpose(tQl).afficher();
#endif        
        ///////////////////////////////////////////////////////////////
	/*
        A_ = new Mat<T>(A);
        U = new Mat<T>(Q1);
        S = new Mat<T>(transpose(R));
        V = new Mat<T>(Q);
        */
        //cout << "Initialement : D vaut :" << endl;
        //transpose(R).afficher();
        
        A_ = new Mat<T>(A);
        U = new Mat<T>(Qr);
        S = new Mat<T>(D);
        V = new Mat<T>(transpose(tQl));

        /*nettoyage */
        /*rien... that's how the cookie crumble x) !! */

    }

    ~SVDbis()
    {
        delete A_;
        delete qr1;
        delete qr;
        delete U;
        delete S;
        delete V;
    }

    Mat<T> getU() const
    {
        return *U;
    }

    Mat<T> getS() const
    {
        return *S;
    }

    Mat<T> getV() const
    {
        return *V;
    }

};


#endif
