#include "../MATv2/Mat.h"
#define dim 3
#include <vector>
using namespace std;

template<typename T>
class GNA
{

	private :
	Mat<T> Beta;					//variables
	vector<Mat<T> > params;				//argument to the function.
	Mat<T> realValue;			//real values to the functions on each Beta according to params.
	
	T alpha;// learning ration.
	Mat<T> Delta;					//Error term.
	Mat<T> J;					//Jacobian to store.
	
	Mat<T> (*ptrF)(vector<Mat<T> >, Mat<T>);		//ptr to the function.
	Mat<T> (*ptrJ)(vector<Mat<T> >, Mat<T>);		//ptr to the jacobian constructor.
	
	
	
	public:
	
	GNA( Mat<T> (*InitB)(void), vector<Mat<T> > params_, Mat<T> rV, Mat<T> (*Ff)(vector<Mat<T> >, Mat<T>), Mat<T> (*Jf)(vector<Mat<T> >, Mat<T>), int iteration)
	{
		ptrF = Ff;
		ptrJ = Jf;
		Mat<T> (*init)(void) = InitB;
		
		realValue = rV;
		params = params_;
        alpha = (T)0.0001;
		
		Beta = init();

		cout << "GNA : Initialization : DONE." << endl;	
        Mat<T> cost(realValue -ptrF(params, Beta));

		for(int i=1;i<=iteration;i++)
		{
			cout << "///////////////////////////\nGNA : Running : iteration " << i << endl;
			J = ptrJ(params, Beta);
            //Beta.afficher();
            //cout << "GNA : J : computed : "<< endl;
			
            //J.afficher();
			
			Mat<T> temp(invGJ(transpose(J)*J));
			temp = temp*transpose(J);
            //temp.afficher();
			
            //cout << "GNA : inverse of J : computed." << endl;
			Delta = temp * (realValue - ptrF(params, Beta));
            //cout << "GNA : Delta : computed." << endl;
            cost = (realValue-ptrF(params,Beta)) - cost;
            cost.afficher();
            if(!(cost <= Mat<T>((T)0,cost.getLine(),cost.getColumn()) ) )
                alpha = (alpha >= numeric_limits<T>::epsilon()*10e2 ? alpha/10 : alpha);
            else
                alpha = alpha*10;

            cout << "alpha =" << alpha << endl;

            cost = (realValue-ptrF(params,Beta));


			
			temp = Delta;
			int dimx=Beta.getLine();
			int dimy=Beta.getColumn();
			Delta = Mat<T>((T)0, dimx,dimy);
			
			for(int i=1;i<=dimx;i++)
			{
				for(int j=1;j<=dimy;j++)
				{
					Delta.set( temp.get( i*j, 1), i, j);
				}
			}
			
            //Delta.afficher();
            //Beta.afficher();
            Beta = Beta + ((T)alpha)*Delta;
			cout << "GNA : Beta : updated : "<< endl;
			Beta.afficher();
		}

	}
	
	
	~GNA()	{};
	
	Mat<T> getBeta()	const
	{
		return Beta;
	}

};



template<typename T>
Mat<T> initGLrand()
{
	Mat<T> r(dim,dim, (char)1);
	T det = computeDeterminant(r);
	
	do
	{
		r.afficher();
		r =Mat<T>(dim,dim, (char)1);
		det = computeDeterminant(r);			
				
	}while(det == 0);
	
	return r;
}

template<typename T>
Mat<T> SFM_F(vector<Mat<T> > params, Mat<T> Beta)	// with params[f] = (If  Jf) column... F columns
{
	int F_ = params.size();
	Mat<T> r( 3,1);
	Mat<T> B(Beta);
	
	r.set( ( transpose( extract(params[0], 1,1, 3,1)) * B * transpose(B) * extract(params[0], 1,1, 3,1) ).get(1,1), 1,1);
	r.set( ( transpose( extract(params[0], 1,2, 3,2)) * B * transpose(B) * extract(params[0], 1,2, 3,2) ).get(1,1), 2,1);
	r.set( ( transpose( extract(params[0], 1,1, 3,1)) * B * transpose(B) * extract(params[0], 1,2, 3,2) ).get(1,1), 3,1);		
	
	for(int f=1;f<=F_-1;f++)
	{
		Mat<T> temp(3,1);
		
		temp.set( ( transpose( extract(params[f], 1,1, 3,1)) * B * transpose(B) * extract(params[f], 1,1, 3,1) ).get(1,1), 1,1);
		temp.set( ( transpose( extract(params[f], 1,2, 3,2)) * B * transpose(B) * extract(params[f], 1,2, 3,2) ).get(1,1), 2,1);
		temp.set( ( transpose( extract(params[f], 1,1, 3,1)) * B * transpose(B) * extract(params[f], 1,2, 3,2) ).get(1,1), 3,1);
		
		r = operatorC(r, temp);
        //cout << "Etape F : " << f << " / " << F_-1 << endl;
		//r.afficher();
        //Beta.afficher();
	}
	
	return r;
}


template<typename T>
Mat<T> SFM_J(vector<Mat<T> > params, Mat<T> Beta)
{
	int F_ = params.size();
	int n = Beta.getLine();
	Mat<T> B(Beta);
	Mat<T> r( 3, n*n);
	
	for(int k=1;k<=n;k++)
	{
		for(int l=1;l<=n;l++)
		{
			r.set( ( transpose( extract(params[0], 1,1, 3,1)) * derive(B, k,l) * transpose(B) * extract(params[0], 1,1, 3,1) ).get(1,1) + ( transpose( extract(params[0], 1,1, 3,1)) * B * transpose( derive(B, k,l) ) * extract(params[0], 1,1, 3,1) ).get(1,1), 1,k*l);
			r.set( ( transpose( extract(params[0], 1,2, 3,2)) * derive(B, k,l) * transpose(B) * extract(params[0], 1,2, 3,2) ).get(1,1) + ( transpose( extract(params[0], 1,2, 3,2)) * B * transpose( derive(B, k,l)) * extract(params[0], 1,2, 3,2) ).get(1,1), 2,k*l);
			r.set( ( transpose( extract(params[0], 1,1, 3,1)) * derive(B, k,l) * transpose(B) * extract(params[0], 1,2, 3,2) ).get(1,1) + ( transpose( extract(params[0], 1,1, 3,1)) * B * transpose( derive(B, k,l)) * extract(params[0], 1,2, 3,2) ).get(1,1), 3,k*l);		
		}
	}
	
	
	
	for(int f=1;f<=F_-1;f++)
	{
		Mat<T> temp(3,n*n);
		
		for(int k=1;k<=n;k++)
		{
			for(int l=1;l<=n;l++)
			{
				temp.set( ( transpose( extract(params[f], 1,1, 3,1)) * derive(B, k,l) * transpose(B) * extract(params[f], 1,1, 3,1) ).get(1,1) + ( transpose( extract(params[f], 1,1, 3,1)) * B * transpose( derive(B, k,l) ) * extract(params[f], 1,1, 3,1) ).get(1,1), 1,k*l);
				temp.set( ( transpose( extract(params[f], 1,2, 3,2)) * derive(B, k,l) * transpose(B) * extract(params[f], 1,2, 3,2) ).get(1,1) + ( transpose( extract(params[f], 1,2, 3,2)) * B * transpose( derive(B, k,l)) * extract(params[f], 1,2, 3,2) ).get(1,1), 2,k*l);
				temp.set( ( transpose( extract(params[f], 1,1, 3,1)) * derive(B, k,l) * transpose(B) * extract(params[f], 1,2, 3,2) ).get(1,1) + ( transpose( extract(params[f], 1,1, 3,1)) * B * transpose( derive(B, k,l)) * extract(params[f], 1,2, 3,2) ).get(1,1), 3,k*l);		
			}
		}
		
		r = operatorC(r, temp);
        //cout << "Etape J : " << f << " / " << F_-1 << endl;
		//r.afficher();
        //B.afficher();
	}
	
	return r;
}
