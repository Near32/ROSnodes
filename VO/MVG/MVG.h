#ifndef MVG_H
#define MVG_H

#include "../MAT/Mat.h"


template<typename T>
Mat<T> expW( const Mat<T>& w);
template<typename T>
Mat<T> expM( const Mat<T>& khi);
template<typename T>
Mat<T> expM(Mat<T>* khi);
template<typename T>
Mat<T> expMSIM3(const Mat<T>& khi);
template<typename T>
Mat<T> crossProduct(const Mat<T>& n);


template<typename T>
Mat<T> expW( const Mat<T>& w)
{		
	T theta = w.get(1,1)*w.get(1,1);
	for(int i=2;i<=3;i++)	theta += w.get(i,1)*w.get(i,1);
	
	theta = sqrt(theta);
	T norme = norme1(w);
	
	Mat<T> n(3,1);
	if(norme == (T)0)
	{
	    theta = numeric_limits<T>::epsilon();
	    Mat<T> ww = numeric_limits<T>::epsilon()*Mat<T>(w.getLine(), w.getColumn(), (char)1); 
	    n = (((T)1.0/norme1(ww))*ww);
	}
	else
	{
	   n = (((T)1.0/norme1(w))*w);
	}
#ifdef expW_debug	
	cout << "ANGLE : " << theta << endl;
	cout << "Unit Vector n: " << endl;
	n.afficher();
#endif
	
    Mat<T> nx(crossProduct<T>(n));
	
#ifdef expW_debug
	cout << "[N]x : " << endl;
	nx.afficher();	
	cout << "[Omega]x : " << endl;
	(theta*nx).afficher();
#endif	
	
	Mat<T> id((T)0.0,3,3);
	for(int i=1;i<=3;i++)	id.set((T)1.0,i,i);
	
	Mat<T> nx2(n*transpose(n)-id);
	
	//Rodriguez Formula
	return id - (T)sin(theta)*nx + ((T)1.0-(T)cos(theta))*nx2;

}


template<typename T>
Mat<T> expM(const Mat<T>& khi)
{
	return operatorL( operatorC( expW( extract(khi,1,1,3,1) ), Mat<T>((T)0,1,3) ), operatorC( extract(khi, 4,1, 6,1), Mat<T>((T)1,1,1) ) );
}

template<typename T>
Mat<T> expM(Mat<T>* khi)
{
	return operatorL( operatorC( expW( extract(khi,1,1,3,1) ), Mat<T>((T)0,1,3) ), operatorC( extract(khi, 4,1, 6,1), Mat<T>((T)1,1,1) ) );
}

template<typename T>
Mat<T> expMSIM3(const Mat<T>& khi)
{
    return operatorL( operatorC( khi.get(7,1)*expW( extract(khi,1,1,3,1) ), Mat<T>((T)0,1,3) ), operatorC( extract(khi, 4,1, 6,1), Mat<T>((T)1,1,1) ) );
}

template<typename T>
Mat<T> crossProduct(const Mat<T>& n)
{
    Mat<T> r((T)0.0,3,3);
    r.set(-n.get(3,1),1,2);
    r.set(n.get(3,1),2,1);

    r.set(n.get(2,1),1,3);
    r.set(-n.get(2,1),3,1);

    r.set(-n.get(1,1),2,3);
    r.set(n.get(1,1),3,2);

    return r;
}

#endif

