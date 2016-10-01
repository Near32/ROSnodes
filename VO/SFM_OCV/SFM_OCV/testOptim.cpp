#include "../../MATv2/Mat.h"

int main(int argc, char* argv[])
{
	Mat<double> A(8000,8000,(char)1);
	Mat<double> B(8000,6,(char)1);
	/*
	for(int i=4;i--;)
	{
		A.set((double)0, 4,i+1);
		A.set((double)0, i+1,4);
		B.set((double)0, 4,i+1);
		//B.set((double)0, i+1,4);
		A.set(1.0,4,4);
		B.set(1.0,4,4);
	}
	*/
	//A.afficher();
	//B.afficher();
		
	clock_t timer = clock();
	
	A =A*B;
	
	cout << "l'execution a prise : " << (double)(clock()-timer)/CLOCKS_PER_SEC << endl;
	//A.afficher();
	

	return 0;
}
