#include "MVG.h"
#include "../ONSC/OO.h"
#include "../ONSC/ONSC.h"
#include "../ONSC/ONSCNewton.h"

#define v_bench
int main(int argc, char* argv[])
{
    int h = 20;
    int w = 20;
    Mat<double> im1(200.0,h,w);
    Mat<double> im2(im1);
    Mat<double> dm1(100.0,h,w);

    //Benchmark creation
    for(int i=h/10;i<=h-h/10;i++)
    {
        im1.set(10.0, i, 4*w/10);
        im1.set(10.0, i, 4*w/10+1);
        im1.set(10.0, i, 4*w/10+2);
    }
    for(int j=w/10;j<=w-w/10;j++)
    {
    	im1.set(10.0, 4*h/10, j);
        im1.set(10.0, 4*h/10+1, j);
        im1.set(10.0, 4*h/10+2, j);
    }
    for(int i=h/10;i<=h-h/10;i++)
    {
        im2.set(9.0, i, 4*w/10+5);
        im2.set(10.0, i, 4*w/10+1+5);
        im2.set(11.0, i, 4*w/10+2+5);
    }
    for(int j=w/10;j<=w-w/10;j++)
    {
    	im2.set(10.0, 4*h/10+5, j);
        im2.set(10.0, 4*h/10+1+5, j);
        im2.set(10.0, 4*h/10+2+5, j);
    }

    for(int i=h/10;i<=h-h/10;i++)
    {
        dm1.set(105.0, i, 4*w/10);
        dm1.set(110.0, i, 4*w/10+1);
        dm1.set(115.0, i, 4*w/10+2);        
    }
    for(int j=w/10;j<=w-w/10;j++)
    {
    	dm1.set(120.0, 4*h/10, j);
        dm1.set(120.0, 4*h/10+1, j);
        dm1.set(120.0, 4*h/10+2, j);
    }
    //------------------

    Frame<double> f1(im1,1);
    Frame<double> f2(im2,1);

    Mat<double> K(0.0,3,3);
    K.set(500.0,1,1); //fx
    K.set(500.0,2,2); //fy
    K.set((double)h/2,1,3); //cx
    K.set((double)w/2,2,3); //cy
    K.set(1.0,3,3);
    Mat<double> invK(invGJ(K));

#ifdef v_bench
    cout << "Intrinsic Matrix : " << endl;
    K.afficher();
    cout << "Inverse Intrinsic Matrix : " << endl;
    invK.afficher();
    (K*invK).afficher();

    //afficherMat<double>(&im1,&im2, NULL, false, 4.0);
#endif

    clock_t timer = clock();

    int it = 4;
    OODirectSlamSIM3<double> instanceOO(&f1,&f2,&K,&invK,&dm1);
    ONSC<double> instanceONSC( &instanceOO, it);
    Mat<double> stack(instanceONSC.getX(it));
    
    cout << "L'execution a prise : " << (double)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;

    if(it <= 100)
    {
        //regardons l'evolution de notre variable :
        
        for(int i=it;i--;)
            stack = operatorL(stack, instanceONSC.getX(i));
        stack.afficher();
    }
    
    timer = clock();
    (expW(extract(stack,1,1,3,1) )).afficher();
    cout << "L'execution a prise : " << (double)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;
    
    
    
    

    return 0;
}

