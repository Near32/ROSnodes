#ifndef CAMERA_H
#define CAMERA_H
#include "../../MAT/Mat.h"

class Camera
{
public:
    Camera();
    Camera(Mat<double> position, Mat<double> target, Mat<double> axeVertical);
    ~Camera();

    static Mat<double> orienter(Mat<double> pos, Mat<double> targ);
    void orienter(double xRel, double yRel);

    Mat<double> getPos(){   return position;}
    double getPosX(){ return position.get(1,1); }
    double getPosY(){ return position.get(2,1); }
    double getPosZ(){ return position.get(3,1); }

    Mat<double> getTarget(){    return target;}
    double getTargetX(){ return target.get(1,1); }
    double getTargetY(){ return target.get(2,1); }
    double getTargetZ(){ return target.get(3,1); }

    Mat<double> getVert(){  return axeVertical;}
    double getVertX(){ return axeVertical.get(1,1); }
    double getVertY(){ return axeVertical.get(2,1); }
    double getVertZ(){ return axeVertical.get(3,1); }

private :
    double theta;
    double phi;
    Mat<double> orientation;

    Mat<double> axeVertical;
    Mat<double> deplacementLateral;

    Mat<double> position;
    Mat<double> target;

};

#endif // CAMERA_H
