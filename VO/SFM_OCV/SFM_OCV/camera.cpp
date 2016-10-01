#include "camera.h"

Camera::Camera()
{
    phi = 0.0;
    theta = 0.0;

    orientation = Mat<double>(0.0,3,1);
    orientation.set(1.0,3,1);
    // axe optique selon l'axe z.

    axeVertical = Mat<double>(0.0,3,1);
    axeVertical.set(1.0,2,1);
    // vertical selon l'axe y.

    position = Mat<double>(3.0,3,1);
    target = orientation;

    deplacementLateral = Mat<double>(0.0,3,1);
    deplacementLateral.set(-1.0,1,1);
    //orientation x axeVertical = z x y = -x

}

Camera::Camera(Mat<double> position, Mat<double> target, Mat<double> axeVertical) : phi(0.0), theta(0.0), position(position), target(target), axeVertical(axeVertical)
{
    deplacementLateral = crossProduct(orientation,axeVertical);
    deplacementLateral = (1.0/norme2(deplacementLateral))*deplacementLateral;
    //normalization

    orientation = orienter(position,target);
    phi = asin(orientation.get(2,1) );
    theta = atan21(orientation.get(1,1),orientation.get(3,1));
}

Camera::~Camera()
{

}

Mat<double> Camera::orienter(Mat<double> pos, Mat<double> targ)
{
    Mat<double> v(targ-pos);
    double norme = norme2(v);

    if(norme != 0.0)
        v = (1.0/norme)*v;
    else
        v.set(1.0,3,1); // z as optical axis.

    return v;
}

void Camera::orienter(double xRel, double yRel)
{
    phi -= (PI/180)*10*yRel;
    theta -= (PI/180)*10*xRel;

    /*
    if(fabs_(phi) > (PI/180.0)*89.0)
        phi = (PI/180.0) * (phi > 0.0 ? 89.0 :-89.0);
    */
    //limitation de l'angle phi .

    //mise à jour orientation :

    orientation.set( cos(phi)*sin(theta), 1,1);
    orientation.set( sin(phi),2,1);
    orientation.set( cos(phi)*cos(theta),3,1);


    // Si l'axe vertical est l'axe X
    /*

    if(axeVertical.get(1,1) == 1.0)
    {
        // Calcul des coordonnées sphériques

        orientation.set(sin(phi), 1,1);
        orientation.set( cos(phi) * cos(theta), 2,1);
        orientation.set(cos(phi) * sin(theta), 3,1);
    }


    // Si c'est l'axe Y

    else if(axeVertical.get(2,1) == 1.0)
    {
        // Calcul des coordonnées sphériques

        orientation.set( cos(phi) * sin(theta), 1,1);
        orientation.set( sin(phi), 2,1);
        orientation.set( cos(phi) * cos(theta), 3,1);
    }


    // Sinon c'est l'axe Z

    else
    {
        // Calcul des coordonnées sphériques

        orientation.set( cos(phi) * cos(theta), 1,1);
        orientation.set( cos(phi) * sin(theta), 2,1);
        orientation.set( sin(phi), 3,1);
    }
    */


    deplacementLateral = crossProduct(orientation,axeVertical);
    deplacementLateral = (1.0/norme2(deplacementLateral))*deplacementLateral;

    target = position+orientation;

    //axevertical & position have not changed.


}
