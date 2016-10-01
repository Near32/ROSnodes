#ifndef MYWINDOW_H
#define MYWINDOW_H

#include "myGLWidget.h"
#include <QMouseEvent>
#include "camera.h"
#include <qgl.h>
#include <GL/glu.h>

class myWindow : public myGLWidget
{
    Q_OBJECT
public:
        explicit myWindow(int fps = 60, QWidget *parent = 0);
        void initializeGL();
        void resizeGL(int width, int height);
        void paintGL();

        //mouse handler
        void mouseMoveEvent(QMouseEvent* event);

public slots:
        void timeOutSlot(){ updateGL();}

protected :
        Camera cam;
        Mat<double> posMouse;
        float test;
        int fps;


};

#endif // MYWINDOW_H
