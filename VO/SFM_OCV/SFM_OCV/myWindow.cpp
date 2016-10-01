#include "myWindow.h"

myWindow::myWindow(int fps, QWidget* parent) : myGLWidget(fps,parent,"SLAM")
{
    cam = Camera();

    posMouse = Mat<double>(0.0,2,1);

    //this->setMouseTracking(true);
    test = 0.0;
    this->fps = fps;
}

void myWindow::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}


void myWindow::resizeGL(int width, int height)
{
    if(height == 0)
        height = 1;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(75.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void myWindow::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();   

    gluLookAt(GLdouble(cam.getPosX()), GLdouble(cam.getPosY()), GLdouble(cam.getPosZ()),
              GLdouble(cam.getTargetX()), GLdouble(cam.getTargetY()), GLdouble(cam.getTargetZ()),
              //0,0,0,
              GLdouble(cam.getVertX()), GLdouble(cam.getVertY()), GLdouble(cam.getVertZ()) );

    //test = (float)((int)(test +1.0)%10);
    glBegin(GL_QUADS);
        glVertex3f(-100.0f, 100.0f, 0.0f);
        glVertex3f(-100.0f, -100.0f, 0.0f);
        glVertex3f(100.0f, -100.0f, 0.0f);
        glVertex3d(100.0f, 100.0f, 0.0f);
    glEnd();

    glTranslatef(-1.5f, 0.0f-test, -6.0f);

    glBegin(GL_TRIANGLES);
        glVertex3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-1.0f, -1.0f, 0.0f);
        glVertex3f(1.0f, -1.0f, 0.0f);
    glEnd();

    glTranslatef(3.0f, 0.0f, -6.0f);

    glBegin(GL_QUADS);
        glVertex3f(-1.0f, 1.0f, 0.0f);
        glVertex3f(-1.0f, -1.0f, 0.0f);
        glVertex3f(1.0f, -1.0f, 0.0f);
        glVertex3d(1.0f, 1.0f, 0.0f);
    glEnd();
    glBegin(GL_QUADS);
        glVertex3f(-1.0f, 1.0f, 1.0f);
        glVertex3f(-1.0f, -1.0f, 1.0f);
        glVertex3f(1.0f, -1.0f, 1.0f);
        glVertex3d(1.0f, 1.0f, 1.0f);
    glEnd();



}

void myWindow::mouseMoveEvent(QMouseEvent * event)
{

   int dx = 0;
   int dy = 0;


   dx = event->x() - posMouse.get(1,1);
   dy = event->y() - posMouse.get(2,1);

   posMouse.set( event->x(), 1,1);
   posMouse.set( event->y(), 2,1);

   //dx = dx/fps*0.05;
   //dy = dy/fps*0.001;
   cam.orienter( (double)dx/fps, (double)dy/fps);
   //cout << " x = " << event->x() << " \t y = " << event->y() << endl;
   cout << " dx = " << dx << " \t dy = " << dy << endl;
   //cout << "POSITION : " << endl;
   //(cam.getPos()).afficher();
   //cout << "TARGET : " << endl;
   //(cam.getTarget()).afficher();
}

