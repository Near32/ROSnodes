#include "myGLWidget.h"

myGLWidget::myGLWidget(int framesPerSecond, QWidget *parent, char *name) : QGLWidget(parent)
{
    setWindowTitle(QString::fromUtf8(name));
    if(framesPerSecond==0)
        t_Timer = NULL;
    else
    {
        int secondes = 1000;
        int timerInterval = secondes/framesPerSecond;
        t_Timer = new QTimer(this);
        connect(t_Timer,SIGNAL(timeout()), this,SLOT(timeOutSlot()) );
        t_Timer->start(timerInterval);
    }       
}

void myGLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
    switch(keyEvent->key())
    {
        case Qt::Key_Escape:
            close();
            break;
    }
}

void myGLWidget::timeOutSlot()
{
}
