#ifndef _JOYSTICK_WIDGET_H_
#define _JOYSTICK_WIDGET_H_

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <stdio.h>
#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBrush>
#endif

class JoystickWidget: public QWidget
{
    Q_OBJECT
    

    public:
        explicit JoystickWidget(QWidget *parent = nullptr);
        void paintEvent(QPaintEvent *e);
        void mouseMoveEvent(QMouseEvent *e);
        void mousePressEvent(QMouseEvent *e);
        void mouseReleaseEvent(QMouseEvent *e);
        void resizeEvent(QResizeEvent *e);

        void setJoystickPos(float x, float y);

    public:
        
    
    private:
        QPoint m_mousePosWhenPressed;
        QPoint m_joystickPos;
        bool m_isPressedInside;
        int m_pushRadius;
        int m_padRadius;

    Q_SIGNALS:
        void JoystickValueChanged(float,float);

};
#endif