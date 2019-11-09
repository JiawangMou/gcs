#ifndef _JOYSTICK_WIDGET_H_
#define _JOYSTICK_WIDGET_H_

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <stdio.h>
#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#endif

class JoystickWidget: public QWidget
{
    Q_OBJECT

    public:
        explicit JoystickWidget(QWidget *parent = nullptr);
        void paintEvent(QPaintEvent *e);
};
#endif