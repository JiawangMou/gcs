#include <stdio.h>
#include "rviz_teleop_commander/joystick_widget.h"



JoystickWidget::JoystickWidget(QWidget* parent):
    QWidget(parent){


}

void JoystickWidget::paintEvent(QPaintEvent *e){

    QPainter painter(this);
    painter.save();
    QPen pen;
    pen.setColor(QColor(255,0,0));
    pen.setWidth(8);
    painter.setPen(pen);
    QColor bgColor(50,50,50);
    painter.setBrush(bgColor);
    painter.drawRoundedRect(this->rect().center().x(),this->rect().center().y(),50,50,5,5);
    painter.restore();

}