#include <stdio.h>
#include <ros/ros.h>
#include "rviz_teleop_commander/joystick_widget.h"



JoystickWidget::JoystickWidget(QWidget* parent):
    QWidget(parent){

    m_isPressedInside = false;

}

void JoystickWidget::paintEvent(QPaintEvent *e){
    
    QPainter painter(this);
    painter.save();

    QBrush push_ball_brush;
    QRadialGradient push_ball_brush_color;
    push_ball_brush_color.setColorAt(0,QColor(0,0,0));
    push_ball_brush_color.setColorAt(1,QColor(255,255,255));
    push_ball_brush_color.setRadius(20);
    push_ball_brush_color.setCoordinateMode(QGradient::CoordinateMode::StretchToDeviceMode);
    push_ball_brush_color.setCenter(QPoint(0,0));

    painter.setPen(Qt::NoPen);
    QColor bgColor(50,50,50);
    painter.setBrush(push_ball_brush_color);
    painter.drawEllipse(this->rect().center(),20,20);
    painter.restore();

}

void JoystickWidget::mouseMoveEvent(QMouseEvent *e){

    ROS_INFO("x:%d,y:%d",e->x(),e->y());
    Q_EMIT JoystickValueChanged(e->x(),e->y());
    update();
}

void JoystickWidget::mousePressEvent(QMouseEvent *e){
    ROS_INFO("press:x:%d,y:%d",e->x(),e->y());
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *e){
    ROS_INFO("release:x:%d,y:%d",e->x(),e->y());
}