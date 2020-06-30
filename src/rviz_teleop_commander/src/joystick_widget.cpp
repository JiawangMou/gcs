#include <stdio.h>
#include <ros/ros.h>
#include <math.h>
#include "rviz_teleop_commander/joystick_widget.h"



JoystickWidget::JoystickWidget(QWidget* parent):
    QWidget(parent){

    m_isPressedInside = false;
    m_pushRadius = 20;
    m_padRadius = 50;
    m_joystickPos.setX(rect().center().x());
    m_joystickPos.setY(rect().center().y());
    m_joystickValueX = m_joystickValueY = 0.0;

}

void JoystickWidget::resizeEvent(QResizeEvent *e){

    if(!m_isPressedInside){
        m_joystickPos.setX(rect().center().x());
        m_joystickPos.setY(rect().center().y());
    }
}


void JoystickWidget::paintEvent(QPaintEvent *e){
    
    QPainter painter(this);

    QFont font;
    font.setPixelSize(15);

    //draw move pad
    painter.save();
    painter.translate(rect().center());
    QRadialGradient move_pad_brush(0, 0, m_padRadius * 1.1 ,0, 0);
    move_pad_brush.setColorAt(1 / 1.1, QColor(100,100,100));
    move_pad_brush.setColorAt(1, QColor(255, 255, 255, 0));
    QPainterPath big_circle, small_circle;
    big_circle.addEllipse(QPoint(0, 0), m_padRadius * 1.1, m_padRadius * 1.1);
    small_circle.addEllipse(QPoint(0, 0), m_padRadius, m_padRadius);
    QPainterPath path = big_circle - small_circle;
    painter.setPen(Qt::NoPen);
    painter.setBrush(move_pad_brush);
    painter.drawPath(path);
    painter.restore();

    //draw text
    painter.save();
    painter.setPen(Qt::black);
    painter.setFont(font);
    QFontMetrics pfm = painter.fontMetrics();
    painter.drawText(QPoint(rect().center().x() - pfm.width("- 前进 -") / 2,
                            rect().center().y() - m_padRadius * 1.2 ), "- 前进 -");
    painter.drawText(QPoint(rect().center().x() - pfm.width("- 后退 -") / 2,
                            rect().center().y() + m_padRadius * 1.2 + pfm.height()), "- 后退 -");
    painter.drawText(QPoint(rect().center().x() - pfm.width("- 左移 -") - m_padRadius * 1.2,
                            rect().center().y() + pfm.height() / 2), "- 左移 -");
    painter.drawText(QPoint(rect().center().x() + m_padRadius * 1.2,
                            rect().center().y() + pfm.height() / 2), "- 右移 -");
    painter.restore();

    //draw push button
    painter.save();
    QLinearGradient push_btn_brush;
    if(m_isPressedInside){
        push_btn_brush.setColorAt(0, QColor(50,50,50));
        push_btn_brush.setColorAt(1, QColor(150,150,150));
    }
    else{
        push_btn_brush.setColorAt(1, QColor(50,50,50));
        push_btn_brush.setColorAt(0, QColor(150,150,150));
    }
    push_btn_brush.setStart(m_joystickPos.x(), m_joystickPos.y() - m_pushRadius);
    push_btn_brush.setFinalStop(m_joystickPos.x(), m_joystickPos.y() + m_pushRadius);
    push_btn_brush.setCoordinateMode(QGradient::CoordinateMode::LogicalMode);
    painter.setPen(Qt::NoPen);
    painter.setBrush(push_btn_brush);
    painter.drawEllipse(m_joystickPos, m_pushRadius, m_pushRadius);
    painter.restore();

    //draw legend
    painter.save();
    painter.setFont(font);
    QPen pen;
    pen.setColor(Qt::darkGray);
    pen.setWidth(2);
    painter.setPen(Qt::darkGray);
    QFontMetrics pfm2 = painter.fontMetrics();
    int rect_tl_x = rect().bottomLeft().x() + 2;
    int rect_tl_y = rect().bottom() - pfm2.height() * 2 * 1.2;
    painter.drawRect(rect_tl_x, rect_tl_y, pfm2.width("前进:xxx") * 1.2, pfm2.height() * 2 * 1.2);

    painter.setPen(Qt::black);
    QString text;
    if(m_joystickValueY >=0)
        text = "前进:";
    else
        text = "后退:";
    text += QString("%1").arg((int)(abs(m_joystickValueY * 100)), 3, 10, QChar('0')); 
    painter.drawText(QPoint(rect_tl_x + pfm2.width("前进:xxx") * 0.1, rect_tl_y + pfm2.height() * 1.1), text);
    if(m_joystickValueX >=0)
        text = "左移:";
    else
        text = "右移:";
    text += QString("%1").arg((int)(abs(m_joystickValueX * 100)), 3, 10, QChar('0')); 
    painter.drawText(QPoint(rect_tl_x + pfm2.width("前进:xxx") * 0.1, rect_tl_y + pfm2.height() * 2.1), text);

    painter.restore();

}

void JoystickWidget::mouseMoveEvent(QMouseEvent *e){

    if(m_isPressedInside){
        
        int joy_value_x = e -> x() - m_mousePosWhenPressed.x(),
            joy_value_y = e -> y() - m_mousePosWhenPressed.y();
        
        if(joy_value_x * joy_value_x + joy_value_y * joy_value_y >= m_padRadius * m_padRadius){
            double factor = sqrt(m_padRadius * m_padRadius * 1.0 / (joy_value_x * joy_value_x + joy_value_y * joy_value_y));
            joy_value_x *= factor;
            joy_value_y *= factor;
        }

        m_joystickPos.setX(rect().center().x() + joy_value_x);
        m_joystickPos.setY(rect().center().y() + joy_value_y);
        m_joystickValueX = - joy_value_x * 1.0 / m_padRadius;
        m_joystickValueY = - joy_value_y * 1.0 / m_padRadius;
        Q_EMIT JoystickValueChanged(m_joystickValueX, m_joystickValueY);
        update();
    }

}

void JoystickWidget::mousePressEvent(QMouseEvent *e){
    
    m_mousePosWhenPressed.setX(e -> x() + rect().center().x() - m_joystickPos.x());
    m_mousePosWhenPressed.setY(e -> y() + rect().center().y() - m_joystickPos.y());
    int dx = e -> x() - m_joystickPos.x(), dy = e -> y() - m_joystickPos.y();
    if(dx * dx + dy * dy <= m_pushRadius * m_pushRadius){
        m_isPressedInside = true;
        update();
    }
    
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *e){

    if(m_isPressedInside){
        Q_EMIT JoystickValueChanged(0.0, 0.0);
        m_isPressedInside = false;
        m_joystickPos.setX(rect().center().x());
        m_joystickPos.setY(rect().center().y());
        m_joystickValueX = 0.0;
        m_joystickValueY = 0.0;
        update();
    }

}

void JoystickWidget::setJoystickPos(float x, float y){

    if(m_isPressedInside)
        return;

    if(x * x + y * y > 1){
        double factor = sqrt(1.0 / (x * x + y * y));
        x *= factor;
        y *= factor;
    }

    m_joystickPos.setX(rect().center().x() - x * m_padRadius);
    m_joystickPos.setY(rect().center().y() - y * m_padRadius);
    m_joystickValueX = x;
    m_joystickValueY = y;
    update();
}