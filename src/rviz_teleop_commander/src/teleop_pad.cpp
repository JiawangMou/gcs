#include <stdio.h>
#include <sstream>
#include <queue>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>

#include <geometry_msgs/Twist.h>
#include "rviz_teleop_commander/PID.h"
#include <QDebug>

#include "rviz_teleop_commander/teleop_pad.h"
#include <visualization_msgs/Marker.h>

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
FMAVStatusPanel::FMAVStatusPanel( QWidget* parent )
  : rviz::Panel( parent )
{
    //初始化变量
#ifdef TWO_WING
    mid_servo_pwm_set_ = 0;
    climb_pwm_set_ = 0;
#endif
    left_servo_pwm_set_ = 0;
    right_servo_pwm_set_ = 0;
    throttle_pwm_set_ = 0;
#ifdef FOUR_WING
    throttle_2_pwm_set_ = 0;
#endif

    throttle_joystick_ = 0;
    pitch_joystick_ = roll_joystick_ = yaw_joystick_ = 0;

    uint i, j;
    for(i = 0; i < 3; i ++){
        for(j = 0; j < 3; j ++){
            pid_ext_set_[i][j] = 0.0;
            pid_int_set_[i][j] = 0.0;
        }
        pid_ext_uplimit_[i] = 0.0;
        pid_ext_lowlimit_[i] = 0.0;
        pid_int_uplimit_[i] = 0;
        pid_int_lowlimit_[i] = 0;
        pid_setvalue_[i] = 0.0;
    }
    pid_freq_ = 0;
    pid_id_set_ = 0;
    is_throttle_enabled_ = false;
#ifdef FOUR_WING
    is_throttle_2_enabled_ = 0;
#endif
    is_vicon_started = false;
    is_displaying_msg = false;
    is_calibrating_mag = false;
    mag_calib_data_[0].resize(MAG_CALIB_REQUIRE_CNT);
    mag_calib_data_[1].resize(MAG_CALIB_REQUIRE_CNT);
    mag_calib_data_[2].resize(MAG_CALIB_REQUIRE_CNT);

    // 标志logo
    std::string logo_img_path;
    nh_.param<std::string>("/logo_path", logo_img_path, "");
    logo_img_ = new QLabel();
    logo_img_ -> setMaximumWidth(300);
    logo_img_ -> setMaximumHeight(100);
    logo_img_ -> setScaledContents(true);
    logo_img_ -> setAlignment(Qt::AlignCenter);
    logo_img_ -> setPixmap(QPixmap(logo_img_path.c_str()));

    // 建立一个水平layout(显示飞行模式)
    QHBoxLayout* mode_layout = new QHBoxLayout;
    mode_front_label_ = new QLabel("状态");
    mode_front_label_ -> setAlignment(Qt::AlignCenter);
    mode_front_label_ -> setFont(QFont("Timers", 18, QFont::Normal));
    mode_front_label_ -> setMaximumWidth(100);
    mode_layout -> addWidget(mode_front_label_);
    mode_label_ = new QLabel("尚未连接");
    mode_label_ -> setAlignment(Qt::AlignCenter);
    mode_label_ -> setFrameShape(QFrame::Panel);
    mode_label_ -> setFrameShadow(QFrame::Sunken);
    mode_label_ -> setFont(QFont("Timers", 18, QFont::Normal));
    mode_label_ -> setAutoFillBackground(true);
    mode_layout -> addWidget(mode_label_);

    // 时间
    time_layout_ = new QHBoxLayout();
    time_s_label_ = new QLabel("");
    time_s_label_ -> setAlignment(Qt::AlignRight);
    time_ms_label_ = new QLabel("");
    time_ms_label_ -> setAlignment(Qt::AlignLeft);
    time_s_label_ -> setFont(QFont("Timers", 18, QFont::Normal));
    time_ms_label_ -> setFont(QFont("Timers", 12, QFont::Normal));
    time_layout_ -> addWidget(time_s_label_);
    time_layout_ -> addWidget(time_ms_label_);

    // 多个传感器layout
    QHBoxLayout* roll_layout = new QHBoxLayout;
    roll_front_label_ = new QLabel("Roll: ");
    roll_front_label_ -> setAlignment(Qt::AlignCenter);
    roll_layout -> addWidget(roll_front_label_);
    roll_label_ = new QLabel("-");
    roll_label_ -> setAlignment(Qt::AlignCenter);
    roll_label_ -> setFrameShape(QFrame::Panel);
    roll_label_ -> setFrameShadow(QFrame::Sunken);
    roll_layout -> addWidget(roll_label_);
    roll_rate_front_label_ = new QLabel("Roll Rate: ");
    roll_rate_front_label_ -> setAlignment(Qt::AlignCenter);
    roll_layout -> addWidget(roll_rate_front_label_);
    roll_rate_label_ = new QLabel("-");
    roll_rate_label_ -> setAlignment(Qt::AlignCenter);
    roll_rate_label_ -> setFrameShape(QFrame::Panel);
    roll_rate_label_ -> setFrameShadow(QFrame::Sunken);
    roll_layout -> addWidget(roll_rate_label_);

    QHBoxLayout* pitch_layout = new QHBoxLayout;
    pitch_front_label_ = new QLabel("Pitch: ");
    pitch_front_label_ -> setAlignment(Qt::AlignCenter);
    pitch_layout -> addWidget(pitch_front_label_);
    pitch_label_ = new QLabel("-");
    pitch_label_ -> setAlignment(Qt::AlignCenter);
    pitch_label_ -> setFrameShape(QFrame::Panel);
    pitch_label_ -> setFrameShadow(QFrame::Sunken);
    pitch_layout -> addWidget(pitch_label_);
    pitch_rate_front_label_ = new QLabel("Pitch Rate: ");
    pitch_rate_front_label_ -> setAlignment(Qt::AlignCenter);
    pitch_layout -> addWidget(pitch_rate_front_label_);
    pitch_rate_label_ = new QLabel("-");
    pitch_rate_label_ -> setAlignment(Qt::AlignCenter);
    pitch_rate_label_ -> setFrameShape(QFrame::Panel);
    pitch_rate_label_ -> setFrameShadow(QFrame::Sunken);
    pitch_layout -> addWidget(pitch_rate_label_);

    QHBoxLayout* yaw_layout = new QHBoxLayout;
    yaw_front_label_ = new QLabel("Yaw: ");
    yaw_front_label_ -> setAlignment(Qt::AlignCenter);
    yaw_layout -> addWidget(yaw_front_label_);
    yaw_label_ = new QLabel("-");
    yaw_label_ -> setAlignment(Qt::AlignCenter);
    yaw_label_ -> setFrameShape(QFrame::Panel);
    yaw_label_ -> setFrameShadow(QFrame::Sunken);
    yaw_layout -> addWidget(yaw_label_);
    yaw_rate_front_label_ = new QLabel("Yaw Rate: ");
    yaw_rate_front_label_ -> setAlignment(Qt::AlignCenter);
    yaw_layout -> addWidget(yaw_rate_front_label_);
    yaw_rate_label_ = new QLabel("-");
    yaw_rate_label_ -> setAlignment(Qt::AlignCenter);
    yaw_rate_label_ -> setFrameShape(QFrame::Panel);
    yaw_rate_label_ -> setFrameShadow(QFrame::Sunken);
    yaw_layout -> addWidget(yaw_rate_label_);

    QVBoxLayout* servos_layout = new QVBoxLayout;
#ifdef TWO_WING
    mid_servo_front_label_ = new QLabel("中舵机PWM: ");
    mid_servo_front_label_ -> setAlignment(Qt::AlignCenter);
    servos_layout -> addWidget(mid_servo_front_label_);
    mid_servo_label_ = new QLabel("-");
    mid_servo_label_ -> setAlignment(Qt::AlignCenter);
    mid_servo_label_ -> setFrameShape(QFrame::Panel);
    mid_servo_label_ -> setFrameShadow(QFrame::Sunken);
    servos_layout -> addWidget(mid_servo_label_);
#endif
    left_servo_front_label_ = new QLabel("左舵机PWM: ");
    left_servo_front_label_ -> setAlignment(Qt::AlignCenter);
    servos_layout -> addWidget(left_servo_front_label_);
    left_servo_label_ = new QLabel("-");
    left_servo_label_ -> setAlignment(Qt::AlignCenter);
    left_servo_label_ -> setFrameShape(QFrame::Panel);
    left_servo_label_ -> setFrameShadow(QFrame::Sunken);
    servos_layout -> addWidget(left_servo_label_);
    right_servo_front_label_ = new QLabel("右舵机PWM: ");
    right_servo_front_label_ -> setAlignment(Qt::AlignCenter);
    servos_layout -> addWidget(right_servo_front_label_);
    right_servo_label_ = new QLabel("-");
    right_servo_label_ -> setAlignment(Qt::AlignCenter);
    right_servo_label_ -> setFrameShape(QFrame::Panel);
    right_servo_label_ -> setFrameShadow(QFrame::Sunken);
    servos_layout -> addWidget(right_servo_label_);

    QVBoxLayout* otherpwm_layout = new QVBoxLayout;
    throttle_front_label_ = new QLabel("扑翼油门PWM: ");
    throttle_front_label_ -> setAlignment(Qt::AlignCenter);
    otherpwm_layout -> addWidget(throttle_front_label_);
    throttle_label_ = new QLabel("-");
    throttle_label_ -> setAlignment(Qt::AlignCenter);
    throttle_label_ -> setFrameShape(QFrame::Panel);
    throttle_label_ -> setFrameShadow(QFrame::Sunken);
    otherpwm_layout -> addWidget(throttle_label_);
#ifdef FOUR_WING
    throttle_2_front_label_ = new QLabel("扑翼2油门PWM: ");
    throttle_2_front_label_ -> setAlignment(Qt::AlignCenter);
    otherpwm_layout -> addWidget(throttle_2_front_label_);
    throttle_2_label_ = new QLabel("-");
    throttle_2_label_ -> setAlignment(Qt::AlignCenter);
    throttle_2_label_ -> setFrameShape(QFrame::Panel);
    throttle_2_label_ -> setFrameShadow(QFrame::Sunken);
    otherpwm_layout -> addWidget(throttle_2_label_);
#endif
#ifdef TWO_WING
    climb_front_label_ = new QLabel("爬行机构PWM: ");
    climb_front_label_ -> setAlignment(Qt::AlignCenter);
    otherpwm_layout -> addWidget(climb_front_label_);
    climb_label_ = new QLabel("-");
    climb_label_ -> setAlignment(Qt::AlignCenter);
    climb_label_ -> setFrameShape(QFrame::Panel);
    climb_label_ -> setFrameShadow(QFrame::Sunken);
    otherpwm_layout -> addWidget(climb_label_);
#endif

    pid_id_front_label_ = new QLabel("当前调参PID: ");
    pid_id_front_label_ -> setAlignment(Qt::AlignCenter);
    otherpwm_layout -> addWidget(pid_id_front_label_);
    pid_id_label_ = new QLabel("-");
    pid_id_label_ -> setAlignment(Qt::AlignCenter);
    pid_id_label_ -> setFrameShape(QFrame::Panel);
    pid_id_label_ -> setFrameShadow(QFrame::Sunken);
    otherpwm_layout -> addWidget(pid_id_label_);


    QHBoxLayout* pwm_layout = new QHBoxLayout;
    pwm_layout -> addLayout(servos_layout);
    pwm_layout -> addLayout(otherpwm_layout);
    

    //参数界面layouts
    QHBoxLayout* para_menu = new QHBoxLayout;
    para_front_label_ = new QLabel("参数设置");
    para_front_label_ -> setAlignment(Qt::AlignCenter);
    para_menu -> addWidget(para_front_label_);
    upload_to_mav_ = new QPushButton("上传参数");
    upload_to_mav_ -> setEnabled(false);
    para_menu -> addWidget(upload_to_mav_);
    download_from_mav_ = new QPushButton("下载参数");
    download_from_mav_ -> setEnabled(false);
    para_menu -> addWidget(download_from_mav_);
    write_flash_front_label_ = new QLabel("写flash");
    write_flash_front_label_ -> setAlignment(Qt::AlignRight);
    para_menu -> addWidget(write_flash_front_label_);
    write_flash_checkbox_ = new QCheckBox();
    write_flash_checkbox_ -> setChecked(false);
    para_menu -> setStretchFactor(para_front_label_, 6);
    para_menu -> setStretchFactor(upload_to_mav_, 6);
    para_menu -> setStretchFactor(write_flash_front_label_, 4);
    para_menu -> setStretchFactor(write_flash_checkbox_, 1);
    para_menu -> addWidget(write_flash_checkbox_);

    mode_sel_combo_ = new QComboBox();
    mode_sel_combo_ -> addItem("-", default_mode);
    mode_sel_combo_ -> addItem("舵机/电机调试模式", servo_debug_mode);
    mode_sel_combo_ -> addItem("传感器校准模式", sensor_alignment_mode);
    mode_sel_combo_ -> addItem("飞行模式", flight_mode);
    mode_sel_combo_ -> addItem("调参模式", tuning_mode);
    mode_sel_combo_ -> addItem("Vicon测试模式", vicon_test_mode);
    mode_sel_combo_ -> setCurrentIndex(default_mode);

    //PWM调节栏(开始模式，手动模式)
#ifdef TWO_WING
    mid_servo_set_layout_ = new QHBoxLayout;
    mid_servo_set_front_label_ = new QLabel("中舵机PWM: ");
    mid_servo_set_front_label_ -> setAlignment(Qt::AlignCenter);
    mid_servo_set_layout_ -> addWidget(mid_servo_set_front_label_);
    mid_servo_set_slider_ = new QSlider(Qt::Horizontal);
    mid_servo_set_slider_ -> setRange(servo_pwm_min, servo_pwm_max);
    mid_servo_set_layout_ -> addWidget(mid_servo_set_slider_);
    mid_servo_set_spin_ = new QSpinBox();
    mid_servo_set_spin_ -> setRange(servo_pwm_min, servo_pwm_max);
    mid_servo_set_layout_ -> addWidget(mid_servo_set_spin_);
    mid_servo_set_layout_ -> setStretchFactor(mid_servo_set_slider_, 4);
    mid_servo_set_layout_ -> setStretchFactor(mid_servo_set_front_label_, 2);
    mid_servo_set_layout_ -> setStretchFactor(mid_servo_set_spin_, 1);
    connect(mid_servo_set_spin_, SIGNAL(valueChanged(int)), mid_servo_set_slider_, SLOT(setValue(int)));
    connect(mid_servo_set_slider_, SIGNAL(valueChanged(int)), mid_servo_set_spin_, SLOT(setValue(int)));
#endif

    left_servo_set_layout_ = new QHBoxLayout;
    left_servo_set_front_label_ = new QLabel("左舵机PWM: ");
    left_servo_set_front_label_ -> setAlignment(Qt::AlignCenter);
    left_servo_set_layout_ -> addWidget(left_servo_set_front_label_);
    left_servo_set_slider_ = new QSlider(Qt::Horizontal);
    left_servo_set_slider_ -> setRange(servo_pwm_min, servo_pwm_max);
    left_servo_set_layout_ -> addWidget(left_servo_set_slider_);
    left_servo_set_spin_ = new QSpinBox();
    left_servo_set_spin_ -> setRange(servo_pwm_min, servo_pwm_max);
    left_servo_set_layout_ -> addWidget(left_servo_set_spin_);
    left_servo_set_layout_ -> setStretchFactor(left_servo_set_slider_, 4);
    left_servo_set_layout_ -> setStretchFactor(left_servo_set_front_label_, 2);
    left_servo_set_layout_ -> setStretchFactor(left_servo_set_spin_, 1);
    connect(left_servo_set_spin_, SIGNAL(valueChanged(int)), left_servo_set_slider_, SLOT(setValue(int)));
    connect(left_servo_set_slider_, SIGNAL(valueChanged(int)), left_servo_set_spin_, SLOT(setValue(int)));

    right_servo_set_layout_ = new QHBoxLayout;
    right_servo_set_front_label_ = new QLabel("右舵机PWM: ");
    right_servo_set_front_label_ -> setAlignment(Qt::AlignCenter);
    right_servo_set_layout_ -> addWidget(right_servo_set_front_label_);
    right_servo_set_slider_ = new QSlider(Qt::Horizontal);
    right_servo_set_slider_ -> setRange(servo_pwm_min, servo_pwm_max);
    right_servo_set_layout_ -> addWidget(right_servo_set_slider_);
    right_servo_set_spin_ = new QSpinBox();
    right_servo_set_spin_ -> setRange(servo_pwm_min, servo_pwm_max);
    right_servo_set_layout_ -> addWidget(right_servo_set_spin_);
    right_servo_set_layout_ -> setStretchFactor(right_servo_set_slider_, 4);
    right_servo_set_layout_ -> setStretchFactor(right_servo_set_front_label_, 2);
    right_servo_set_layout_ -> setStretchFactor(right_servo_set_spin_, 1);
    connect(right_servo_set_spin_, SIGNAL(valueChanged(int)), right_servo_set_slider_, SLOT(setValue(int)));
    connect(right_servo_set_slider_, SIGNAL(valueChanged(int)), right_servo_set_spin_, SLOT(setValue(int)));

    throttle_set_layout_ = new QHBoxLayout;
    throttle_enable_ = new QPushButton("启动");
    throttle_set_layout_ -> addWidget(throttle_enable_);
    throttle_set_front_label_ = new QLabel("扑翼油门PWM: ");
    throttle_set_front_label_ -> setAlignment(Qt::AlignCenter);
    throttle_set_layout_ -> addWidget(throttle_set_front_label_);
    throttle_set_slider_ = new QSlider(Qt::Horizontal);
    throttle_set_slider_ -> setRange(0, THROTTLE_MAX);
    throttle_set_layout_ -> addWidget(throttle_set_slider_);
    throttle_set_spin_ = new QSpinBox();
    throttle_set_spin_ -> setRange(0, THROTTLE_MAX);
    throttle_set_layout_ -> addWidget(throttle_set_spin_);
    throttle_set_layout_ -> setStretchFactor(throttle_set_slider_, 4);
    throttle_set_layout_ -> setStretchFactor(throttle_set_front_label_, 2);
    throttle_set_layout_ -> setStretchFactor(throttle_set_spin_, 1);
    connect(throttle_set_spin_, SIGNAL(valueChanged(int)), throttle_set_slider_, SLOT(setValue(int)));
    connect(throttle_set_slider_, SIGNAL(valueChanged(int)), throttle_set_spin_, SLOT(setValue(int)));

#ifdef FOUR_WING
    throttle_2_set_layout_ = new QHBoxLayout;
    throttle_2_enable_ = new QPushButton("启动");
    throttle_2_set_layout_ -> addWidget(throttle_2_enable_);
    throttle_2_set_front_label_ = new QLabel("扑翼2油门PWM: ");
    throttle_2_set_front_label_ -> setAlignment(Qt::AlignCenter);
    throttle_2_set_layout_ -> addWidget(throttle_2_set_front_label_);
    throttle_2_set_slider_ = new QSlider(Qt::Horizontal);
    throttle_2_set_slider_ -> setRange(0, THROTTLE_MAX);
    throttle_2_set_layout_ -> addWidget(throttle_2_set_slider_);
    throttle_2_set_spin_ = new QSpinBox();
    throttle_2_set_spin_ -> setRange(0, THROTTLE_MAX);
    throttle_2_set_layout_ -> addWidget(throttle_2_set_spin_);
    throttle_2_set_layout_ -> setStretchFactor(throttle_2_set_slider_, 4);
    throttle_2_set_layout_ -> setStretchFactor(throttle_2_set_front_label_, 2);
    throttle_2_set_layout_ -> setStretchFactor(throttle_2_set_spin_, 1);
    connect(throttle_2_set_spin_, SIGNAL(valueChanged(int)), throttle_2_set_slider_, SLOT(setValue(int)));
    connect(throttle_2_set_slider_, SIGNAL(valueChanged(int)), throttle_2_set_spin_, SLOT(setValue(int)));
#endif
#ifdef TWO_WING
    climb_set_layout_ = new QHBoxLayout;
    climb_set_front_label_ = new QLabel("爬行机构PWM: ");
    climb_set_front_label_ -> setAlignment(Qt::AlignCenter);
    climb_set_layout_ -> addWidget(climb_set_front_label_);
    climb_set_slider_ = new QSlider(Qt::Horizontal);
    climb_set_slider_ -> setRange(0, 100);
    climb_set_layout_ -> addWidget(climb_set_slider_);
    climb_set_spin_ = new QSpinBox();
    climb_set_spin_ -> setRange(0, 100);
    climb_set_layout_ -> addWidget(climb_set_spin_);
    climb_set_layout_ -> setStretchFactor(climb_set_slider_, 4);
    climb_set_layout_ -> setStretchFactor(climb_set_front_label_, 2);
    climb_set_layout_ -> setStretchFactor(climb_set_spin_, 1);
    connect(climb_set_spin_, SIGNAL(valueChanged(int)), climb_set_slider_, SLOT(setValue(int)));
    connect(climb_set_slider_, SIGNAL(valueChanged(int)), climb_set_spin_, SLOT(setValue(int)));
#endif

    //Sensor Alignment Controls
    sensor_calib_layout_ = new QVBoxLayout();

    sensor_calib_btn_layout_ = new QHBoxLayout();
    mag_calib_start_btn_ = new QPushButton("校准磁力计");
    mag_calib_clear_btn_ = new QPushButton("清除预览");
    sensor_calib_btn_layout_ -> addWidget(mag_calib_start_btn_);
    sensor_calib_btn_layout_ -> addWidget(mag_calib_clear_btn_);
    sensor_calib_layout_ -> addLayout(sensor_calib_btn_layout_);

    mag_offset_front_label_ = new QLabel("磁力计偏移量");
    mag_offset_front_label_ -> setAlignment(Qt::AlignCenter);
    sensor_calib_layout_ -> addWidget(mag_offset_front_label_);
    mag_offset_layout_ = new QHBoxLayout();
    mag_offset_x_front_label_ = new QLabel("X:");
    mag_offset_x_edit_ = new QLineEdit();
    mag_offset_y_front_label_ = new QLabel("Y:");
    mag_offset_y_edit_ = new QLineEdit();
    mag_offset_z_front_label_ = new QLabel("Z:");
    mag_offset_z_edit_ = new QLineEdit();
    mag_offset_layout_ -> addWidget(mag_offset_x_front_label_);
    mag_offset_layout_ -> addWidget(mag_offset_x_edit_);
    mag_offset_layout_ -> addWidget(mag_offset_y_front_label_);
    mag_offset_layout_ -> addWidget(mag_offset_y_edit_);
    mag_offset_layout_ -> addWidget(mag_offset_z_front_label_);
    mag_offset_layout_ -> addWidget(mag_offset_z_edit_);
    sensor_calib_layout_ -> addLayout(mag_offset_layout_);

    mag_gain_front_label_ = new QLabel("磁力计矫正半径");
    mag_gain_front_label_ -> setAlignment(Qt::AlignCenter);
    sensor_calib_layout_ -> addWidget(mag_gain_front_label_);
    mag_gain_layout_ = new QHBoxLayout();
    mag_gain_x_front_label_ = new QLabel("X:");
    mag_gain_x_edit_ = new QLineEdit();
    mag_gain_y_front_label_ = new QLabel("Y:");
    mag_gain_y_edit_ = new QLineEdit();
    mag_gain_z_front_label_ = new QLabel("Z:");
    mag_gain_z_edit_ = new QLineEdit();
    mag_gain_layout_ -> addWidget(mag_gain_x_front_label_);
    mag_gain_layout_ -> addWidget(mag_gain_x_edit_);
    mag_gain_layout_ -> addWidget(mag_gain_y_front_label_);
    mag_gain_layout_ -> addWidget(mag_gain_y_edit_);
    mag_gain_layout_ -> addWidget(mag_gain_z_front_label_);
    mag_gain_layout_ -> addWidget(mag_gain_z_edit_);
    sensor_calib_layout_ -> addLayout(mag_gain_layout_);


    //PID Tuning Controls
    pid_tuning_layout_ = new QVBoxLayout();
    
    pid_front_layout_ = new QHBoxLayout();
    pid_setvalue_front_label_ = new QLabel("设定角度目标值 - Yaw :");
    pid_setvalue_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_front_layout_ -> addWidget(pid_setvalue_front_label_);
    pid_setvalue_edit_ = new QLineEdit();
    pid_front_layout_ -> addWidget(pid_setvalue_edit_);
    pid_freq_front_label_ = new QLabel("PID更新频率:");
    pid_freq_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_front_layout_ -> addWidget(pid_freq_front_label_);
    pid_freq_edit_ = new QLineEdit();
    pid_front_layout_ -> addWidget(pid_freq_edit_);
    pid_tuning_layout_ -> addLayout(pid_front_layout_);
    

    pid_ext_front_label_ = new QLabel("外环PID参数 - Yaw");
    pid_ext_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_tuning_layout_ -> addWidget(pid_ext_front_label_);

    pid_ext_layout_ = new QHBoxLayout();
    pid_ext_p_front_label_ = new QLabel("P:");
    pid_ext_p_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_ext_layout_ -> addWidget(pid_ext_p_front_label_);
    pid_ext_p_edit_ = new QLineEdit();
    pid_ext_layout_ -> addWidget(pid_ext_p_edit_);
    pid_ext_i_front_label_ = new QLabel("I:");
    pid_ext_i_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_ext_layout_ -> addWidget(pid_ext_i_front_label_);
    pid_ext_i_edit_ = new QLineEdit();
    pid_ext_layout_ -> addWidget(pid_ext_i_edit_);
    pid_ext_d_front_label_ = new QLabel("D:");
    pid_ext_d_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_ext_layout_ -> addWidget(pid_ext_d_front_label_);
    pid_ext_d_edit_ = new QLineEdit();
    pid_ext_layout_ -> addWidget(pid_ext_d_edit_);
    pid_tuning_layout_ -> addLayout(pid_ext_layout_);

    pid_ext_limit_layout_ = new QHBoxLayout();
    pid_ext_lowlimit_front_label_ = new QLabel("外环PID下限:");
    pid_ext_lowlimit_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_ext_limit_layout_ -> addWidget(pid_ext_lowlimit_front_label_);
    pid_ext_lowlimit_edit_ = new QLineEdit();
    pid_ext_limit_layout_ -> addWidget(pid_ext_lowlimit_edit_);
    pid_ext_uplimit_front_label_ = new QLabel("外环PID上限:");
    pid_ext_uplimit_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_ext_limit_layout_ -> addWidget(pid_ext_uplimit_front_label_);
    pid_ext_uplimit_edit_ = new QLineEdit();
    pid_ext_limit_layout_ -> addWidget(pid_ext_uplimit_edit_);
    pid_tuning_layout_ -> addLayout(pid_ext_limit_layout_);

    pid_int_front_label_ = new QLabel("内环PID参数 - Yaw");
    pid_int_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_tuning_layout_ -> addWidget(pid_int_front_label_);

    pid_int_layout_ = new QHBoxLayout();
    pid_int_p_front_label_ = new QLabel("P:");
    pid_int_p_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_int_layout_ -> addWidget(pid_int_p_front_label_);
    pid_int_p_edit_ = new QLineEdit();
    pid_int_layout_ -> addWidget(pid_int_p_edit_);
    pid_int_i_front_label_ = new QLabel("I:");
    pid_int_i_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_int_layout_ -> addWidget(pid_int_i_front_label_);
    pid_int_i_edit_ = new QLineEdit();
    pid_int_layout_ -> addWidget(pid_int_i_edit_);
    pid_int_d_front_label_ = new QLabel("D:");
    pid_int_d_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_int_layout_ -> addWidget(pid_int_d_front_label_);
    pid_int_d_edit_ = new QLineEdit();
    pid_int_layout_ -> addWidget(pid_int_d_edit_);
    pid_tuning_layout_ -> addLayout(pid_int_layout_);
    
    pid_int_limit_layout_ = new QHBoxLayout();
    pid_int_lowlimit_front_label_ = new QLabel("内环PID下限:");
    pid_int_lowlimit_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_int_limit_layout_ -> addWidget(pid_int_lowlimit_front_label_);
    pid_int_lowlimit_edit_ = new QLineEdit();
    pid_int_limit_layout_ -> addWidget(pid_int_lowlimit_edit_);
    pid_int_uplimit_front_label_ = new QLabel("内环PID上限:");
    pid_int_uplimit_front_label_ -> setAlignment(Qt::AlignCenter);
    pid_int_limit_layout_ -> addWidget(pid_int_uplimit_front_label_);
    pid_int_uplimit_edit_ = new QLineEdit();
    pid_int_limit_layout_ -> addWidget(pid_int_uplimit_edit_);
    pid_tuning_layout_ -> addLayout(pid_int_limit_layout_);


    pid_id_layout_ = new QHBoxLayout();
    pid_id_set_front_label_  = new QLabel("当前目标PID轴:");
    pid_id_layout_ -> addWidget(pid_id_set_front_label_);
    pid_id_pitch_ = new QPushButton("Pitch");
    pid_id_roll_ = new QPushButton("Roll");
    pid_id_yaw_ = new QPushButton("Yaw");
    pid_id_pitch_ -> setCheckable(true);
    pid_id_roll_ -> setCheckable(true);
    pid_id_yaw_ -> setCheckable(true);
    pid_id_btn_group_ = new QButtonGroup();
    pid_id_btn_group_ -> addButton(pid_id_yaw_);
    pid_id_btn_group_ -> addButton(pid_id_pitch_);
    pid_id_btn_group_ -> addButton(pid_id_roll_);
    pid_id_btn_group_ -> setId(pid_id_yaw_, 0);
    pid_id_btn_group_ -> setId(pid_id_pitch_, 1);
    pid_id_btn_group_ -> setId(pid_id_roll_, 2);
    pid_id_layout_ -> addWidget(pid_id_yaw_);
    pid_id_layout_ -> addWidget(pid_id_pitch_);
    pid_id_layout_ -> addWidget(pid_id_roll_);

    pid_tuning_layout_ -> addLayout(pid_id_layout_);

    //Vicon Test Layout
    vicon_test_layout_ = new QVBoxLayout();
    vicon_topic_refresh_btn_ = new QPushButton("刷新Vicon话题");
    vicon_test_layout_ -> addWidget(vicon_topic_refresh_btn_);
    vicon_topic_combo_ = new QComboBox();
    vicon_test_layout_ -> addWidget(vicon_topic_combo_);
    vicon_start_btn_ = new QPushButton("开始Vicon转发");
    vicon_test_layout_ -> addWidget(vicon_start_btn_);
    
    //飞行模式摇杆
    flight_control_joysitck_ = new JoystickWidget();
    flight_control_joysitck_ -> setFixedHeight(200);


    QFrame* spline_1 = new QFrame();
    spline_1->setFrameShape(QFrame::HLine);
    spline_1->setFrameShadow(QFrame::Sunken);
    spline_1 -> setMinimumHeight(10);
    spline_1 -> setLineWidth(2);
    QFrame* spline_2 = new QFrame();
    spline_2->setFrameShape(QFrame::HLine);
    spline_2->setFrameShadow(QFrame::Sunken);
    spline_2 -> setMinimumHeight(10);
    spline_2 -> setLineWidth(2);
    QFrame* spline_3 = new QFrame();
    spline_3->setFrameShape(QFrame::HLine);
    spline_3->setFrameShadow(QFrame::Sunken);
    spline_3 -> setMinimumHeight(50);
    spline_3 -> setLineWidth(2);
    QFrame* spline_4 = new QFrame();
    spline_4->setFrameShape(QFrame::HLine);
    spline_4->setFrameShadow(QFrame::Sunken);
    spline_4 -> setMinimumHeight(10);
    spline_4 -> setLineWidth(2);

    QVBoxLayout* layout = new QVBoxLayout;
    layout -> setContentsMargins(20,20,20,20);
    layout -> addWidget(logo_img_);
    layout -> addLayout(mode_layout);
    layout -> addLayout(time_layout_);
    layout -> addWidget(spline_1);
    layout -> addLayout(roll_layout);
    layout -> addLayout(pitch_layout);
    layout -> addLayout(yaw_layout);
    layout -> addWidget(spline_2);
    layout -> addLayout(pwm_layout);
    layout -> addWidget(spline_3);
    layout -> addLayout(para_menu);
    layout -> addWidget(mode_sel_combo_);
    layout -> addWidget(spline_4);
#ifdef TWO_WING
    layout -> addLayout(mid_servo_set_layout_);
#endif
    layout -> addLayout(left_servo_set_layout_);
    layout -> addLayout(right_servo_set_layout_);
    layout -> addLayout(throttle_set_layout_);
#ifdef FOUR_WING
    layout -> addLayout(throttle_2_set_layout_);
#endif
#ifdef TWO_WING
    layout -> addLayout(climb_set_layout_);
#endif
    layout -> addWidget(flight_control_joysitck_);
    layout -> addLayout(pid_tuning_layout_);
    layout -> addLayout(vicon_test_layout_);
    layout -> addLayout(sensor_calib_layout_);
    layout -> addStretch();
    setLayout( layout );

    mav_down_sub_ = nh_.subscribe("/received_data", 10, &FMAVStatusPanel::updateMAVStatus, this);
    vis_pub_ = nh_.advertise< visualization_msgs::Marker>("/mav_vis", 10);
    mav_config_pub_ = nh_.advertise<mav_comm_driver::MFPUnified>("/mav_download", 10);
    mag_calib_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mag_calib_vis",10);

    //飞行模式手柄下发计时器
    joystick_send_timer_ = new QTimer( this );

    //检测断联计时器
    QTimer* connection_check_timer = new QTimer( this );
    mav_down_msg_cnt_ = mav_down_msg_cnt_prev_ = 0;
    is_connected = false;

    //消息显示延时计时器
    message_display_timer_ = new QTimer( this );
    connect( message_display_timer_, SIGNAL( timeout() ), this, SLOT( endMessage() ));

    connect( connection_check_timer, SIGNAL( timeout() ), this, SLOT( checkConnection() ));
    connect( joystick_send_timer_, SIGNAL( timeout() ), this, SLOT( uploadJoystick() ));
    connect( upload_to_mav_,  SIGNAL( clicked() ), this, SLOT( uploadConfig() ));
    connect( download_from_mav_,  SIGNAL( clicked() ), this, SLOT( downloadConfig() ));
    connect( mode_sel_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(setParamMode(int)));
    connect( pid_id_btn_group_, SIGNAL(buttonClicked(int)), this, SLOT(changeTuningAxis(int)));
    connect( throttle_enable_, SIGNAL(clicked()), this, SLOT(enableThrottle()));
#ifdef FOUR_WING
    connect( throttle_2_enable_, SIGNAL(clicked()), this, SLOT(enableThrottle2()));
#endif
    connect( vicon_topic_refresh_btn_,  SIGNAL( clicked() ), this, SLOT( refreshViconTopicList() ));
    connect( vicon_start_btn_,  SIGNAL( clicked() ), this, SLOT( viconStartEnd() ));
    connect( flight_control_joysitck_, SIGNAL(JoystickValueChanged(float,float)), this, SLOT(joystickMove(float, float)) );
    connect( mag_calib_start_btn_,  SIGNAL( clicked() ), this, SLOT( calibrateMag() ));
    connect( mag_calib_clear_btn_,  SIGNAL( clicked() ), this, SLOT( clearCalibrateVisualization() ));

    //T=500ms
    connection_check_timer -> start( 500 );

    param_mode_ = default_mode;
    setParamMode(default_mode);

    // system("rosrun joy joy_node&");

}

FMAVStatusPanel::~FMAVStatusPanel(){

    // system("rosnode kill /joy_node");
    //pclose(joystick_);
}

void FMAVStatusPanel::updateMAVStatus(const mav_comm_driver::MFPUnified::ConstPtr& msg){

    mav_down_msg_cnt_ ++;
    
    QPalette palette;
    char numstr[30];

    if(!is_connected){
        is_connected = true;
        displayStatus();
    }
    
    if(!upload_to_mav_ -> isEnabled())
        upload_to_mav_ -> setEnabled(true);
    
    if(!download_from_mav_ -> isEnabled())
        download_from_mav_ -> setEnabled(true);

    if(!throttle_enable_ -> isEnabled()){
        throttle_enable_ -> setEnabled(true);
#ifdef FOUR_WING
        throttle_2_enable_ -> setEnabled(true);
#endif
    }

    mode_id_ = msg -> msg_id;
    switch(msg -> msg_id){
        case(mav_comm_driver::MFPUnified::UP_STATUS):

            cur_roll_ = (int16_t)(msg -> data[2] << 8 | msg -> data[3]) / 100.0;
            cur_pitch_ = - (int16_t)(msg -> data[4] << 8 | msg -> data[5]) / 100.0;
            cur_yaw_ = - (int16_t)(msg -> data[6] << 8 | msg -> data[7]) / 100.0;

            // update display values
            if(!is_vicon_started){
                sprintf(numstr, "%.2f", cur_roll_);
                roll_label_ -> setText(numstr);
                sprintf(numstr, "%.2f", cur_pitch_);
                pitch_label_ -> setText(numstr);
                sprintf(numstr, "%.2f", cur_yaw_);
                yaw_label_ -> setText(numstr);

                //send tf transform
                tf::Transform transform;
                tf::Quaternion q;
                q.setRPY(cur_roll_ * DEG2RAD, cur_pitch_ * DEG2RAD, cur_yaw_ * DEG2RAD);
                transform.setRotation(q);
                transform.setOrigin(tf::Vector3(0,
                                                0,
                                                0));
                tf_pub_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            }
        break;

        case(mav_comm_driver::MFPUnified::UP_SENSER):
            cur_roll_rate_ = (int16_t)(msg -> data[8] << 8 | msg -> data[9]) / 10.0;
            cur_pitch_rate_ = (int16_t)(msg -> data[10] << 8 | msg -> data[11]) / 10.0;
            cur_yaw_rate_ = (int16_t)(msg -> data[12] << 8 | msg -> data[13]) / 10.0;
            cur_mag_raw_[0] = (int16_t)(msg -> data[14] << 8 | msg -> data[15]);
            cur_mag_raw_[1] = (int16_t)(msg -> data[16] << 8 | msg -> data[17]);
            cur_mag_raw_[2] = (int16_t)(msg -> data[18] << 8 | msg -> data[19]);

            // update display values
            sprintf(numstr, "%.2f", cur_roll_rate_);
            roll_rate_label_ -> setText(numstr);
            sprintf(numstr, "%.2f", cur_pitch_rate_);
            pitch_rate_label_ -> setText(numstr);
            sprintf(numstr, "%.2f", cur_yaw_rate_);
            yaw_rate_label_ -> setText(numstr);

            if(is_calibrating_mag){
                
                mag_calib_data_[0](mag_data_cnt_) = cur_mag_raw_[0];
                mag_calib_data_[1](mag_data_cnt_) = cur_mag_raw_[1];
                mag_calib_data_[2](mag_data_cnt_) = cur_mag_raw_[2];
                mag_data_cnt_ ++;

                geometry_msgs::Point mag_pt;
                mag_pt.x = cur_mag_raw_[0] / MAG_CALIB_POINT_SCALE;
                mag_pt.y = cur_mag_raw_[1] / MAG_CALIB_POINT_SCALE;
                mag_pt.z = cur_mag_raw_[2] / MAG_CALIB_POINT_SCALE;
                mag_calib_vis_pts_.points.push_back(mag_pt);
                mag_calib_vis_msg_.markers[0] = mag_calib_vis_pts_;
                mag_calib_vis_pub_.publish(mag_calib_vis_msg_);

                if(mag_data_cnt_ == MAG_CALIB_REQUIRE_CNT){
                    is_calibrating_mag = false;
                    Eigen::Vector3d ofs, gain;
                    if(Sensor::ellipsoidFit(mag_calib_data_[0], mag_calib_data_[1], mag_calib_data_[2],
                                         ofs, gain, 1)){
                        displayMessage("校准成功", Qt::green);

                        mag_offset_set_[0] = ofs(0);
                        mag_offset_set_[1] = ofs(1);
                        mag_offset_set_[2] = ofs(2);
                        mag_gain_set_[0] = gain(0);
                        mag_gain_set_[1] = gain(1);
                        mag_gain_set_[2] = gain(2);

                        mag_calib_vis_ellipsoid_.header.frame_id = "map";
                        mag_calib_vis_ellipsoid_.ns = "mag_pts";
                        mag_calib_vis_ellipsoid_.id = 105;
                        mag_calib_vis_ellipsoid_.type = visualization_msgs::Marker::SPHERE;
                        mag_calib_vis_ellipsoid_.color.r = 0.0;
                        mag_calib_vis_ellipsoid_.color.g = 0.0;
                        mag_calib_vis_ellipsoid_.color.b = 0.6;
                        mag_calib_vis_ellipsoid_.color.a = 0.7;
                        mag_calib_vis_ellipsoid_.scale.x = mag_gain_set_[0] * 2 / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_ellipsoid_.scale.y = mag_gain_set_[1] * 2 / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_ellipsoid_.scale.z = mag_gain_set_[2] * 2 / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_ellipsoid_.pose.position.x = mag_offset_set_[0] / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_ellipsoid_.pose.position.y = mag_offset_set_[1] / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_ellipsoid_.pose.position.z = mag_offset_set_[2] / MAG_CALIB_POINT_SCALE;
                        mag_calib_vis_msg_.markers.resize(2);
                        mag_calib_vis_msg_.markers[1] = mag_calib_vis_ellipsoid_;
                        mag_calib_vis_pub_.publish(mag_calib_vis_msg_);

                        setPanelValues();
                    }
                    else{
                        displayMessage("校准失败", Qt::red);
                    }
                    mag_calib_start_btn_ -> setText("校准磁力计");
                }
            }
        break;

        case(mav_comm_driver::MFPUnified::UP_PID1):
            //0: yaw, 1: pitch, 2:roll
            pid_int_set_[2][0] = (int16_t)(msg -> data[2] << 8 | msg -> data[3]) / 10.0;
            pid_int_set_[2][1] = (int16_t)(msg -> data[4] << 8 | msg -> data[5]) / 10.0;
            pid_int_set_[2][2] = (int16_t)(msg -> data[6] << 8 | msg -> data[7]) / 10.0;
            pid_int_set_[1][0] = (int16_t)(msg -> data[8] << 8 | msg -> data[9]) / 10.0;
            pid_int_set_[1][1] = (int16_t)(msg -> data[10] << 8 | msg -> data[11]) / 10.0;
            pid_int_set_[1][2] = (int16_t)(msg -> data[12] << 8 | msg -> data[13]) / 10.0;
            pid_int_set_[0][0] = (int16_t)(msg -> data[14] << 8 | msg -> data[15]) / 10.0;
            pid_int_set_[0][1] = (int16_t)(msg -> data[16] << 8 | msg -> data[17]) / 10.0;
            pid_int_set_[0][2] = (int16_t)(msg -> data[18] << 8 | msg -> data[19]) / 10.0;

            displayMessage("收到内环PID参数", Qt::yellow);
            // update display values
            setPanelValues();
        break;

        case(mav_comm_driver::MFPUnified::UP_PID2):
            //0: yaw, 1: pitch, 2:roll
            pid_ext_set_[2][0] = (int16_t)(msg -> data[2] << 8 | msg -> data[3]) / 10.0;
            pid_ext_set_[2][1] = (int16_t)(msg -> data[4] << 8 | msg -> data[5]) / 10.0;
            pid_ext_set_[2][2] = (int16_t)(msg -> data[6] << 8 | msg -> data[7]) / 10.0;
            pid_ext_set_[1][0] = (int16_t)(msg -> data[8] << 8 | msg -> data[9]) / 10.0;
            pid_ext_set_[1][1] = (int16_t)(msg -> data[10] << 8 | msg -> data[11]) / 10.0;
            pid_ext_set_[1][2] = (int16_t)(msg -> data[12] << 8 | msg -> data[13]) / 10.0;
            pid_ext_set_[0][0] = (int16_t)(msg -> data[14] << 8 | msg -> data[15]) / 10.0;
            pid_ext_set_[0][1] = (int16_t)(msg -> data[16] << 8 | msg -> data[17]) / 10.0;
            pid_ext_set_[0][2] = (int16_t)(msg -> data[18] << 8 | msg -> data[19]) / 10.0;

            displayMessage("收到外环PID参数", Qt::yellow);
            // update display values
            setPanelValues();
        break;

        case(mav_comm_driver::MFPUnified::UP_PID3):
            //0: x, 1: y, 2:z
            pid_vel_set_[2][0] = (int16_t)(msg -> data[2] << 8 | msg -> data[3]) / 10.0;
            pid_vel_set_[2][1] = (int16_t)(msg -> data[4] << 8 | msg -> data[5]) / 10.0;
            pid_vel_set_[2][2] = (int16_t)(msg -> data[6] << 8 | msg -> data[7]) / 10.0;
            pid_pos_set_[2][0] = (int16_t)(msg -> data[8] << 8 | msg -> data[9]) / 10.0;
            pid_pos_set_[2][1] = (int16_t)(msg -> data[10] << 8 | msg -> data[11]) / 10.0;
            pid_pos_set_[2][2] = (int16_t)(msg -> data[12] << 8 | msg -> data[13]) / 10.0;
            pid_vel_set_[0][0] = (int16_t)(msg -> data[14] << 8 | msg -> data[15]) / 10.0;
            pid_vel_set_[0][1] = (int16_t)(msg -> data[16] << 8 | msg -> data[17]) / 10.0;
            pid_vel_set_[0][2] = (int16_t)(msg -> data[18] << 8 | msg -> data[19]) / 10.0;

            ROS_INFO_STREAM("VZ: P:" << pid_vel_set_[2][0] << "I:" << pid_vel_set_[2][1] << "D:" << pid_vel_set_[2][2]);
            ROS_INFO_STREAM("VX: P:" << pid_vel_set_[0][0] << "I:" << pid_vel_set_[0][1] << "D:" << pid_vel_set_[0][2]);
            ROS_INFO_STREAM("PZ: P:" << pid_pos_set_[2][0] << "I:" << pid_pos_set_[2][1] << "D:" << pid_pos_set_[2][2]);
        break;

        case(mav_comm_driver::MFPUnified::UP_PID4):
            //0: x, 1: y, 2:z
            pid_pos_set_[0][0] = (int16_t)(msg -> data[2] << 8 | msg -> data[3]) / 10.0;
            pid_pos_set_[0][1] = (int16_t)(msg -> data[4] << 8 | msg -> data[5]) / 10.0;
            pid_pos_set_[0][2] = (int16_t)(msg -> data[6] << 8 | msg -> data[7]) / 10.0;
            
            ROS_INFO_STREAM("PX: P:" << pid_pos_set_[0][0] << "I:" << pid_pos_set_[0][1] << "D:" << pid_pos_set_[0][2]);
        break;

        case(mav_comm_driver::MFPUnified::UP_RCDATA):
            throttle_joystick_ = (uint)(msg -> data[2] << 8 | msg -> data[3]);
            yaw_joystick_ = (uint)(msg -> data[4] << 8 | msg -> data[5]);
            roll_joystick_ = (uint)(msg -> data[6] << 8 | msg -> data[7]);
            pitch_joystick_ = (uint)(msg -> data[2] << 8 | msg -> data[3]);

            flight_control_joysitck_ -> setJoystickPos(pitch_joystick_, roll_joystick_);
#ifdef TWO_WING
            throttle_pwm_set_ = throttle_joystick_ > 0.0 ? (int)(throttle_joystick_ * THROTTLE_MAX) : 0;
            throttle_set_spin_ -> setValue(throttle_pwm_set_);
#endif
#ifdef FOUR_WING
            throttle_pwm_set_ = throttle_joystick_ > 0.0 ? (int)(throttle_joystick_ * THROTTLE_MAX - 100) : 0;
            throttle_set_spin_ -> setValue(throttle_pwm_set_);
            throttle_2_pwm_set_ = throttle_joystick_ > 0.0 ? (int)(throttle_joystick_ * (THROTTLE_MAX - 100)) : 0;
            throttle_2_set_spin_ -> setValue(throttle_2_pwm_set_);
#endif
        

    }
//     mode_id_ = msg -> mode_id;

//     cur_roll_ = msg -> roll_angle;
//     cur_pitch_ = msg -> pitch_angle;
//     cur_yaw_ = msg -> yaw_angle;
//     cur_roll_rate_ = msg -> roll_rate;
//     cur_pitch_rate_ = msg -> pitch_rate;
//     cur_yaw_rate_ = msg -> yaw_rate;
//     cur_mid_servo_pwm_ = msg -> mid_servo_pwm;
//     cur_left_servo_pwm_ = msg -> left_servo_pwm;
//     cur_right_servo_pwm_ = msg -> right_servo_pwm;
//     cur_throttle_pwm_ = msg -> left_throttle_pwm;
//     cur_throttle_2_pwm_ = msg -> right_throttle_pwm;
//     cur_climb_pwm_ = msg -> climb_pwm;
//     cur_time_ms_ = msg -> board_time;
//     cur_pid_id_ = msg -> pid_id;

//     // update display values
//     char numstr[30];
//     sprintf(numstr, "%.2f", cur_roll_);
//     roll_label_ -> setText(numstr);
//     sprintf(numstr, "%.2f", cur_pitch_);
//     pitch_label_ -> setText(numstr);
//     sprintf(numstr, "%.2f", cur_yaw_);
//     yaw_label_ -> setText(numstr);
//     sprintf(numstr, "%.2f", cur_roll_rate_);
//     roll_rate_label_ -> setText(numstr);
//     sprintf(numstr, "%.2f", cur_pitch_rate_);
//     pitch_rate_label_ -> setText(numstr);
//     sprintf(numstr, "%.2f", cur_yaw_rate_);
//     yaw_rate_label_ -> setText(numstr);

// #ifdef TWO_WING
//     sprintf(numstr, "%u", cur_mid_servo_pwm_);
//     mid_servo_label_ -> setText(numstr);
// #endif
//     sprintf(numstr, "%u", cur_left_servo_pwm_);
//     left_servo_label_ -> setText(numstr);
//     sprintf(numstr, "%u", cur_right_servo_pwm_);
//     right_servo_label_ -> setText(numstr);
//     sprintf(numstr, "%u", cur_throttle_pwm_);
//     throttle_label_ -> setText(numstr);
// #ifdef FOUR_WING
//     sprintf(numstr, "%u", cur_throttle_2_pwm_);
//     throttle_2_label_ -> setText(numstr);
// #endif
// #ifdef TWO_WING
//     sprintf(numstr, "%u", cur_climb_pwm_);
//     climb_label_ -> setText(numstr);
// #endif

//     if(mode_id_ == tuning_mode){

//         if(!pid_id_label_ -> isVisible()){
//             pid_id_label_ -> setVisible(true);
//             pid_id_front_label_ -> setVisible(true);
//         }
//         switch(cur_pid_id_){
//             case(0):    //yaw
//                 pid_id_label_ -> setText("Yaw");
//             break;
//             case(1):    //pitch
//                 pid_id_label_ -> setText("Pitch");
//             break;
//             case(2):    //roll
//                 pid_id_label_ -> setText("Roll");
//             break;
//         }
//     }
//     else{
//         if(pid_id_label_ -> isVisible()){
//             pid_id_label_ -> setVisible(false);
//             pid_id_front_label_ -> setVisible(false);
//         }
//     }

//     if(!time_s_label_ -> isVisible())
//         boxLayoutVisible(time_layout_, true);
//     sprintf(numstr, "%u s", cur_time_ms_ / 1000000);
//     time_s_label_ -> setText(numstr);
//     sprintf(numstr, "%03u ms %03u us [开机时间]", (cur_time_ms_/1000) % 1000, cur_time_ms_ % 1000);
//     time_ms_label_ -> setText(numstr);

//     //temp marker for the 3d model vislization
//     // visualization_msgs::Marker marker;
//     // marker.pose.orientation.x = msg -> odom.pose.pose.orientation.x;
//     // marker.pose.orientation.y = msg -> odom.pose.pose.orientation.y;
//     // marker.pose.orientation.z = msg -> odom.pose.pose.orientation.z;
//     // marker.pose.orientation.w = msg -> odom.pose.pose.orientation.w;

//     // marker.header.frame_id = "map";
//     // marker.id = 1;
//     // marker.lifetime = ros::Duration(0.5);
//     // marker.color.a = 0.8;
//     // marker.color.r = 1; marker.color.g = 0; marker.color.b = 0;
//     // marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 2;
//     // marker.type = visualization_msgs::Marker::CUBE;
//     // marker.action = visualization_msgs::Marker::MODIFY;
//     // marker.frame_locked = false;
//     // marker.pose.position.x = 0;
//     // marker.pose.position.y = 0;

//     // vis_pub_.publish(marker);

//     //send tf transform
//     tf::Transform transform;
//     tf::Quaternion q;
//     q.setX(msg -> odom.pose.pose.orientation.x);
//     q.setY(msg -> odom.pose.pose.orientation.y);
//     q.setZ(msg -> odom.pose.pose.orientation.z);
//     q.setW(msg -> odom.pose.pose.orientation.w);
//     transform.setRotation(q);
//     transform.setOrigin(tf::Vector3(msg -> odom.pose.pose.position.x,
//                                     msg -> odom.pose.pose.position.y,
//                                     msg -> odom.pose.pose.position.z));
//     tf_pub_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

}

void FMAVStatusPanel::joystickReceive(const sensor_msgs::Joy::ConstPtr& msg){

    if(msg -> buttons[7] && is_connected){
        enableThrottle();
#ifdef FOUR_WING
        enableThrottle2();
#endif
    }

    flight_control_joysitck_ -> setJoystickPos(msg -> axes[0], msg -> axes[1]);

    // uint8_t old_throttle = throttle_pwm_set_;
#ifdef TWO_WING
    throttle_pwm_set_ = msg -> axes[4] > 0.0 ? (int)(msg -> axes[4] * THROTTLE_MAX) : 0;
    throttle_set_spin_ -> setValue(throttle_pwm_set_);
#endif
#ifdef FOUR_WING
    throttle_pwm_set_ = msg -> axes[4] > 0.0 ? (int)(msg -> axes[4] * THROTTLE_MAX - 100) : 0;
    throttle_set_spin_ -> setValue(throttle_pwm_set_);
    throttle_2_pwm_set_ = msg -> axes[4] > 0.0 ? (int)(msg -> axes[4] * (THROTTLE_MAX - 100)) : 0;
    throttle_2_set_spin_ -> setValue(throttle_2_pwm_set_);
#endif
    // if(is_throttle_enabled_ && is_connected && (old_throttle != throttle_pwm_set_))
    //     uploadConfig();
}

void FMAVStatusPanel::uploadConfig(){  //button slot: transfer config to fmav

    getParamValues();
    Q_EMIT configChanged();

    mav_comm_driver::MFPUnified msg;
    switch (param_mode_)
    {
        case default_mode:

        break;

        case servo_debug_mode:
            msg.msg_id = mav_comm_driver::MFPUnified::DOWN_MOTOR;
            msg.length = 10;
            msg.data.push_back(mav_comm_driver::MFPUnified::DOWN_MOTOR);
            msg.data.push_back(10);

            if(write_flash_checkbox_ -> isChecked())
                msg.data.push_back(0x01);
            else
                msg.data.push_back(0x00);

            if(is_throttle_enabled_)
                msg.data.push_back(0x01);
            else
                msg.data.push_back(0x00);
            
            msg.data.push_back(throttle_pwm_set_ >> 8);
            msg.data.push_back(throttle_pwm_set_);
#ifdef FOUR_WING
            msg.data.push_back(throttle_2_pwm_set_ >> 8);
            msg.data.push_back(throttle_2_pwm_set_);
#else
            msg.data.push_back(0);
            msg.data.push_back(0);
#endif
            msg.data.push_back(left_servo_pwm_set_ >> 8);
            msg.data.push_back(left_servo_pwm_set_);
            msg.data.push_back(right_servo_pwm_set_ >> 8);
            msg.data.push_back(right_servo_pwm_set_);
            displayMessage("下发舵机/电机参数", Qt::yellow);
            
        break;


    }
    msg.header.stamp = ros::Time::now();
    mav_config_pub_.publish(msg);
//     mav_comm_driver::ModeConfig msg;
//     uint i;

//     if(!is_connected) return;

//     switch(param_mode_){
//         case(fault_mode):
//             msg.mode_id = fault_mode;
//             msg.data.reserve(3);
//             msg.data.push_back(fault_mode);
//             msg.data.push_back(0x0d);
//             msg.data.push_back(0x0a);
//         break;

//         case(start_mode):
//             msg.mode_id = start_mode;
// #ifdef TWO_WING
//             msg.data.reserve(7);
// #else
//             msg.data.reserve(9);
// #endif
//             msg.data.push_back(start_mode);
//             msg.data.push_back(right_servo_pwm_set_);
//             msg.data.push_back(left_servo_pwm_set_);
// #ifdef TWO_WING
//             msg.data.push_back(mid_servo_pwm_set_);
//             if(is_throttle_enabled_)
//                 msg.data.push_back(throttle_pwm_set_);
//             else
//                 msg.data.push_back(0);
// #else
//             if(is_throttle_enabled_){
//                 msg.data.push_back(throttle_pwm_set_);
//                 msg.data.push_back(throttle_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
//             if(is_throttle_2_enabled_){
//                 msg.data.push_back(throttle_2_pwm_set_);
//                 msg.data.push_back(throttle_2_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
// #endif
//             msg.data.push_back(0x0d);
//             msg.data.push_back(0x0a);
//         break;

//         case(manual_mode):
//             msg.mode_id = manual_mode;
// #ifdef TWO_WING
//             msg.data.reserve(8);
// #else
//             msg.data.reserve(9);
// #endif
//             msg.data.push_back(manual_mode);
//             msg.data.push_back(right_servo_pwm_set_);
//             msg.data.push_back(left_servo_pwm_set_);
// #ifdef TWO_WING
//             msg.data.push_back(mid_servo_pwm_set_);

//             if(is_throttle_enabled_)
//                 msg.data.push_back(throttle_pwm_set_);
//             else
//                 msg.data.push_back(0);
            
//             msg.data.push_back(climb_pwm_set_);
// #else
//             if(is_throttle_enabled_){
//                 msg.data.push_back(throttle_pwm_set_);
//                 msg.data.push_back(throttle_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
//             if(is_throttle_2_enabled_){
//                 msg.data.push_back(throttle_2_pwm_set_);
//                 msg.data.push_back(throttle_2_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
// #endif
//             msg.data.push_back(0x0d);
//             msg.data.push_back(0x0a);
//         break;

//         case(flight_mode):
//             msg.mode_id = flight_mode;
//             msg.data.reserve(9);
//             msg.data.push_back(flight_mode);
//             msg.data.push_back(0);
//             msg.data.push_back(0);
//             msg.data.push_back(0);
//             msg.data.push_back(0);
// #ifdef TWO_WING
//             if(is_throttle_enabled_)
//                 msg.data.push_back(throttle_pwm_set_);
//             else
//                 msg.data.push_back(0);
// #endif
// #ifdef FOUR_WING
//             if(is_throttle_enabled_){
//                 msg.data.push_back(throttle_pwm_set_);
//                 msg.data.push_back(throttle_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
// #endif
            
//             msg.data.push_back(0x0d);
//             msg.data.push_back(0x0a);
//         break;

//         case(tuning_mode):
//             msg.mode_id = tuning_mode;
// #ifdef TWO_WING
//             msg.data.reserve(26);
// #else
//             msg.data.reserve(29);
// #endif
//             msg.data.push_back(tuning_mode);

//             int16_t tmp_16;
//             tmp_16 = (int16_t)(pid_int_set_[pid_id_set_][0] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             tmp_16 = (int16_t)(pid_int_set_[pid_id_set_][1] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             tmp_16 = (int16_t)(pid_int_set_[pid_id_set_][2] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);

//             tmp_16 = (int16_t)(pid_ext_set_[pid_id_set_][0] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             tmp_16 = (int16_t)(pid_ext_set_[pid_id_set_][1] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             tmp_16 = (int16_t)(pid_ext_set_[pid_id_set_][2] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);

//             msg.data.push_back(pid_freq_);

//             msg.data.push_back((uint8_t)pid_id_set_);

//             tmp_16 = (int16_t)(pid_ext_lowlimit_[pid_id_set_] * 10);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             tmp_16 = (int16_t)(pid_ext_uplimit_[pid_id_set_] * 10);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);
//             msg.data.push_back(pid_int_lowlimit_[pid_id_set_]);
//             msg.data.push_back(pid_int_uplimit_[pid_id_set_]);

//             tmp_16 = (int16_t)(pid_setvalue_[pid_id_set_] * 100);
//             msg.data.push_back(tmp_16);
//             msg.data.push_back(tmp_16 >> 8);

// #ifdef TWO_WING
//             if(is_throttle_enabled_)
//                 msg.data.push_back(throttle_pwm_set_);
//             else
//                 msg.data.push_back(0);
// #else
//             if(is_throttle_enabled_){
//                 msg.data.push_back(throttle_pwm_set_);
//                 msg.data.push_back(throttle_pwm_set_ >> 8);
//                 msg.data.push_back(throttle_pwm_set_);
//                 msg.data.push_back(throttle_pwm_set_ >> 8);
//             }
//             else{
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//                 msg.data.push_back(0);
//             }
// #endif

//             msg.data.push_back(0x0d);
//             msg.data.push_back(0x0a);

//         break;
//     }
//     if(write_flash_checkbox_ -> isChecked()){
//         msg.mode_id = msg.mode_id | 0x01;
//         msg.data[0] = msg.data[0] | 0x01;
//     }
//     mav_config_pub_.publish(msg);
}

void FMAVStatusPanel::downloadConfig(){ //button slot: download config from fmav
    
    mav_comm_driver::MFPUnified msg;
    
    switch (param_mode_)
    {
        case tuning_mode:
            msg.msg_id = mav_comm_driver::MFPUnified::DOWN_ACK;
            msg.length = 1;
            msg.data.push_back(mav_comm_driver::MFPUnified::DOWN_ACK);
            msg.data.push_back(1);
            msg.data.push_back(0x01);   // CMD 0x01 读取PID请求
        break;
    }

    msg.header.stamp = ros::Time::now();
    mav_config_pub_.publish(msg);
}

void FMAVStatusPanel::checkConnection(){
    if(mav_down_msg_cnt_ == mav_down_msg_cnt_prev_){
        is_connected = false;
        displayStatus();

        upload_to_mav_ -> setEnabled(false);
        download_from_mav_ -> setEnabled(false);
	    throttle_enable_ -> setEnabled(false);
        if(is_throttle_enabled_){
            throttle_pwm_set_ = 0;
            throttle_set_spin_ -> setValue(throttle_pwm_set_);
            is_throttle_enabled_ = false;
            throttle_enable_ -> setText("启动");
        }
#ifdef FOUR_WING
        throttle_2_enable_ -> setEnabled(false);
        if(is_throttle_2_enabled_){
            throttle_2_pwm_set_ = 0;
            throttle_2_set_spin_ -> setValue(throttle_2_pwm_set_);
            is_throttle_2_enabled_ = false;
            throttle_2_enable_ -> setText("启动");
        }
#endif

        boxLayoutVisible(time_layout_, false);
        if(joystick_send_timer_ -> isActive())
            joystick_send_timer_ -> stop();
    }
    else{
        mav_down_msg_cnt_prev_ = mav_down_msg_cnt_;
    }
}

void FMAVStatusPanel::setParamMode(int index){

    boxLayoutVisible(right_servo_set_layout_, false);
    boxLayoutVisible(left_servo_set_layout_, false);
    boxLayoutVisible(throttle_set_layout_, false);
#ifdef FOUR_WING
    boxLayoutVisible(throttle_2_set_layout_, false);
#endif
#ifdef TWO_WING
    boxLayoutVisible(mid_servo_set_layout_, false);
    boxLayoutVisible(climb_set_layout_, false);
#endif
    boxLayoutVisible(pid_tuning_layout_, false);
    boxLayoutVisible(vicon_test_layout_, false);
    boxLayoutVisible(sensor_calib_layout_, false);
    // if(param_mode_ == vicon_test_mode)
        // system("rosnode kill /vicon_bridge");
    
    joystick_sub_.shutdown();
    if(joystick_send_timer_ -> isActive())
        joystick_send_timer_ -> stop();
    
    if(is_throttle_enabled_){
        if(is_connected)
            enableThrottle();
        else{
            throttle_pwm_set_ = 0;
            throttle_set_spin_ -> setValue(throttle_pwm_set_);
            is_throttle_enabled_ = false;
            throttle_enable_ -> setText("启动");
        }
    }

#ifdef FOUR_WING
    throttle_set_slider_ -> setRange(0, THROTTLE_MAX);
    throttle_2_set_slider_ -> setRange(0, THROTTLE_MAX);
#endif

#ifdef FOUR_WING
    if(is_throttle_2_enabled_){
        if(is_connected)
            enableThrottle2();
        else{
            throttle_2_pwm_set_ = 0;
            throttle_2_set_spin_ -> setValue(throttle_2_pwm_set_);
            is_throttle_2_enabled_ = false;
            throttle_2_enable_ -> setText("启动");
        }
    }
#endif

    flight_control_joysitck_ -> setVisible(false);

    switch(mode_sel_combo_ -> itemData(index).toInt()){
        case(default_mode):    //DEFAULT_MODE
            param_mode_ = default_mode;
            break;

        case(servo_debug_mode):    //SERVO_DEBUG_MODE
            param_mode_ = servo_debug_mode;
            boxLayoutVisible(right_servo_set_layout_, true);
#ifdef TWO_WING
            boxLayoutVisible(mid_servo_set_layout_, true);
            boxLayoutVisible(climb_set_layout_, true);
#endif
            boxLayoutVisible(left_servo_set_layout_, true);
            boxLayoutVisible(throttle_set_layout_, true);
#ifdef FOUR_WING
            boxLayoutVisible(throttle_2_set_layout_, true);
#endif
            break;
        
        case(sensor_alignment_mode):
            param_mode_ = sensor_alignment_mode;
            boxLayoutVisible(sensor_calib_layout_, true);
            break;

        case(flight_mode):    //FLIGHT_MODE
            param_mode_ = flight_mode;
            boxLayoutVisible(throttle_set_layout_, true);
            joystick_sub_ = nh_.subscribe("/joy", 10, &FMAVStatusPanel::joystickReceive, this);
            flight_control_joysitck_ -> setVisible(true);
#ifdef FOUR_WING
            throttle_set_slider_ -> setRange(0, THROTTLE_MAX - 100);
            throttle_2_set_slider_ -> setRange(0, THROTTLE_MAX - 100);
#endif
            break;

        case(tuning_mode):    //TUNING MODE
            param_mode_ = tuning_mode;
            boxLayoutVisible(throttle_set_layout_, true);
            boxLayoutVisible(pid_tuning_layout_, true);
#ifdef FOUR_WING
            throttle_set_slider_ -> setRange(0, THROTTLE_MAX - 100);
            throttle_2_set_slider_ -> setRange(0, THROTTLE_MAX - 100);
#endif
            break;
        
        case(vicon_test_mode):    //VICON TEST MODE
            param_mode_ = vicon_test_mode;
            boxLayoutVisible(vicon_test_layout_, true);
            break;
    }
}

void FMAVStatusPanel::getParamValues(){

#ifdef TWO_WING
    mid_servo_pwm_set_ = mid_servo_set_spin_ -> value();
    climb_pwm_set_ = climb_set_spin_ -> value();
#endif
    left_servo_pwm_set_ = left_servo_set_spin_ -> value();
    right_servo_pwm_set_ = right_servo_set_spin_ -> value();
    throttle_pwm_set_ = throttle_set_spin_ -> value();
#ifdef FOUR_WING
    throttle_2_pwm_set_ = throttle_2_set_spin_ -> value();
#endif
    

    pid_id_set_ = pid_id_btn_group_ -> checkedId();

    pid_ext_set_[pid_id_set_][0] = pid_ext_p_edit_ -> text().toFloat();
    pid_ext_set_[pid_id_set_][1] = pid_ext_i_edit_ -> text().toFloat();
    pid_ext_set_[pid_id_set_][2] = pid_ext_d_edit_ -> text().toFloat();

    pid_int_set_[pid_id_set_][0] = pid_int_p_edit_ -> text().toFloat();
    pid_int_set_[pid_id_set_][1] = pid_int_i_edit_ -> text().toFloat();
    pid_int_set_[pid_id_set_][2] = pid_int_d_edit_ -> text().toFloat();

    pid_ext_lowlimit_[pid_id_set_] = pid_ext_lowlimit_edit_ -> text().toFloat();
    pid_ext_uplimit_[pid_id_set_] = pid_ext_uplimit_edit_ -> text().toFloat();
    pid_int_lowlimit_[pid_id_set_] = pid_int_lowlimit_edit_ -> text().toUInt();
    pid_int_uplimit_[pid_id_set_] = pid_int_uplimit_edit_ -> text().toUInt();
    pid_setvalue_[pid_id_set_] = pid_setvalue_edit_ -> text().toFloat();
    pid_freq_ = pid_freq_edit_ -> text().toUInt();

    mag_offset_set_[0] = mag_offset_x_edit_ -> text().toDouble();
    mag_offset_set_[1] = mag_offset_y_edit_ -> text().toDouble();
    mag_offset_set_[2] = mag_offset_z_edit_ -> text().toDouble();
    mag_gain_set_[0] = mag_gain_x_edit_ -> text().toDouble();
    mag_gain_set_[1] = mag_gain_y_edit_ -> text().toDouble();
    mag_gain_set_[2] = mag_gain_z_edit_ -> text().toDouble();

}

void FMAVStatusPanel::setPanelValues(){

#ifdef TWO_WING
    mid_servo_set_spin_ -> setValue(mid_servo_pwm_set_);
    climb_set_spin_ -> setValue(climb_pwm_set_);
#endif
    left_servo_set_spin_ -> setValue(left_servo_pwm_set_);
    right_servo_set_spin_ -> setValue(right_servo_pwm_set_);
    throttle_set_spin_ -> setValue(throttle_pwm_set_);
#ifdef FOUR_WING
    throttle_2_set_spin_ -> setValue(throttle_2_pwm_set_);
#endif

    switch(pid_id_set_){
        case(0):
            pid_id_yaw_ -> setChecked(true);
            pid_ext_front_label_ -> setText("外环PID参数 - Yaw");
            pid_int_front_label_ -> setText("内环PID参数 - Yaw");
            pid_setvalue_front_label_ -> setText("设定角度目标值 - Yaw");
        break;
        case(1):
            pid_id_pitch_ -> setChecked(true);
            pid_ext_front_label_ -> setText("外环PID参数 - Pitch");
            pid_int_front_label_ -> setText("内环PID参数 - Pitch");
            pid_setvalue_front_label_ -> setText("设定角度目标值 - Pitch");
        break;
        case(2):
            pid_id_roll_ -> setChecked(true);
            pid_ext_front_label_ -> setText("外环PID参数 - Roll");
            pid_int_front_label_ -> setText("内环PID参数 - Roll");
            pid_setvalue_front_label_ -> setText("设定角度目标值 - Roll");
        break;
    }
    
    pid_ext_p_edit_ -> setText(QString::number(pid_ext_set_[pid_id_set_][0]));
    pid_ext_i_edit_ -> setText(QString::number(pid_ext_set_[pid_id_set_][1]));
    pid_ext_d_edit_ -> setText(QString::number(pid_ext_set_[pid_id_set_][2]));

    pid_int_p_edit_ -> setText(QString::number(pid_int_set_[pid_id_set_][0]));
    pid_int_i_edit_ -> setText(QString::number(pid_int_set_[pid_id_set_][1]));
    pid_int_d_edit_ -> setText(QString::number(pid_int_set_[pid_id_set_][2]));

    pid_ext_lowlimit_edit_ -> setText(QString::number(pid_ext_lowlimit_[pid_id_set_]));
    pid_ext_uplimit_edit_ -> setText(QString::number(pid_ext_uplimit_[pid_id_set_]));
    pid_int_lowlimit_edit_ -> setText(QString::number(pid_int_lowlimit_[pid_id_set_]));
    pid_int_uplimit_edit_ -> setText(QString::number(pid_int_uplimit_[pid_id_set_]));
    pid_setvalue_edit_ -> setText(QString::number(pid_setvalue_[pid_id_set_]));
    pid_freq_edit_ -> setText(QString::number(pid_freq_));

    mag_offset_x_edit_ -> setText(QString::number(mag_offset_set_[0]));
    mag_offset_y_edit_ -> setText(QString::number(mag_offset_set_[1]));
    mag_offset_z_edit_ -> setText(QString::number(mag_offset_set_[2]));
    mag_gain_x_edit_ -> setText(QString::number(mag_gain_set_[0]));
    mag_gain_y_edit_ -> setText(QString::number(mag_gain_set_[1]));
    mag_gain_z_edit_ -> setText(QString::number(mag_gain_set_[2]));
    
}

void FMAVStatusPanel::changeTuningAxis(int btn_id){

    pid_id_set_ = btn_id;
    setPanelValues();
}

void FMAVStatusPanel::enableThrottle(){

    ros::Rate r(10);
    uint count;
    getParamValues();

    if(is_throttle_enabled_){
	    is_throttle_enabled_ = false;
        count = 0;
        do{
            if(count >= 100){
                is_throttle_enabled_ = true;
                QMessageBox::critical(this, "错误", "发送超时");
                return;
            }
            uploadConfig();
            ros::spinOnce();
            r.sleep();
            count ++;
        }while(cur_throttle_pwm_ != 0);
        throttle_enable_ -> setText("启动");
    }
    else{
        // if(throttle_pwm_set_ == 0){
        //     QMessageBox::warning(this, "警告", "当前设置油门量为0");
        //     return;
        // }

        is_throttle_enabled_ = true;
        // count = 0;
        // do {
        //     if(count >= 100){
        //         is_throttle_enabled_ = false;
        //         QMessageBox::critical(this, "错误", "发送超时");
        //         return;
        //     }
        //     uploadConfig();
        //     ros::spinOnce();
        //     r.sleep();
        //     count ++;
        // } while(cur_throttle_pwm_ != throttle_pwm_set_);
        uploadConfig();

        throttle_enable_ -> setText("急停");
    }

    if(is_throttle_enabled_ && param_mode_ == flight_mode)
        joystick_send_timer_ -> start(100);
    else if(joystick_send_timer_ -> isActive())
        joystick_send_timer_ -> stop();
    
}

#ifdef FOUR_WING
void FMAVStatusPanel::enableThrottle2(){

    ros::Rate r(10);
    uint count;
    getParamValues();

    if(is_throttle_2_enabled_){
	    is_throttle_2_enabled_ = false;
        count = 0;
        do{
            if(count >= 100){
                is_throttle_2_enabled_ = true;
                QMessageBox::critical(this, "错误", "发送超时");
                return;
            }
            uploadConfig();
            ros::spinOnce();
            r.sleep();
            count ++;
        }while(cur_throttle_2_pwm_ != 0);
        throttle_2_enable_ -> setText("启动");
    }
    else{
        // if(throttle_pwm_set_ == 0){
        //     QMessageBox::warning(this, "警告", "当前设置油门量为0");
        //     return;
        // }

        is_throttle_2_enabled_ = true;
        // count = 0;
        // do {
        //     if(count >= 100){
        //         is_throttle_2_enabled_ = false;
        //         QMessageBox::critical(this, "错误", "发送超时");
        //         return;
        //     }
        //     uploadConfig();
        //     ros::spinOnce();
        //     r.sleep();
        //     count ++;
        // } while(cur_throttle_2_pwm_ != throttle_2_pwm_set_);
	    uploadConfig();
        throttle_2_enable_ -> setText("急停");
    }
    
}
#endif

void FMAVStatusPanel::joystickMove(float x, float y){

    ROS_INFO("I hear:%f,%f",x,y);

}

void FMAVStatusPanel::uploadJoystick(){
    if(is_throttle_enabled_)
        uploadConfig();
}

void FMAVStatusPanel::boxLayoutVisible(QBoxLayout *boxLayout, bool bVisible)
{
	if (NULL == boxLayout) return;

    std::queue<QLayout*> q;
    int nColum;
    q.push(boxLayout);

    while(!q.empty()){

        nColum = q.front() -> count();
        for (int i = 0; i < nColum; i++)
        {
            auto item = q.front() -> itemAt(i);
            if (item)
            {
                if(item -> widget()){
                    item -> widget() -> setVisible(bVisible);
                }
                else if(item -> layout()){
                    q.push(item -> layout());
                }
            }
        }
        q.pop();
    }
	
}

void FMAVStatusPanel::refreshViconTopicList(){

    std::stringstream ss(getCommandOutput("rostopic list"));
    std::string topic;
    vicon_topic_combo_ -> clear();
    while(ss >> topic){
        if(topic.find("/vicon") != topic.npos){
            vicon_topic_combo_ -> addItem(topic.c_str());
        }
    }
    return;
}

// 读取命令行输出
std::string FMAVStatusPanel::getCommandOutput(const char* cmd){

    char buffer[1024];
    FILE *fp;
    std::string res;
    if((fp = popen(cmd, "r")) != NULL)
    {
        while(fgets(buffer, 1024, fp) != NULL)
           res.append(buffer);
        pclose(fp);
    }
    return res;
}

void FMAVStatusPanel::viconStartEnd(){
    if(!is_vicon_started){
        std::string topic = vicon_topic_combo_ -> currentText().toStdString();
        if(!topic.empty()){
            vicon_sub_ = nh_.subscribe(topic, 1000, &FMAVStatusPanel::viconReceive, this);

            topic = "rosrun rviz_teleop_commander vicon_transfer " + topic + "&";
            system(topic.c_str());

            vicon_start_btn_ -> setText("结束Vicon转发");
            is_vicon_started = true;
        }
        else QMessageBox::critical(this, "错误", "无效话题名");
    }
    else{
        vicon_sub_.shutdown();

        system("rosnode kill /vicon_transfer");
        vicon_start_btn_ -> setText("开始Vicon转发");
        is_vicon_started = false;
    }
}

void FMAVStatusPanel::viconReceive(const geometry_msgs::TransformStamped::ConstPtr& msg){
    
    //send tf transform
    geometry_msgs::TransformStamped new_tf_msg;
    new_tf_msg = *msg;
    new_tf_msg.header.frame_id = "map";
    new_tf_msg.child_frame_id = "base_link";
    tf_pub_.sendTransform(new_tf_msg);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg -> transform.rotation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);   //In radians

    char numstr[30];
    sprintf(numstr, "%.2f", roll / DEG2RAD);
    roll_label_ -> setText(numstr);
    sprintf(numstr, "%.2f", pitch / DEG2RAD);
    pitch_label_ -> setText(numstr);
    sprintf(numstr, "%.2f", yaw / DEG2RAD);
    yaw_label_ -> setText(numstr);

    return;
}

// 显示消息
void FMAVStatusPanel::displayMessage(const QString& msg, const QColor &color){

    if(is_displaying_msg){
        if(message_display_timer_ -> isActive())
            message_display_timer_ -> stop();
    }

    is_displaying_msg = true;
    message_display_timer_ -> start(1000); //T = 1000ms
    ROS_INFO_STREAM(msg.toStdString());
    //设置消息
    QPalette palette;
    mode_label_ -> setText(msg);
    palette.setColor(QPalette::Background, color);
    mode_label_ -> setPalette(palette);
    palette.setColor(QPalette::WindowText, QColor(255 - color.red(), 255 - color.green(), 255 - color.blue()));
    mode_label_ -> setPalette(palette);

}

void FMAVStatusPanel::endMessage(){
    
    message_display_timer_ -> stop();
    is_displaying_msg = false;
    displayStatus();
    return;
}

void FMAVStatusPanel::displayStatus(){

    QPalette palette;
    if(is_connected){
        if(!is_vicon_started){
            mode_label_ -> setText("连接正常");
            palette.setColor(QPalette::Background, QColor(0, 255, 0));
            mode_label_ -> setPalette(palette);
            palette.setColor(QPalette::WindowText, QColor(200, 0, 0));
            mode_label_ -> setPalette(palette);
        }
        else{
            mode_label_ -> setText("Vicon测试中");
            palette.setColor(QPalette::Background, QColor(255, 165, 0));
            mode_label_ -> setPalette(palette);
            palette.setColor(QPalette::WindowText, QColor(200, 0, 255));
            mode_label_ -> setPalette(palette);
        }

    }
    else{
        mode_label_ -> setText("连接断开");
        palette.setColor(QPalette::Background, QColor(100, 100, 100));
        mode_label_ -> setPalette(palette);
        palette.setColor(QPalette::WindowText, QColor(Qt::white));
        mode_label_ -> setPalette(palette);
    }
    return;
}

// 磁力计校准
void FMAVStatusPanel::calibrateMag(){

    if(is_calibrating_mag){
        is_calibrating_mag = false;
        mag_calib_start_btn_ -> setText("磁力计校准");
    }
    else{
        if(is_connected){
            mag_data_cnt_ = 0;
            mag_calib_vis_msg_.markers.resize(1);
            mag_calib_vis_pts_.points.clear();
            mag_calib_vis_pts_.header.frame_id = "map";
            mag_calib_vis_pts_.ns = "mag_pts";
            mag_calib_vis_pts_.id = 104;
            mag_calib_vis_pts_.type = visualization_msgs::Marker::POINTS;
            mag_calib_vis_pts_.action = visualization_msgs::Marker::MODIFY;
            mag_calib_vis_pts_.color.r = 0.0;
            mag_calib_vis_pts_.color.g = 0.8;
            mag_calib_vis_pts_.color.b = 0.0;
            mag_calib_vis_pts_.color.a = 1.0;
            mag_calib_vis_pts_.scale.x = 6.0 / MAG_CALIB_POINT_SCALE;
            mag_calib_vis_pts_.scale.y = 6.0 / MAG_CALIB_POINT_SCALE;

            is_calibrating_mag = true;
            mag_calib_start_btn_ -> setText("中止磁力计校准");
            displayMessage("校准磁力计中...", Qt::yellow);

        }
        else{
            QMessageBox::critical(this, "错误", "飞行器尚未连接或通讯故障");
        }
    }
}

// 清除磁力计图形
void FMAVStatusPanel::clearCalibrateVisualization(){

    if(!is_calibrating_mag){
        mag_calib_vis_msg_.markers.resize(1);
        
        mag_calib_vis_pts_.header.frame_id = "map";
        mag_calib_vis_pts_.id = 104;
        mag_calib_vis_pts_.points.clear();
        mag_calib_vis_pts_.action = visualization_msgs::Marker::DELETEALL;
        mag_calib_vis_msg_.markers[0] = mag_calib_vis_pts_;

        mag_calib_vis_pub_.publish(mag_calib_vis_msg_);
    }

}

// 重载父类的功能
void FMAVStatusPanel::save( rviz::Config config ) const
{   
    rviz::Panel::save( config );
    config.mapSetValue("MidServoPWMSet", mid_servo_pwm_set_);
    config.mapSetValue("LeftServoPWMSet", left_servo_pwm_set_);
    config.mapSetValue("RightServoPWMSet", right_servo_pwm_set_);
    config.mapSetValue("ThrottleServoPWMSet", throttle_pwm_set_);
#ifdef FOUR_WING
    config.mapSetValue("Throttle2ServoPWMSet", throttle_2_pwm_set_);
#endif
    config.mapSetValue("ClimbServoPWMSet", climb_pwm_set_);

    config.mapSetValue("PIDExtRollSet0", pid_ext_set_[2][0]);
    config.mapSetValue("PIDExtRollSet1", pid_ext_set_[2][1]);
    config.mapSetValue("PIDExtRollSet2", pid_ext_set_[2][2]);
    config.mapSetValue("PIDExtPitchSet0", pid_ext_set_[1][0]);
    config.mapSetValue("PIDExtPitchSet1", pid_ext_set_[1][1]);
    config.mapSetValue("PIDExtPitchSet2", pid_ext_set_[1][2]);
    config.mapSetValue("PIDExtYawSet0", pid_ext_set_[0][0]);
    config.mapSetValue("PIDExtYawSet1", pid_ext_set_[0][1]);
    config.mapSetValue("PIDExtYawSet2", pid_ext_set_[0][2]);
    config.mapSetValue("PIDIntRollSet0", pid_int_set_[2][0]);
    config.mapSetValue("PIDIntRollSet1", pid_int_set_[2][1]);
    config.mapSetValue("PIDIntRollSet2", pid_int_set_[2][2]);
    config.mapSetValue("PIDIntPitchSet0", pid_int_set_[1][0]);
    config.mapSetValue("PIDIntPitchSet1", pid_int_set_[1][1]);
    config.mapSetValue("PIDIntPitchSet2", pid_int_set_[1][2]);
    config.mapSetValue("PIDIntYawSet0", pid_int_set_[0][0]);
    config.mapSetValue("PIDIntYawSet1", pid_int_set_[0][1]);
    config.mapSetValue("PIDIntYawSet2", pid_int_set_[0][2]);
    config.mapSetValue("PIDID", pid_id_set_);

    config.mapSetValue("PIDFrequency", pid_freq_);
    config.mapSetValue("PIDSetValueYaw", pid_setvalue_[0]);
    config.mapSetValue("PIDSetValuePitch", pid_setvalue_[1]);
    config.mapSetValue("PIDSetValueRoll", pid_setvalue_[2]);
    config.mapSetValue("PIDExtUplimitYaw", pid_ext_uplimit_[0]);
    config.mapSetValue("PIDExtUplimitPitch", pid_ext_uplimit_[1]);
    config.mapSetValue("PIDExtUplimitRoll", pid_ext_uplimit_[2]);
    config.mapSetValue("PIDExtLowlimitYaw", pid_ext_lowlimit_[0]);
    config.mapSetValue("PIDExtLowlimitPitch", pid_ext_lowlimit_[1]);
    config.mapSetValue("PIDExtLowlimitRoll", pid_ext_lowlimit_[2]);
    config.mapSetValue("PIDIntUplimitYaw", pid_int_uplimit_[0]);
    config.mapSetValue("PIDIntUplimitPitch", pid_int_uplimit_[1]);
    config.mapSetValue("PIDIntUplimitRoll", pid_int_uplimit_[2]);
    config.mapSetValue("PIDIntLowlimitYaw", pid_int_lowlimit_[0]);
    config.mapSetValue("PIDIntLowlimitPitch", pid_int_lowlimit_[1]);
    config.mapSetValue("PIDIntLowlimitRoll", pid_int_lowlimit_[2]);

    config.mapSetValue("SensorMagOffsetX", mag_offset_set_[0]);
    config.mapSetValue("SensorMagOffsetY", mag_offset_set_[1]);
    config.mapSetValue("SensorMagOffsetZ", mag_offset_set_[2]);
    config.mapSetValue("SensorMagGainX", mag_gain_set_[0]);
    config.mapSetValue("SensorMagGainY", mag_gain_set_[1]);
    config.mapSetValue("SensorMagGainZ", mag_gain_set_[2]);

}

// 重载父类的功能，加载配置数据
void FMAVStatusPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
    QVariant v_tmp;
    config.mapGetValue("MidServoPWMSet", &v_tmp);  mid_servo_pwm_set_ = v_tmp.toUInt();
    config.mapGetValue("LeftServoPWMSet", &v_tmp); left_servo_pwm_set_ = v_tmp.toUInt();
    config.mapGetValue("RightServoPWMSet", &v_tmp); right_servo_pwm_set_ = v_tmp.toUInt();
    config.mapGetValue("ThrottleServoPWMSet", &v_tmp); throttle_pwm_set_ = v_tmp.toUInt();
#ifdef FOUR_WING
    config.mapGetValue("Throttle2ServoPWMSet", &v_tmp); throttle_2_pwm_set_ = v_tmp.toUInt();
#endif
    config.mapGetValue("ClimbServoPWMSet", &v_tmp); climb_pwm_set_ = v_tmp.toUInt();

    config.mapGetValue("PIDExtRollSet0", &v_tmp); pid_ext_set_[2][0] = v_tmp.toFloat();
    config.mapGetValue("PIDExtRollSet1", &v_tmp); pid_ext_set_[2][1] = v_tmp.toFloat();
    config.mapGetValue("PIDExtRollSet2", &v_tmp); pid_ext_set_[2][2] = v_tmp.toFloat();
    config.mapGetValue("PIDExtPitchSet0", &v_tmp); pid_ext_set_[1][0] = v_tmp.toFloat();
    config.mapGetValue("PIDExtPitchSet1", &v_tmp); pid_ext_set_[1][1] = v_tmp.toFloat();
    config.mapGetValue("PIDExtPitchSet2", &v_tmp); pid_ext_set_[1][2] = v_tmp.toFloat();
    config.mapGetValue("PIDExtYawSet0", &v_tmp); pid_ext_set_[0][0] = v_tmp.toFloat();
    config.mapGetValue("PIDExtYawSet1", &v_tmp); pid_ext_set_[0][1] = v_tmp.toFloat();
    config.mapGetValue("PIDExtYawSet2", &v_tmp); pid_ext_set_[0][2] = v_tmp.toFloat();
    config.mapGetValue("PIDIntRollSet0", &v_tmp); pid_int_set_[2][0] = v_tmp.toFloat();
    config.mapGetValue("PIDIntRollSet1", &v_tmp); pid_int_set_[2][1] = v_tmp.toFloat();
    config.mapGetValue("PIDIntRollSet2", &v_tmp); pid_int_set_[2][2] = v_tmp.toFloat();
    config.mapGetValue("PIDIntPitchSet0", &v_tmp); pid_int_set_[1][0] = v_tmp.toFloat();
    config.mapGetValue("PIDIntPitchSet1", &v_tmp); pid_int_set_[1][1] = v_tmp.toFloat();
    config.mapGetValue("PIDIntPitchSet2", &v_tmp); pid_int_set_[1][2] = v_tmp.toFloat();
    config.mapGetValue("PIDIntYawSet0", &v_tmp); pid_int_set_[0][0] = v_tmp.toFloat();
    config.mapGetValue("PIDIntYawSet1", &v_tmp); pid_int_set_[0][1] = v_tmp.toFloat();
    config.mapGetValue("PIDIntYawSet2", &v_tmp); pid_int_set_[0][2] = v_tmp.toFloat();
    config.mapGetValue("PIDID", &v_tmp); pid_id_set_ = v_tmp.toUInt();

    config.mapGetValue("PIDFrequency", &v_tmp); pid_freq_ = v_tmp.toUInt();
    config.mapGetValue("PIDSetValueYaw", &v_tmp); pid_setvalue_[0] = v_tmp.toFloat();
    config.mapGetValue("PIDSetValuePitch", &v_tmp); pid_setvalue_[1] = v_tmp.toFloat();
    config.mapGetValue("PIDSetValueRoll", &v_tmp); pid_setvalue_[2] = v_tmp.toFloat();
    config.mapGetValue("PIDExtUplimitYaw", &v_tmp); pid_ext_uplimit_[0] = v_tmp.toFloat();
    config.mapGetValue("PIDExtUplimitPitch", &v_tmp); pid_ext_uplimit_[1] = v_tmp.toFloat();
    config.mapGetValue("PIDExtUplimitRoll", &v_tmp); pid_ext_uplimit_[2] = v_tmp.toFloat();
    config.mapGetValue("PIDExtLowlimitYaw", &v_tmp); pid_ext_lowlimit_[0] = v_tmp.toFloat();
    config.mapGetValue("PIDExtLowlimitPitch", &v_tmp); pid_ext_lowlimit_[1] = v_tmp.toFloat();
    config.mapGetValue("PIDExtLowlimitRoll", &v_tmp); pid_ext_lowlimit_[2] = v_tmp.toFloat();
    config.mapGetValue("PIDIntUplimitYaw", &v_tmp); pid_int_uplimit_[0] = v_tmp.toUInt();
    config.mapGetValue("PIDIntUplimitPitch", &v_tmp); pid_int_uplimit_[1] = v_tmp.toUInt();
    config.mapGetValue("PIDIntUplimitRoll", &v_tmp); pid_int_uplimit_[2] = v_tmp.toUInt();
    config.mapGetValue("PIDIntLowlimitYaw", &v_tmp); pid_int_lowlimit_[0] = v_tmp.toUInt();
    config.mapGetValue("PIDIntLowlimitPitch", &v_tmp); pid_int_lowlimit_[1] = v_tmp.toUInt();
    config.mapGetValue("PIDIntLowlimitRoll", &v_tmp); pid_int_lowlimit_[2] = v_tmp.toUInt();

    config.mapGetValue("SensorMagOffsetX", &v_tmp); mag_offset_set_[0] = v_tmp.toDouble();
    config.mapGetValue("SensorMagOffsetY", &v_tmp); mag_offset_set_[1] = v_tmp.toDouble();
    config.mapGetValue("SensorMagOffsetZ", &v_tmp); mag_offset_set_[2] = v_tmp.toDouble();
    config.mapGetValue("SensorMagGainX", &v_tmp); mag_gain_set_[0] = v_tmp.toDouble();
    config.mapGetValue("SensorMagGainY", &v_tmp); mag_gain_set_[1] = v_tmp.toDouble();
    config.mapGetValue("SensorMagGainZ", &v_tmp); mag_gain_set_[2] = v_tmp.toDouble();

    setPanelValues();
    ROS_INFO("Parameter History Loaded.");

}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::FMAVStatusPanel,rviz::Panel )
// END_TUTORIAL
