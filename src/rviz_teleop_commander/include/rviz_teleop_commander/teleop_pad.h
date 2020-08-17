#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <queue>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QBoxLayout>
#include <QCheckBox>
#include <QButtonGroup>
#include <QMessageBox>
#include <QTimer>

// #include <mav_comm_driver/MAVStatus.h>
#include <mav_comm_driver/ModeConfig.h>
#include <mav_comm_driver/MFPUnified.h>
#include <rviz_teleop_commander/joystick_widget.h>

#include "EllipsoidFit.h"
#include <Eigen/Dense>
#endif

#ifdef TWO_WING
    #define THROTTLE_MAX 100
#else
    #define THROTTLE_MAX 65536
#endif

#define DEG2RAD (M_PI/180.0)

class QLineEdit;

namespace rviz_teleop_commander
{

const uint16_t servo_pwm_max = 2100;
const uint16_t servo_pwm_min = 900;


// 所有的plugin都必须是rviz::Panel的子类
class FMAVStatusPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    FMAVStatusPanel( QWidget* parent = 0 );

    ~FMAVStatusPanel();

    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
    // 中，数据就是topic的名称
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

// 公共槽.
// public Q_SLOTS:
    // 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher
    // void setTopic( const QString& topic );

// 内部槽.
protected Q_SLOTS:
    void updateMAVStatus(const mav_comm_driver::MFPUnified::ConstPtr&);
    void uploadConfig();
    void downloadConfig();
    void checkConnection();
    void setParamMode(int);
    void changeTuningAxis(int);
    void enableThrottleDebug();
    void uploadJoystick();
    void refreshViconTopicList();
    void viconControlStartEnd();
    void viconViewStartEnd();
    void joystickMove(float x, float y);
    void endMessage();
    void calibrateMag();
    void clearCalibrateVisualization();

protected:
    void joystickReceive(const sensor_msgs::Joy::ConstPtr&);
    void viconReceive(const geometry_msgs::TransformStamped::ConstPtr&);
    void boxLayoutVisible(QBoxLayout *, bool);
    void getParamValues();
    void setPanelValues();
    std::string getCommandOutput(const char* cmd);

    typedef struct{
        std::string text;
        QColor color;
        uint time;  // in ms
        uint priority;
    }message_t;
    void displayMessage(const message_t &, const bool);
    void displayStatus();
    void initPosePublish();

// 内部变量.
protected:

    // 图标logo & 电池
    QHBoxLayout* logo_voltage_layout_;
    QLabel* logo_img_;
    QVBoxLayout* voltage_layout_;
    QLabel* voltage_front_label_;
    QLabel* voltage_label_;

    // 模式显示控件
    QLabel* mode_front_label_;
    QLabel* mode_label_;
    QLabel* is_acc_healthy_label_;
    
    // 时间显示控件
    QHBoxLayout* time_layout_;
    QLabel* time_s_label_;
    QLabel* time_ms_label_;

    // 传感器值显示控件
    QLabel* roll_front_label_;
    QLabel* roll_label_;
    QLabel* pitch_front_label_;
    QLabel* pitch_label_;
    QLabel* yaw_front_label_;
    QLabel* yaw_label_;
    QLabel* roll_rate_front_label_;
    QLabel* roll_rate_label_;
    QLabel* pitch_rate_front_label_;
    QLabel* pitch_rate_label_;
    QLabel* yaw_rate_front_label_;
    QLabel* yaw_rate_label_;
    QLabel* height_front_label_;
    QLabel* height_label_;
#ifdef TWO_WING
    QLabel* mid_servo_label_;
    QLabel* mid_servo_front_label_;
#endif
    QLabel* left_servo_label_;
    QLabel* left_servo_front_label_;
    QLabel* right_servo_label_;
    QLabel* right_servo_front_label_;
    QLabel* throttle_label_;
    QLabel* throttle_front_label_;
#ifdef FOUR_WING
    QLabel* throttle_2_label_;
    QLabel* throttle_2_front_label_;
#endif
#ifdef TWO_WING
    QLabel* climb_label_;
    QLabel* climb_front_label_;
#endif
    QLabel* pid_id_front_label_;
    QLabel* pid_id_label_;

    //回传数据变量
    uint8_t mode_id_;   // 0x00:manual, 0x01:altitude, 0x02:position, 0xff:not determined
    uint8_t sub_mode_id_;   // 0x00:none, 0x01:takeoff, 0x02:landing, 0x03:emergency
    float cur_roll_;
    float cur_pitch_;
    float cur_yaw_;
    float cur_height_;
    float cur_roll_rate_;
    float cur_pitch_rate_;
    float cur_yaw_rate_;
    float cur_mag_raw_[3];
    float cur_acc_raw_[3];
    float cur_voltage_;
    uint16_t cur_mid_servo_pwm_;
    uint16_t cur_left_servo_pwm_;
    uint16_t cur_right_servo_pwm_;
    uint16_t cur_throttle_pwm_;
    uint16_t cur_throttle_2_pwm_;

    uint16_t cur_climb_pwm_;
    uint32_t cur_time_ms_;
    uint8_t cur_pid_id_;

    //参数设置标题控件
    QLabel* para_front_label_;
    QPushButton* upload_to_mav_;
    QPushButton* download_from_mav_;
    QLabel* write_flash_front_label_;
    QCheckBox* write_flash_checkbox_;

    QLabel* mode_sel_front_label_;
    QComboBox* mode_sel_combo_;

    //当前参数表模式
    uint8_t param_mode_;

    static const uint8_t default_mode = 0;
    static const uint8_t servo_debug_mode = 1;
    static const uint8_t manual_mode = 7;
    static const uint8_t sensor_alignment_mode = 2;
    static const uint8_t flight_mode = 3;
    static const uint8_t tuning_mode = 4;
    static const uint8_t pos_tuning_mode = 5;
    static const uint8_t vicon_test_mode = 6;
    static const uint8_t debug_mode = 7;


    //舵机/电机调试模式控件
    QHBoxLayout* right_servo_set_layout_;
    QLabel* right_servo_set_front_label_;
    QSlider* right_servo_set_slider_;
    QSpinBox* right_servo_set_spin_;
    QHBoxLayout* left_servo_set_layout_;
    QLabel* left_servo_set_front_label_;
    QSlider* left_servo_set_slider_;
    QSpinBox* left_servo_set_spin_;
#ifdef TWO_WING
    QHBoxLayout* mid_servo_set_layout_;
    QLabel* mid_servo_set_front_label_;
    QSlider* mid_servo_set_slider_;
    QSpinBox* mid_servo_set_spin_;
#endif
    QVBoxLayout* throttle_layout_;
    QPushButton* throttle_debug_enable_;
    QHBoxLayout* throttle_set_layout_;
    QLabel* throttle_set_front_label_;
    QSlider* throttle_set_slider_;
    QSpinBox* throttle_set_spin_;
#ifdef FOUR_WING
    QHBoxLayout* throttle_2_set_layout_;
    QLabel* throttle_2_set_front_label_;
    QSlider* throttle_2_set_slider_;
    QSpinBox* throttle_2_set_spin_;
#endif
#ifdef TWO_WING
    QHBoxLayout* climb_set_layout_;
    QLabel* climb_set_front_label_;
    QSlider* climb_set_slider_;
    QSpinBox* climb_set_spin_;
#endif

    //PWM 参数存储变量
    uint16_t mid_servo_pwm_set_;
    uint16_t climb_pwm_set_;
    uint16_t left_servo_pwm_set_;
    uint16_t right_servo_pwm_set_;
    uint16_t throttle_pwm_set_;
#ifdef FOUR_WING
    uint16_t throttle_2_pwm_set_;
#endif
    bool is_debugging_servo_throttle_;
    bool is_throttle_enabled_;
#ifdef FOUR_WING
    bool is_throttle_2_enabled_;
#endif

    //校准界面控件
    QVBoxLayout* sensor_calib_layout_;
    QHBoxLayout* sensor_calib_btn_layout_;
    QPushButton* mag_calib_start_btn_;
    QPushButton* mag_calib_clear_btn_;

    QLabel* mag_offset_front_label_;
    QHBoxLayout* mag_offset_layout_;
    QLabel* mag_offset_x_front_label_;
    QLineEdit* mag_offset_x_edit_;
    QLabel* mag_offset_y_front_label_;
    QLineEdit* mag_offset_y_edit_;
    QLabel* mag_offset_z_front_label_;
    QLineEdit* mag_offset_z_edit_;

    QLabel* mag_gain_front_label_;
    QHBoxLayout* mag_gain_layout_;
    QLabel* mag_gain_x_front_label_;
    QLineEdit* mag_gain_x_edit_;
    QLabel* mag_gain_y_front_label_;
    QLineEdit* mag_gain_y_edit_;
    QLabel* mag_gain_z_front_label_;
    QLineEdit* mag_gain_z_edit_;

    //校准变量
    double mag_offset_set_[3];  //0:x   1:y   2:z
    double mag_gain_set_[3];


    //调参模式的控件
    QVBoxLayout* pid_tuning_layout_;

    QHBoxLayout* pid_front_layout_;
    QLabel* pid_setvalue_front_label_;
    QLineEdit* pid_setvalue_edit_;
    QLabel* pid_freq_front_label_;
    QLineEdit* pid_freq_edit_;
    
    QLabel* pid_ext_front_label_;
    QHBoxLayout* pid_ext_layout_;
    QLabel* pid_ext_p_front_label_;
    QLineEdit* pid_ext_p_edit_;
    QLabel* pid_ext_i_front_label_;
    QLineEdit* pid_ext_i_edit_;
    QLabel* pid_ext_d_front_label_;
    QLineEdit* pid_ext_d_edit_;
    QHBoxLayout* pid_ext_limit_layout_;
    QLabel* pid_ext_uplimit_front_label_;
    QLineEdit* pid_ext_uplimit_edit_;
    QLabel* pid_ext_lowlimit_front_label_;
    QLineEdit* pid_ext_lowlimit_edit_;

    QLabel* pid_int_front_label_;
    QHBoxLayout* pid_int_layout_;
    QLabel* pid_int_p_front_label_;
    QLineEdit* pid_int_p_edit_;
    QLabel* pid_int_i_front_label_;
    QLineEdit* pid_int_i_edit_;
    QLabel* pid_int_d_front_label_;
    QLineEdit* pid_int_d_edit_;
    QHBoxLayout* pid_int_limit_layout_;
    QLabel* pid_int_uplimit_front_label_;
    QLineEdit* pid_int_uplimit_edit_;
    QLabel* pid_int_lowlimit_front_label_;
    QLineEdit* pid_int_lowlimit_edit_;

    QHBoxLayout* pid_id_layout_;
    QLabel* pid_id_set_front_label_;
    QButtonGroup* pid_id_btn_group_;
    QPushButton* pid_id_roll_;
    QPushButton* pid_id_pitch_;
    QPushButton* pid_id_yaw_;

    QHBoxLayout* pos_pid_id_layout_;
    QLabel* pos_pid_id_set_front_label_;
    QButtonGroup* pos_pid_id_btn_group_;
    QPushButton* pid_id_x_;
    QPushButton* pid_id_y_;
    QPushButton* pid_id_z_;

    //Vicon测试模式控件
    QVBoxLayout* vicon_test_layout_;
    QPushButton* vicon_topic_refresh_btn_;
    QComboBox* vicon_topic_combo_;
    QPushButton* vicon_control_btn_;
    QPushButton* vicon_view_btn_;

    //Debug模式控件
    QVBoxLayout* debug_layout_;
    QLabel* z_mode_label_;
    
    //飞行模式摇杆
    JoystickWidget* flight_control_joysitck_;

    //摇杆存储变量
    uint throttle_joystick_;
    uint pitch_joystick_, roll_joystick_, yaw_joystick_;


    //PID 参数存储变量
    uint8_t pid_freq_;
    float pid_setvalue_[3];
    float pid_ext_set_[3][3];
    float pid_int_set_[3][3];
    float pid_ext_uplimit_[3];
    float pid_ext_lowlimit_[3];
    float pid_int_uplimit_[3];
    float pid_int_lowlimit_[3];
    uint8_t pid_id_set_;        //0:yaw 1:pitch 2:roll

    float pid_vel_set_[3][3];   //0:x 1:y 2:z
    float pid_pos_set_[3][3];   //0:p 1:i 2:d
    float pid_vel_uplimit_[3];
    float pid_vel_lowlimit_[3];
    float pid_pos_uplimit_[3];
    float pid_pos_lowlimit_[3];
    uint8_t pos_pid_id_set_;

    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::Publisher mav_config_pub_;
    ros::Publisher mag_calib_vis_pub_;
    ros::Subscriber mav_down_sub_;
    ros::Subscriber joystick_sub_;
    ros::Subscriber vicon_sub_;
    tf::TransformBroadcaster tf_pub_;

    //ROS节点句柄（调试）
    ros::Publisher acc_vis_pub_;

    QTimer* joystick_send_timer_;
    QTimer* message_display_timer_;

    //子进程句柄
    // FILE* joystick_;

    bool is_connected;
    bool is_vicon_control_started;
    bool is_vicon_view_started;

    bool is_displaying_msg;
    std::queue<message_t> display_message_queue_;

    uint mav_down_msg_cnt_;
    uint mav_down_msg_cnt_prev_;

    // Mag Calibration
    bool is_calibrating_mag;
    uint mag_data_cnt_;
    Eigen::VectorXd mag_calib_data_[3];
    visualization_msgs::MarkerArray mag_calib_vis_msg_;
    visualization_msgs::Marker mag_calib_vis_pts_;
    visualization_msgs::Marker mag_calib_vis_ellipsoid_;
    static const uint MAG_CALIB_REQUIRE_CNT = 600;
    static const uint MAG_CALIB_POINT_SCALE = 200;

};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
