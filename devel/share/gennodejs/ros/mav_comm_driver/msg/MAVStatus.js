// Auto-generated. Do not edit!

// (in-package mav_comm_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MAVStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mode_id = null;
      this.imu_data = null;
      this.odom = null;
      this.mid_servo_pwm = null;
      this.left_servo_pwm = null;
      this.right_servo_pwm = null;
      this.throttle_pwm = null;
      this.climb_pwm = null;
      this.sys_status = null;
      this.yaw_angle = null;
      this.pitch_angle = null;
      this.roll_angle = null;
      this.yaw_rate = null;
      this.pitch_rate = null;
      this.roll_rate = null;
      this.board_time = null;
      this.pid_id = null;
      this.pid_ext_err = null;
      this.pid_int_err = null;
      this.ext_p_output = null;
      this.ext_i_output = null;
      this.ext_d_output = null;
      this.int_p_output = null;
      this.int_i_output = null;
      this.int_d_output = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('mode_id')) {
        this.mode_id = initObj.mode_id
      }
      else {
        this.mode_id = 0;
      }
      if (initObj.hasOwnProperty('imu_data')) {
        this.imu_data = initObj.imu_data
      }
      else {
        this.imu_data = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('odom')) {
        this.odom = initObj.odom
      }
      else {
        this.odom = new nav_msgs.msg.Odometry();
      }
      if (initObj.hasOwnProperty('mid_servo_pwm')) {
        this.mid_servo_pwm = initObj.mid_servo_pwm
      }
      else {
        this.mid_servo_pwm = 0;
      }
      if (initObj.hasOwnProperty('left_servo_pwm')) {
        this.left_servo_pwm = initObj.left_servo_pwm
      }
      else {
        this.left_servo_pwm = 0;
      }
      if (initObj.hasOwnProperty('right_servo_pwm')) {
        this.right_servo_pwm = initObj.right_servo_pwm
      }
      else {
        this.right_servo_pwm = 0;
      }
      if (initObj.hasOwnProperty('throttle_pwm')) {
        this.throttle_pwm = initObj.throttle_pwm
      }
      else {
        this.throttle_pwm = 0;
      }
      if (initObj.hasOwnProperty('climb_pwm')) {
        this.climb_pwm = initObj.climb_pwm
      }
      else {
        this.climb_pwm = 0;
      }
      if (initObj.hasOwnProperty('sys_status')) {
        this.sys_status = initObj.sys_status
      }
      else {
        this.sys_status = 0;
      }
      if (initObj.hasOwnProperty('yaw_angle')) {
        this.yaw_angle = initObj.yaw_angle
      }
      else {
        this.yaw_angle = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_angle')) {
        this.pitch_angle = initObj.pitch_angle
      }
      else {
        this.pitch_angle = 0.0;
      }
      if (initObj.hasOwnProperty('roll_angle')) {
        this.roll_angle = initObj.roll_angle
      }
      else {
        this.roll_angle = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_rate')) {
        this.pitch_rate = initObj.pitch_rate
      }
      else {
        this.pitch_rate = 0.0;
      }
      if (initObj.hasOwnProperty('roll_rate')) {
        this.roll_rate = initObj.roll_rate
      }
      else {
        this.roll_rate = 0.0;
      }
      if (initObj.hasOwnProperty('board_time')) {
        this.board_time = initObj.board_time
      }
      else {
        this.board_time = 0;
      }
      if (initObj.hasOwnProperty('pid_id')) {
        this.pid_id = initObj.pid_id
      }
      else {
        this.pid_id = 0;
      }
      if (initObj.hasOwnProperty('pid_ext_err')) {
        this.pid_ext_err = initObj.pid_ext_err
      }
      else {
        this.pid_ext_err = 0.0;
      }
      if (initObj.hasOwnProperty('pid_int_err')) {
        this.pid_int_err = initObj.pid_int_err
      }
      else {
        this.pid_int_err = 0.0;
      }
      if (initObj.hasOwnProperty('ext_p_output')) {
        this.ext_p_output = initObj.ext_p_output
      }
      else {
        this.ext_p_output = 0.0;
      }
      if (initObj.hasOwnProperty('ext_i_output')) {
        this.ext_i_output = initObj.ext_i_output
      }
      else {
        this.ext_i_output = 0.0;
      }
      if (initObj.hasOwnProperty('ext_d_output')) {
        this.ext_d_output = initObj.ext_d_output
      }
      else {
        this.ext_d_output = 0.0;
      }
      if (initObj.hasOwnProperty('int_p_output')) {
        this.int_p_output = initObj.int_p_output
      }
      else {
        this.int_p_output = 0.0;
      }
      if (initObj.hasOwnProperty('int_i_output')) {
        this.int_i_output = initObj.int_i_output
      }
      else {
        this.int_i_output = 0.0;
      }
      if (initObj.hasOwnProperty('int_d_output')) {
        this.int_d_output = initObj.int_d_output
      }
      else {
        this.int_d_output = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MAVStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mode_id]
    bufferOffset = _serializer.uint8(obj.mode_id, buffer, bufferOffset);
    // Serialize message field [imu_data]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu_data, buffer, bufferOffset);
    // Serialize message field [odom]
    bufferOffset = nav_msgs.msg.Odometry.serialize(obj.odom, buffer, bufferOffset);
    // Serialize message field [mid_servo_pwm]
    bufferOffset = _serializer.uint8(obj.mid_servo_pwm, buffer, bufferOffset);
    // Serialize message field [left_servo_pwm]
    bufferOffset = _serializer.uint8(obj.left_servo_pwm, buffer, bufferOffset);
    // Serialize message field [right_servo_pwm]
    bufferOffset = _serializer.uint8(obj.right_servo_pwm, buffer, bufferOffset);
    // Serialize message field [throttle_pwm]
    bufferOffset = _serializer.uint8(obj.throttle_pwm, buffer, bufferOffset);
    // Serialize message field [climb_pwm]
    bufferOffset = _serializer.uint8(obj.climb_pwm, buffer, bufferOffset);
    // Serialize message field [sys_status]
    bufferOffset = _serializer.uint8(obj.sys_status, buffer, bufferOffset);
    // Serialize message field [yaw_angle]
    bufferOffset = _serializer.float32(obj.yaw_angle, buffer, bufferOffset);
    // Serialize message field [pitch_angle]
    bufferOffset = _serializer.float32(obj.pitch_angle, buffer, bufferOffset);
    // Serialize message field [roll_angle]
    bufferOffset = _serializer.float32(obj.roll_angle, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
    // Serialize message field [pitch_rate]
    bufferOffset = _serializer.float32(obj.pitch_rate, buffer, bufferOffset);
    // Serialize message field [roll_rate]
    bufferOffset = _serializer.float32(obj.roll_rate, buffer, bufferOffset);
    // Serialize message field [board_time]
    bufferOffset = _serializer.uint32(obj.board_time, buffer, bufferOffset);
    // Serialize message field [pid_id]
    bufferOffset = _serializer.uint8(obj.pid_id, buffer, bufferOffset);
    // Serialize message field [pid_ext_err]
    bufferOffset = _serializer.float32(obj.pid_ext_err, buffer, bufferOffset);
    // Serialize message field [pid_int_err]
    bufferOffset = _serializer.float32(obj.pid_int_err, buffer, bufferOffset);
    // Serialize message field [ext_p_output]
    bufferOffset = _serializer.float32(obj.ext_p_output, buffer, bufferOffset);
    // Serialize message field [ext_i_output]
    bufferOffset = _serializer.float32(obj.ext_i_output, buffer, bufferOffset);
    // Serialize message field [ext_d_output]
    bufferOffset = _serializer.float32(obj.ext_d_output, buffer, bufferOffset);
    // Serialize message field [int_p_output]
    bufferOffset = _serializer.float32(obj.int_p_output, buffer, bufferOffset);
    // Serialize message field [int_i_output]
    bufferOffset = _serializer.float32(obj.int_i_output, buffer, bufferOffset);
    // Serialize message field [int_d_output]
    bufferOffset = _serializer.float32(obj.int_d_output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MAVStatus
    let len;
    let data = new MAVStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode_id]
    data.mode_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [imu_data]
    data.imu_data = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [odom]
    data.odom = nav_msgs.msg.Odometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [mid_servo_pwm]
    data.mid_servo_pwm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [left_servo_pwm]
    data.left_servo_pwm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [right_servo_pwm]
    data.right_servo_pwm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [throttle_pwm]
    data.throttle_pwm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [climb_pwm]
    data.climb_pwm = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sys_status]
    data.sys_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [yaw_angle]
    data.yaw_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_angle]
    data.pitch_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_angle]
    data.roll_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_rate]
    data.pitch_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_rate]
    data.roll_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [board_time]
    data.board_time = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pid_id]
    data.pid_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pid_ext_err]
    data.pid_ext_err = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pid_int_err]
    data.pid_int_err = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ext_p_output]
    data.ext_p_output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ext_i_output]
    data.ext_i_output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ext_d_output]
    data.ext_d_output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [int_p_output]
    data.int_p_output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [int_i_output]
    data.int_i_output = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [int_d_output]
    data.int_d_output = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu_data);
    length += nav_msgs.msg.Odometry.getMessageSize(object.odom);
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mav_comm_driver/MAVStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b02f3350f8be0b668bda0c83dc2bc899';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 mode_id
    
    # formatted sensor data
    sensor_msgs/Imu imu_data
    nav_msgs/Odometry odom
    
    # PWM output value
    uint8 mid_servo_pwm
    uint8 left_servo_pwm
    uint8 right_servo_pwm
    uint8 throttle_pwm
    uint8 climb_pwm
    
    # system status
    uint8 sys_status
    
    # origin data
    float32 yaw_angle
    float32 pitch_angle
    float32 roll_angle
    float32 yaw_rate
    float32 pitch_rate
    float32 roll_rate
    uint32 board_time # us
    
    # tuning mode value
    uint8 pid_id
    float32 pid_ext_err # degree * 100
    float32 pid_int_err # degree / s * 10
    float32 ext_p_output
    float32 ext_i_output
    float32 ext_d_output
    float32 int_p_output
    float32 int_i_output
    float32 int_d_output
    
    
    # modes
    uint8 FAULT_MODE = 0 # 0x00
    uint8 START_MODE = 8 # 0x08
    uint8 MANUAL_MODE = 16 # 0x10
    uint8 FLIGHT_MODE = 24 # 0x18
    uint8 TUNING_MODE = 56 # 0x38
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: nav_msgs/Odometry
    # This represents an estimate of a position and velocity in free space.  
    # The pose in this message should be specified in the coordinate frame given by header.frame_id.
    # The twist in this message should be specified in the coordinate frame given by the child_frame_id
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MAVStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mode_id !== undefined) {
      resolved.mode_id = msg.mode_id;
    }
    else {
      resolved.mode_id = 0
    }

    if (msg.imu_data !== undefined) {
      resolved.imu_data = sensor_msgs.msg.Imu.Resolve(msg.imu_data)
    }
    else {
      resolved.imu_data = new sensor_msgs.msg.Imu()
    }

    if (msg.odom !== undefined) {
      resolved.odom = nav_msgs.msg.Odometry.Resolve(msg.odom)
    }
    else {
      resolved.odom = new nav_msgs.msg.Odometry()
    }

    if (msg.mid_servo_pwm !== undefined) {
      resolved.mid_servo_pwm = msg.mid_servo_pwm;
    }
    else {
      resolved.mid_servo_pwm = 0
    }

    if (msg.left_servo_pwm !== undefined) {
      resolved.left_servo_pwm = msg.left_servo_pwm;
    }
    else {
      resolved.left_servo_pwm = 0
    }

    if (msg.right_servo_pwm !== undefined) {
      resolved.right_servo_pwm = msg.right_servo_pwm;
    }
    else {
      resolved.right_servo_pwm = 0
    }

    if (msg.throttle_pwm !== undefined) {
      resolved.throttle_pwm = msg.throttle_pwm;
    }
    else {
      resolved.throttle_pwm = 0
    }

    if (msg.climb_pwm !== undefined) {
      resolved.climb_pwm = msg.climb_pwm;
    }
    else {
      resolved.climb_pwm = 0
    }

    if (msg.sys_status !== undefined) {
      resolved.sys_status = msg.sys_status;
    }
    else {
      resolved.sys_status = 0
    }

    if (msg.yaw_angle !== undefined) {
      resolved.yaw_angle = msg.yaw_angle;
    }
    else {
      resolved.yaw_angle = 0.0
    }

    if (msg.pitch_angle !== undefined) {
      resolved.pitch_angle = msg.pitch_angle;
    }
    else {
      resolved.pitch_angle = 0.0
    }

    if (msg.roll_angle !== undefined) {
      resolved.roll_angle = msg.roll_angle;
    }
    else {
      resolved.roll_angle = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    if (msg.pitch_rate !== undefined) {
      resolved.pitch_rate = msg.pitch_rate;
    }
    else {
      resolved.pitch_rate = 0.0
    }

    if (msg.roll_rate !== undefined) {
      resolved.roll_rate = msg.roll_rate;
    }
    else {
      resolved.roll_rate = 0.0
    }

    if (msg.board_time !== undefined) {
      resolved.board_time = msg.board_time;
    }
    else {
      resolved.board_time = 0
    }

    if (msg.pid_id !== undefined) {
      resolved.pid_id = msg.pid_id;
    }
    else {
      resolved.pid_id = 0
    }

    if (msg.pid_ext_err !== undefined) {
      resolved.pid_ext_err = msg.pid_ext_err;
    }
    else {
      resolved.pid_ext_err = 0.0
    }

    if (msg.pid_int_err !== undefined) {
      resolved.pid_int_err = msg.pid_int_err;
    }
    else {
      resolved.pid_int_err = 0.0
    }

    if (msg.ext_p_output !== undefined) {
      resolved.ext_p_output = msg.ext_p_output;
    }
    else {
      resolved.ext_p_output = 0.0
    }

    if (msg.ext_i_output !== undefined) {
      resolved.ext_i_output = msg.ext_i_output;
    }
    else {
      resolved.ext_i_output = 0.0
    }

    if (msg.ext_d_output !== undefined) {
      resolved.ext_d_output = msg.ext_d_output;
    }
    else {
      resolved.ext_d_output = 0.0
    }

    if (msg.int_p_output !== undefined) {
      resolved.int_p_output = msg.int_p_output;
    }
    else {
      resolved.int_p_output = 0.0
    }

    if (msg.int_i_output !== undefined) {
      resolved.int_i_output = msg.int_i_output;
    }
    else {
      resolved.int_i_output = 0.0
    }

    if (msg.int_d_output !== undefined) {
      resolved.int_d_output = msg.int_d_output;
    }
    else {
      resolved.int_d_output = 0.0
    }

    return resolved;
    }
};

// Constants for message
MAVStatus.Constants = {
  FAULT_MODE: 0,
  START_MODE: 8,
  MANUAL_MODE: 16,
  FLIGHT_MODE: 24,
  TUNING_MODE: 56,
}

module.exports = MAVStatus;
