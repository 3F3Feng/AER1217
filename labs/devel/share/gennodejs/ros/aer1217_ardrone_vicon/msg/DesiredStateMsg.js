// Auto-generated. Do not edit!

// (in-package aer1217_ardrone_vicon.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DesiredStateMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.x_dot = null;
      this.yaw = null;
      this.yaw_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('x_dot')) {
        this.x_dot = initObj.x_dot
      }
      else {
        this.x_dot = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DesiredStateMsg
    // Check that the constant length array field [x] has the right length
    if (obj.x.length !== 3) {
      throw new Error('Unable to serialize array field x - length must be 3')
    }
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float64(obj.x, buffer, bufferOffset, 3);
    // Check that the constant length array field [x_dot] has the right length
    if (obj.x_dot.length !== 3) {
      throw new Error('Unable to serialize array field x_dot - length must be 3')
    }
    // Serialize message field [x_dot]
    bufferOffset = _arraySerializer.float64(obj.x_dot, buffer, bufferOffset, 3);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float64(obj.yaw_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DesiredStateMsg
    let len;
    let data = new DesiredStateMsg(null);
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [x_dot]
    data.x_dot = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aer1217_ardrone_vicon/DesiredStateMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc61550db4d69f891417c4905802aea9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MotorCommands
    #
    # Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017
    #
    # Data communicates the motor commands in RPM as a four element array in the
    # following order [front_left, front_right, rear_left, rear_right]
    
    float64[3] x
    float64[3] x_dot
    float64 yaw
    float64 yaw_rate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DesiredStateMsg(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = new Array(3).fill(0)
    }

    if (msg.x_dot !== undefined) {
      resolved.x_dot = msg.x_dot;
    }
    else {
      resolved.x_dot = new Array(3).fill(0)
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    return resolved;
    }
};

module.exports = DesiredStateMsg;
