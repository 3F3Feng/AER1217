// Auto-generated. Do not edit!

// (in-package aer1217_ardrone_simulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor_cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('motor_cmd')) {
        this.motor_cmd = initObj.motor_cmd
      }
      else {
        this.motor_cmd = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCommands
    // Check that the constant length array field [motor_cmd] has the right length
    if (obj.motor_cmd.length !== 4) {
      throw new Error('Unable to serialize array field motor_cmd - length must be 4')
    }
    // Serialize message field [motor_cmd]
    bufferOffset = _arraySerializer.float64(obj.motor_cmd, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCommands
    let len;
    let data = new MotorCommands(null);
    // Deserialize message field [motor_cmd]
    data.motor_cmd = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aer1217_ardrone_simulator/MotorCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '424a0da2970783751aa94563cd70d0bb';
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
    
    float64[4] motor_cmd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCommands(null);
    if (msg.motor_cmd !== undefined) {
      resolved.motor_cmd = msg.motor_cmd;
    }
    else {
      resolved.motor_cmd = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = MotorCommands;
