// Auto-generated. Do not edit!

// (in-package reactive_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class autodrive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drive = null;
      this.turn_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('drive')) {
        this.drive = initObj.drive
      }
      else {
        this.drive = false;
      }
      if (initObj.hasOwnProperty('turn_angle')) {
        this.turn_angle = initObj.turn_angle
      }
      else {
        this.turn_angle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type autodrive
    // Serialize message field [drive]
    bufferOffset = _serializer.bool(obj.drive, buffer, bufferOffset);
    // Serialize message field [turn_angle]
    bufferOffset = _serializer.int8(obj.turn_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type autodrive
    let len;
    let data = new autodrive(null);
    // Deserialize message field [drive]
    data.drive = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [turn_angle]
    data.turn_angle = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reactive_robot/autodrive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad06341ef70ac04503af948ffea2ec63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool drive
    int8 turn_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new autodrive(null);
    if (msg.drive !== undefined) {
      resolved.drive = msg.drive;
    }
    else {
      resolved.drive = false
    }

    if (msg.turn_angle !== undefined) {
      resolved.turn_angle = msg.turn_angle;
    }
    else {
      resolved.turn_angle = 0
    }

    return resolved;
    }
};

module.exports = autodrive;
