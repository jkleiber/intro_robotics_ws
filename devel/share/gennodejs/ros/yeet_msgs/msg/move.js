// Auto-generated. Do not edit!

// (in-package yeet_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class move {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drive = null;
      this.turn = null;
    }
    else {
      if (initObj.hasOwnProperty('drive')) {
        this.drive = initObj.drive
      }
      else {
        this.drive = 0;
      }
      if (initObj.hasOwnProperty('turn')) {
        this.turn = initObj.turn
      }
      else {
        this.turn = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type move
    // Serialize message field [drive]
    bufferOffset = _serializer.int8(obj.drive, buffer, bufferOffset);
    // Serialize message field [turn]
    bufferOffset = _serializer.int8(obj.turn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type move
    let len;
    let data = new move(null);
    // Deserialize message field [drive]
    data.drive = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [turn]
    data.turn = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yeet_msgs/move';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f1cddb8ef3caea21484489b8f1096b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 drive
    int8 turn
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new move(null);
    if (msg.drive !== undefined) {
      resolved.drive = msg.drive;
    }
    else {
      resolved.drive = 0
    }

    if (msg.turn !== undefined) {
      resolved.turn = msg.turn;
    }
    else {
      resolved.turn = 0
    }

    return resolved;
    }
};

module.exports = move;
