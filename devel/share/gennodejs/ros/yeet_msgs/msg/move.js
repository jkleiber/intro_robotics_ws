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
      this.todo = null;
    }
    else {
      if (initObj.hasOwnProperty('todo')) {
        this.todo = initObj.todo
      }
      else {
        this.todo = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type move
    // Serialize message field [todo]
    bufferOffset = _serializer.int8(obj.todo, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type move
    let len;
    let data = new move(null);
    // Deserialize message field [todo]
    data.todo = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yeet_msgs/move';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf55fd99d4d1be34562aaf532f8ee9a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 todo
    #test
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new move(null);
    if (msg.todo !== undefined) {
      resolved.todo = msg.todo;
    }
    else {
      resolved.todo = 0
    }

    return resolved;
    }
};

module.exports = move;
