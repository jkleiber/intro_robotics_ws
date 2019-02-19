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

class collision {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.collision = null;
    }
    else {
      if (initObj.hasOwnProperty('collision')) {
        this.collision = initObj.collision
      }
      else {
        this.collision = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type collision
    // Serialize message field [collision]
    bufferOffset = _serializer.bool(obj.collision, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type collision
    let len;
    let data = new collision(null);
    // Deserialize message field [collision]
    data.collision = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reactive_robot/collision';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec9653804a13642f770edbe4a85843b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool collision
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new collision(null);
    if (msg.collision !== undefined) {
      resolved.collision = msg.collision;
    }
    else {
      resolved.collision = false
    }

    return resolved;
    }
};

module.exports = collision;
