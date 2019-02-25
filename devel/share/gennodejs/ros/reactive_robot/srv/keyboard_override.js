// Auto-generated. Do not edit!

// (in-package reactive_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class keyboard_overrideRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type keyboard_overrideRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type keyboard_overrideRequest
    let len;
    let data = new keyboard_overrideRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reactive_robot/keyboard_overrideRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new keyboard_overrideRequest(null);
    return resolved;
    }
};

class keyboard_overrideResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.overridden = null;
    }
    else {
      if (initObj.hasOwnProperty('overridden')) {
        this.overridden = initObj.overridden
      }
      else {
        this.overridden = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type keyboard_overrideResponse
    // Serialize message field [overridden]
    bufferOffset = _serializer.bool(obj.overridden, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type keyboard_overrideResponse
    let len;
    let data = new keyboard_overrideResponse(null);
    // Deserialize message field [overridden]
    data.overridden = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'reactive_robot/keyboard_overrideResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb5aa041e0dc2943641ab2f5d3442948';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool overridden
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new keyboard_overrideResponse(null);
    if (msg.overridden !== undefined) {
      resolved.overridden = msg.overridden;
    }
    else {
      resolved.overridden = false
    }

    return resolved;
    }
};

module.exports = {
  Request: keyboard_overrideRequest,
  Response: keyboard_overrideResponse,
  md5sum() { return 'cb5aa041e0dc2943641ab2f5d3442948'; },
  datatype() { return 'reactive_robot/keyboard_override'; }
};
