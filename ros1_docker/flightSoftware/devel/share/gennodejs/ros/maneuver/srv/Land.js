// Auto-generated. Do not edit!

// (in-package maneuver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class LandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.height_1 = null;
      this.duration_1 = null;
      this.height_2 = null;
      this.duration_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('height_1')) {
        this.height_1 = initObj.height_1
      }
      else {
        this.height_1 = 0.0;
      }
      if (initObj.hasOwnProperty('duration_1')) {
        this.duration_1 = initObj.duration_1
      }
      else {
        this.duration_1 = 0.0;
      }
      if (initObj.hasOwnProperty('height_2')) {
        this.height_2 = initObj.height_2
      }
      else {
        this.height_2 = 0.0;
      }
      if (initObj.hasOwnProperty('duration_2')) {
        this.duration_2 = initObj.duration_2
      }
      else {
        this.duration_2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LandRequest
    // Serialize message field [height_1]
    bufferOffset = _serializer.float32(obj.height_1, buffer, bufferOffset);
    // Serialize message field [duration_1]
    bufferOffset = _serializer.float32(obj.duration_1, buffer, bufferOffset);
    // Serialize message field [height_2]
    bufferOffset = _serializer.float32(obj.height_2, buffer, bufferOffset);
    // Serialize message field [duration_2]
    bufferOffset = _serializer.float32(obj.duration_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LandRequest
    let len;
    let data = new LandRequest(null);
    // Deserialize message field [height_1]
    data.height_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration_1]
    data.duration_1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height_2]
    data.height_2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration_2]
    data.duration_2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/LandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32263ce3c47b5e10a67905c677fd1816';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 height_1
    float32 duration_1
    float32 height_2
    float32 duration_2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LandRequest(null);
    if (msg.height_1 !== undefined) {
      resolved.height_1 = msg.height_1;
    }
    else {
      resolved.height_1 = 0.0
    }

    if (msg.duration_1 !== undefined) {
      resolved.duration_1 = msg.duration_1;
    }
    else {
      resolved.duration_1 = 0.0
    }

    if (msg.height_2 !== undefined) {
      resolved.height_2 = msg.height_2;
    }
    else {
      resolved.height_2 = 0.0
    }

    if (msg.duration_2 !== undefined) {
      resolved.duration_2 = msg.duration_2;
    }
    else {
      resolved.duration_2 = 0.0
    }

    return resolved;
    }
};

class LandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LandResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LandResponse
    let len;
    let data = new LandResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/LandResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LandResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: LandRequest,
  Response: LandResponse,
  md5sum() { return '6caadcbcd5d54204c00f1a33593265c7'; },
  datatype() { return 'maneuver/Land'; }
};
