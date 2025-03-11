// Auto-generated. Do not edit!

// (in-package controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class LinearWrenchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fx1 = null;
      this.fy1 = null;
      this.fz1 = null;
      this.fx2 = null;
      this.fy2 = null;
      this.fz2 = null;
      this.ramp = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('fx1')) {
        this.fx1 = initObj.fx1
      }
      else {
        this.fx1 = 0.0;
      }
      if (initObj.hasOwnProperty('fy1')) {
        this.fy1 = initObj.fy1
      }
      else {
        this.fy1 = 0.0;
      }
      if (initObj.hasOwnProperty('fz1')) {
        this.fz1 = initObj.fz1
      }
      else {
        this.fz1 = 0.0;
      }
      if (initObj.hasOwnProperty('fx2')) {
        this.fx2 = initObj.fx2
      }
      else {
        this.fx2 = 0.0;
      }
      if (initObj.hasOwnProperty('fy2')) {
        this.fy2 = initObj.fy2
      }
      else {
        this.fy2 = 0.0;
      }
      if (initObj.hasOwnProperty('fz2')) {
        this.fz2 = initObj.fz2
      }
      else {
        this.fz2 = 0.0;
      }
      if (initObj.hasOwnProperty('ramp')) {
        this.ramp = initObj.ramp
      }
      else {
        this.ramp = 0.0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LinearWrenchRequest
    // Serialize message field [fx1]
    bufferOffset = _serializer.float32(obj.fx1, buffer, bufferOffset);
    // Serialize message field [fy1]
    bufferOffset = _serializer.float32(obj.fy1, buffer, bufferOffset);
    // Serialize message field [fz1]
    bufferOffset = _serializer.float32(obj.fz1, buffer, bufferOffset);
    // Serialize message field [fx2]
    bufferOffset = _serializer.float32(obj.fx2, buffer, bufferOffset);
    // Serialize message field [fy2]
    bufferOffset = _serializer.float32(obj.fy2, buffer, bufferOffset);
    // Serialize message field [fz2]
    bufferOffset = _serializer.float32(obj.fz2, buffer, bufferOffset);
    // Serialize message field [ramp]
    bufferOffset = _serializer.float32(obj.ramp, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LinearWrenchRequest
    let len;
    let data = new LinearWrenchRequest(null);
    // Deserialize message field [fx1]
    data.fx1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fy1]
    data.fy1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fz1]
    data.fz1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fx2]
    data.fx2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fy2]
    data.fy2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fz2]
    data.fz2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ramp]
    data.ramp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/LinearWrenchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c003c250150c4e771ba847e7b69d707d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 fx1
    float32 fy1
    float32 fz1
    float32 fx2
    float32 fy2
    float32 fz2
    float32 ramp
    float32 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LinearWrenchRequest(null);
    if (msg.fx1 !== undefined) {
      resolved.fx1 = msg.fx1;
    }
    else {
      resolved.fx1 = 0.0
    }

    if (msg.fy1 !== undefined) {
      resolved.fy1 = msg.fy1;
    }
    else {
      resolved.fy1 = 0.0
    }

    if (msg.fz1 !== undefined) {
      resolved.fz1 = msg.fz1;
    }
    else {
      resolved.fz1 = 0.0
    }

    if (msg.fx2 !== undefined) {
      resolved.fx2 = msg.fx2;
    }
    else {
      resolved.fx2 = 0.0
    }

    if (msg.fy2 !== undefined) {
      resolved.fy2 = msg.fy2;
    }
    else {
      resolved.fy2 = 0.0
    }

    if (msg.fz2 !== undefined) {
      resolved.fz2 = msg.fz2;
    }
    else {
      resolved.fz2 = 0.0
    }

    if (msg.ramp !== undefined) {
      resolved.ramp = msg.ramp;
    }
    else {
      resolved.ramp = 0.0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

class LinearWrenchResponse {
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
    // Serializes a message object of type LinearWrenchResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LinearWrenchResponse
    let len;
    let data = new LinearWrenchResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/LinearWrenchResponse';
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
    const resolved = new LinearWrenchResponse(null);
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
  Request: LinearWrenchRequest,
  Response: LinearWrenchResponse,
  md5sum() { return '676b2c4049e154bfe83fc0a28e11e13d'; },
  datatype() { return 'controller/LinearWrench'; }
};
