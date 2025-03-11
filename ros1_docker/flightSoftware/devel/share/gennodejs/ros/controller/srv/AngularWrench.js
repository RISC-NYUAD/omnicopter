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

class AngularWrenchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fz = null;
      this.phi1 = null;
      this.phi2 = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('fz')) {
        this.fz = initObj.fz
      }
      else {
        this.fz = 0.0;
      }
      if (initObj.hasOwnProperty('phi1')) {
        this.phi1 = initObj.phi1
      }
      else {
        this.phi1 = 0.0;
      }
      if (initObj.hasOwnProperty('phi2')) {
        this.phi2 = initObj.phi2
      }
      else {
        this.phi2 = 0.0;
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
    // Serializes a message object of type AngularWrenchRequest
    // Serialize message field [fz]
    bufferOffset = _serializer.float32(obj.fz, buffer, bufferOffset);
    // Serialize message field [phi1]
    bufferOffset = _serializer.float32(obj.phi1, buffer, bufferOffset);
    // Serialize message field [phi2]
    bufferOffset = _serializer.float32(obj.phi2, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AngularWrenchRequest
    let len;
    let data = new AngularWrenchRequest(null);
    // Deserialize message field [fz]
    data.fz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [phi1]
    data.phi1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [phi2]
    data.phi2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/AngularWrenchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc9e7d68423879ad0d9a0703c727a1f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 fz
    float32 phi1
    float32 phi2
    float32 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AngularWrenchRequest(null);
    if (msg.fz !== undefined) {
      resolved.fz = msg.fz;
    }
    else {
      resolved.fz = 0.0
    }

    if (msg.phi1 !== undefined) {
      resolved.phi1 = msg.phi1;
    }
    else {
      resolved.phi1 = 0.0
    }

    if (msg.phi2 !== undefined) {
      resolved.phi2 = msg.phi2;
    }
    else {
      resolved.phi2 = 0.0
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

class AngularWrenchResponse {
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
    // Serializes a message object of type AngularWrenchResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AngularWrenchResponse
    let len;
    let data = new AngularWrenchResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/AngularWrenchResponse';
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
    const resolved = new AngularWrenchResponse(null);
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
  Request: AngularWrenchRequest,
  Response: AngularWrenchResponse,
  md5sum() { return 'bfbfde29636da2ef0700941fae523d7c'; },
  datatype() { return 'controller/AngularWrench'; }
};
