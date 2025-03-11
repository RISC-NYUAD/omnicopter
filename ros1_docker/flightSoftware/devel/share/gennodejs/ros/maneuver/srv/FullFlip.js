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

class FullFlipRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.roll_bool = null;
      this.pitch_bool = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('roll_bool')) {
        this.roll_bool = initObj.roll_bool
      }
      else {
        this.roll_bool = false;
      }
      if (initObj.hasOwnProperty('pitch_bool')) {
        this.pitch_bool = initObj.pitch_bool
      }
      else {
        this.pitch_bool = false;
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
    // Serializes a message object of type FullFlipRequest
    // Serialize message field [roll_bool]
    bufferOffset = _serializer.bool(obj.roll_bool, buffer, bufferOffset);
    // Serialize message field [pitch_bool]
    bufferOffset = _serializer.bool(obj.pitch_bool, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FullFlipRequest
    let len;
    let data = new FullFlipRequest(null);
    // Deserialize message field [roll_bool]
    data.roll_bool = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pitch_bool]
    data.pitch_bool = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/FullFlipRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a2170e847c498f1e2ea30d6f945867c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool roll_bool
    bool pitch_bool
    float32 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FullFlipRequest(null);
    if (msg.roll_bool !== undefined) {
      resolved.roll_bool = msg.roll_bool;
    }
    else {
      resolved.roll_bool = false
    }

    if (msg.pitch_bool !== undefined) {
      resolved.pitch_bool = msg.pitch_bool;
    }
    else {
      resolved.pitch_bool = false
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

class FullFlipResponse {
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
    // Serializes a message object of type FullFlipResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FullFlipResponse
    let len;
    let data = new FullFlipResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/FullFlipResponse';
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
    const resolved = new FullFlipResponse(null);
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
  Request: FullFlipRequest,
  Response: FullFlipResponse,
  md5sum() { return '432212f34386b2b3d536658964c298f8'; },
  datatype() { return 'maneuver/FullFlip'; }
};
