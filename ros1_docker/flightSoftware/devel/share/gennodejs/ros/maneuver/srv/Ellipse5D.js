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

class Ellipse5DRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_min = null;
      this.x_max = null;
      this.y_min = null;
      this.y_max = null;
      this.z_min = null;
      this.z_max = null;
      this.roll_start = null;
      this.roll_mid = null;
      this.roll_end = null;
      this.pitch_start = null;
      this.pitch_mid = null;
      this.pitch_end = null;
      this.yaw = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('x_min')) {
        this.x_min = initObj.x_min
      }
      else {
        this.x_min = 0.0;
      }
      if (initObj.hasOwnProperty('x_max')) {
        this.x_max = initObj.x_max
      }
      else {
        this.x_max = 0.0;
      }
      if (initObj.hasOwnProperty('y_min')) {
        this.y_min = initObj.y_min
      }
      else {
        this.y_min = 0.0;
      }
      if (initObj.hasOwnProperty('y_max')) {
        this.y_max = initObj.y_max
      }
      else {
        this.y_max = 0.0;
      }
      if (initObj.hasOwnProperty('z_min')) {
        this.z_min = initObj.z_min
      }
      else {
        this.z_min = 0.0;
      }
      if (initObj.hasOwnProperty('z_max')) {
        this.z_max = initObj.z_max
      }
      else {
        this.z_max = 0.0;
      }
      if (initObj.hasOwnProperty('roll_start')) {
        this.roll_start = initObj.roll_start
      }
      else {
        this.roll_start = 0.0;
      }
      if (initObj.hasOwnProperty('roll_mid')) {
        this.roll_mid = initObj.roll_mid
      }
      else {
        this.roll_mid = 0.0;
      }
      if (initObj.hasOwnProperty('roll_end')) {
        this.roll_end = initObj.roll_end
      }
      else {
        this.roll_end = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_start')) {
        this.pitch_start = initObj.pitch_start
      }
      else {
        this.pitch_start = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_mid')) {
        this.pitch_mid = initObj.pitch_mid
      }
      else {
        this.pitch_mid = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_end')) {
        this.pitch_end = initObj.pitch_end
      }
      else {
        this.pitch_end = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
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
    // Serializes a message object of type Ellipse5DRequest
    // Serialize message field [x_min]
    bufferOffset = _serializer.float32(obj.x_min, buffer, bufferOffset);
    // Serialize message field [x_max]
    bufferOffset = _serializer.float32(obj.x_max, buffer, bufferOffset);
    // Serialize message field [y_min]
    bufferOffset = _serializer.float32(obj.y_min, buffer, bufferOffset);
    // Serialize message field [y_max]
    bufferOffset = _serializer.float32(obj.y_max, buffer, bufferOffset);
    // Serialize message field [z_min]
    bufferOffset = _serializer.float32(obj.z_min, buffer, bufferOffset);
    // Serialize message field [z_max]
    bufferOffset = _serializer.float32(obj.z_max, buffer, bufferOffset);
    // Serialize message field [roll_start]
    bufferOffset = _serializer.float32(obj.roll_start, buffer, bufferOffset);
    // Serialize message field [roll_mid]
    bufferOffset = _serializer.float32(obj.roll_mid, buffer, bufferOffset);
    // Serialize message field [roll_end]
    bufferOffset = _serializer.float32(obj.roll_end, buffer, bufferOffset);
    // Serialize message field [pitch_start]
    bufferOffset = _serializer.float32(obj.pitch_start, buffer, bufferOffset);
    // Serialize message field [pitch_mid]
    bufferOffset = _serializer.float32(obj.pitch_mid, buffer, bufferOffset);
    // Serialize message field [pitch_end]
    bufferOffset = _serializer.float32(obj.pitch_end, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ellipse5DRequest
    let len;
    let data = new Ellipse5DRequest(null);
    // Deserialize message field [x_min]
    data.x_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_max]
    data.x_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_min]
    data.y_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_max]
    data.y_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_min]
    data.z_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_max]
    data.z_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_start]
    data.roll_start = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_mid]
    data.roll_mid = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_end]
    data.roll_end = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_start]
    data.pitch_start = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_mid]
    data.pitch_mid = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_end]
    data.pitch_end = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/Ellipse5DRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de591e0b342372a5677c5ef03c671935';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x_min
    float32 x_max
    float32 y_min
    float32 y_max
    float32 z_min
    float32 z_max
    float32 roll_start
    float32 roll_mid
    float32 roll_end
    float32 pitch_start
    float32 pitch_mid
    float32 pitch_end
    float32 yaw
    float32 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ellipse5DRequest(null);
    if (msg.x_min !== undefined) {
      resolved.x_min = msg.x_min;
    }
    else {
      resolved.x_min = 0.0
    }

    if (msg.x_max !== undefined) {
      resolved.x_max = msg.x_max;
    }
    else {
      resolved.x_max = 0.0
    }

    if (msg.y_min !== undefined) {
      resolved.y_min = msg.y_min;
    }
    else {
      resolved.y_min = 0.0
    }

    if (msg.y_max !== undefined) {
      resolved.y_max = msg.y_max;
    }
    else {
      resolved.y_max = 0.0
    }

    if (msg.z_min !== undefined) {
      resolved.z_min = msg.z_min;
    }
    else {
      resolved.z_min = 0.0
    }

    if (msg.z_max !== undefined) {
      resolved.z_max = msg.z_max;
    }
    else {
      resolved.z_max = 0.0
    }

    if (msg.roll_start !== undefined) {
      resolved.roll_start = msg.roll_start;
    }
    else {
      resolved.roll_start = 0.0
    }

    if (msg.roll_mid !== undefined) {
      resolved.roll_mid = msg.roll_mid;
    }
    else {
      resolved.roll_mid = 0.0
    }

    if (msg.roll_end !== undefined) {
      resolved.roll_end = msg.roll_end;
    }
    else {
      resolved.roll_end = 0.0
    }

    if (msg.pitch_start !== undefined) {
      resolved.pitch_start = msg.pitch_start;
    }
    else {
      resolved.pitch_start = 0.0
    }

    if (msg.pitch_mid !== undefined) {
      resolved.pitch_mid = msg.pitch_mid;
    }
    else {
      resolved.pitch_mid = 0.0
    }

    if (msg.pitch_end !== undefined) {
      resolved.pitch_end = msg.pitch_end;
    }
    else {
      resolved.pitch_end = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
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

class Ellipse5DResponse {
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
    // Serializes a message object of type Ellipse5DResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ellipse5DResponse
    let len;
    let data = new Ellipse5DResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'maneuver/Ellipse5DResponse';
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
    const resolved = new Ellipse5DResponse(null);
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
  Request: Ellipse5DRequest,
  Response: Ellipse5DResponse,
  md5sum() { return 'bdb0ce274c1da7c8239ea3857b021bbb'; },
  datatype() { return 'maneuver/Ellipse5D'; }
};
