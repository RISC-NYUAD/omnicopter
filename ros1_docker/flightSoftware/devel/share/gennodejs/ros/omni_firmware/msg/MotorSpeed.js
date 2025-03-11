// Auto-generated. Do not edit!

// (in-package omni_firmware.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorSpeed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorSpeed
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float32(obj.velocity, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorSpeed
    let len;
    let data = new MotorSpeed(null);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.name.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 4 * object.velocity.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'omni_firmware/MotorSpeed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1d54c3c8a939a204eb96629420d7ab7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] name
    float32[] velocity
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorSpeed(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    return resolved;
    }
};

module.exports = MotorSpeed;
