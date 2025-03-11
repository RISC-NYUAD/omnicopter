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

class Uvector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Uvector
    // Check that the constant length array field [value] has the right length
    if (obj.value.length !== 12) {
      throw new Error('Unable to serialize array field value - length must be 12')
    }
    // Serialize message field [value]
    bufferOffset = _arraySerializer.float32(obj.value, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Uvector
    let len;
    let data = new Uvector(null);
    // Deserialize message field [value]
    data.value = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'omni_firmware/Uvector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3cac8a89b50e7d740ace6fafac0a09de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[12] value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Uvector(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = Uvector;
