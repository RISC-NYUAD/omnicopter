// Auto-generated. Do not edit!

// (in-package servo_control_node.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetAnglesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.right = null;
      this.left = null;
    }
    else {
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAnglesRequest
    // Serialize message field [right]
    bufferOffset = _serializer.int32(obj.right, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = _serializer.int32(obj.left, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAnglesRequest
    let len;
    let data = new SetAnglesRequest(null);
    // Deserialize message field [right]
    data.right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'servo_control_node/SetAnglesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56e15b137f05b503f692a3beaa4e3f7a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # srv/SetAngles.srv
    
    int32 right   # Angle for the right motor
    int32 left    # Angle for the left motor
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAnglesRequest(null);
    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0
    }

    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = 0
    }

    return resolved;
    }
};

class SetAnglesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAnglesResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAnglesResponse
    let len;
    let data = new SetAnglesResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'servo_control_node/SetAnglesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success  # Return success if the operation was successful
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAnglesResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetAnglesRequest,
  Response: SetAnglesResponse,
  md5sum() { return 'af6660be3609fe9894731472db7c883b'; },
  datatype() { return 'servo_control_node/SetAngles'; }
};
