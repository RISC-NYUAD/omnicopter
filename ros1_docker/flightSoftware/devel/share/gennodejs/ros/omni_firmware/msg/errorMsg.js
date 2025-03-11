// Auto-generated. Do not edit!

// (in-package omni_firmware.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class errorMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ex = null;
      this.ev = null;
      this.ea = null;
      this.eR = null;
      this.ew = null;
      this.Iex = null;
      this.Ier = null;
      this.Accd = null;
      this.Wrench = null;
      this.exWrench = null;
      this.prop = null;
      this.weight = null;
    }
    else {
      if (initObj.hasOwnProperty('ex')) {
        this.ex = initObj.ex
      }
      else {
        this.ex = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('ev')) {
        this.ev = initObj.ev
      }
      else {
        this.ev = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('ea')) {
        this.ea = initObj.ea
      }
      else {
        this.ea = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('eR')) {
        this.eR = initObj.eR
      }
      else {
        this.eR = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('ew')) {
        this.ew = initObj.ew
      }
      else {
        this.ew = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Iex')) {
        this.Iex = initObj.Iex
      }
      else {
        this.Iex = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Ier')) {
        this.Ier = initObj.Ier
      }
      else {
        this.Ier = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Accd')) {
        this.Accd = initObj.Accd
      }
      else {
        this.Accd = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Wrench')) {
        this.Wrench = initObj.Wrench
      }
      else {
        this.Wrench = [];
      }
      if (initObj.hasOwnProperty('exWrench')) {
        this.exWrench = initObj.exWrench
      }
      else {
        this.exWrench = [];
      }
      if (initObj.hasOwnProperty('prop')) {
        this.prop = initObj.prop
      }
      else {
        this.prop = [];
      }
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type errorMsg
    // Serialize message field [ex]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ex, buffer, bufferOffset);
    // Serialize message field [ev]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ev, buffer, bufferOffset);
    // Serialize message field [ea]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ea, buffer, bufferOffset);
    // Serialize message field [eR]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.eR, buffer, bufferOffset);
    // Serialize message field [ew]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ew, buffer, bufferOffset);
    // Serialize message field [Iex]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Iex, buffer, bufferOffset);
    // Serialize message field [Ier]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Ier, buffer, bufferOffset);
    // Serialize message field [Accd]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Accd, buffer, bufferOffset);
    // Serialize message field [Wrench]
    bufferOffset = _arraySerializer.float32(obj.Wrench, buffer, bufferOffset, null);
    // Serialize message field [exWrench]
    bufferOffset = _arraySerializer.float32(obj.exWrench, buffer, bufferOffset, null);
    // Serialize message field [prop]
    bufferOffset = _arraySerializer.float32(obj.prop, buffer, bufferOffset, null);
    // Serialize message field [weight]
    bufferOffset = _serializer.float32(obj.weight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type errorMsg
    let len;
    let data = new errorMsg(null);
    // Deserialize message field [ex]
    data.ex = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [ev]
    data.ev = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [ea]
    data.ea = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [eR]
    data.eR = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [ew]
    data.ew = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Iex]
    data.Iex = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Ier]
    data.Ier = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Accd]
    data.Accd = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Wrench]
    data.Wrench = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [exWrench]
    data.exWrench = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [prop]
    data.prop = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [weight]
    data.weight = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.Wrench.length;
    length += 4 * object.exWrench.length;
    length += 4 * object.prop.length;
    return length + 208;
  }

  static datatype() {
    // Returns string type for a message object
    return 'omni_firmware/errorMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aca7c5154e5df4742f8034562501f6c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3 ex
    geometry_msgs/Vector3 ev
    geometry_msgs/Vector3 ea
    geometry_msgs/Vector3 eR
    geometry_msgs/Vector3 ew
    geometry_msgs/Vector3 Iex
    geometry_msgs/Vector3 Ier
    geometry_msgs/Vector3 Accd
    float32[] Wrench
    float32[] exWrench
    float32[] prop
    float32 weight
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new errorMsg(null);
    if (msg.ex !== undefined) {
      resolved.ex = geometry_msgs.msg.Vector3.Resolve(msg.ex)
    }
    else {
      resolved.ex = new geometry_msgs.msg.Vector3()
    }

    if (msg.ev !== undefined) {
      resolved.ev = geometry_msgs.msg.Vector3.Resolve(msg.ev)
    }
    else {
      resolved.ev = new geometry_msgs.msg.Vector3()
    }

    if (msg.ea !== undefined) {
      resolved.ea = geometry_msgs.msg.Vector3.Resolve(msg.ea)
    }
    else {
      resolved.ea = new geometry_msgs.msg.Vector3()
    }

    if (msg.eR !== undefined) {
      resolved.eR = geometry_msgs.msg.Vector3.Resolve(msg.eR)
    }
    else {
      resolved.eR = new geometry_msgs.msg.Vector3()
    }

    if (msg.ew !== undefined) {
      resolved.ew = geometry_msgs.msg.Vector3.Resolve(msg.ew)
    }
    else {
      resolved.ew = new geometry_msgs.msg.Vector3()
    }

    if (msg.Iex !== undefined) {
      resolved.Iex = geometry_msgs.msg.Vector3.Resolve(msg.Iex)
    }
    else {
      resolved.Iex = new geometry_msgs.msg.Vector3()
    }

    if (msg.Ier !== undefined) {
      resolved.Ier = geometry_msgs.msg.Vector3.Resolve(msg.Ier)
    }
    else {
      resolved.Ier = new geometry_msgs.msg.Vector3()
    }

    if (msg.Accd !== undefined) {
      resolved.Accd = geometry_msgs.msg.Vector3.Resolve(msg.Accd)
    }
    else {
      resolved.Accd = new geometry_msgs.msg.Vector3()
    }

    if (msg.Wrench !== undefined) {
      resolved.Wrench = msg.Wrench;
    }
    else {
      resolved.Wrench = []
    }

    if (msg.exWrench !== undefined) {
      resolved.exWrench = msg.exWrench;
    }
    else {
      resolved.exWrench = []
    }

    if (msg.prop !== undefined) {
      resolved.prop = msg.prop;
    }
    else {
      resolved.prop = []
    }

    if (msg.weight !== undefined) {
      resolved.weight = msg.weight;
    }
    else {
      resolved.weight = 0.0
    }

    return resolved;
    }
};

module.exports = errorMsg;
