// Auto-generated. Do not edit!

// (in-package common_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.throttle = null;
      this.braking = null;
      this.steering = null;
    }
    else {
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('braking')) {
        this.braking = initObj.braking
      }
      else {
        this.braking = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlRequest
    // Serialize message field [throttle]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.throttle, buffer, bufferOffset);
    // Serialize message field [braking]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.braking, buffer, bufferOffset);
    // Serialize message field [steering]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.steering, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlRequest
    let len;
    let data = new ControlRequest(null);
    // Deserialize message field [throttle]
    data.throttle = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [braking]
    data.braking = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [steering]
    data.steering = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'common_srvs/ControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8a968ff18098750e2487c37c9a29dba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32 throttle
    std_msgs/Float32 braking
    std_msgs/Float32 steering
    
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlRequest(null);
    if (msg.throttle !== undefined) {
      resolved.throttle = std_msgs.msg.Float32.Resolve(msg.throttle)
    }
    else {
      resolved.throttle = new std_msgs.msg.Float32()
    }

    if (msg.braking !== undefined) {
      resolved.braking = std_msgs.msg.Float32.Resolve(msg.braking)
    }
    else {
      resolved.braking = new std_msgs.msg.Float32()
    }

    if (msg.steering !== undefined) {
      resolved.steering = std_msgs.msg.Float32.Resolve(msg.steering)
    }
    else {
      resolved.steering = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

class ControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlResponse
    let len;
    let data = new ControlResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'common_srvs/ControlResponse';
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
    const resolved = new ControlResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: ControlRequest,
  Response: ControlResponse,
  md5sum() { return 'a8a968ff18098750e2487c37c9a29dba'; },
  datatype() { return 'common_srvs/Control'; }
};
