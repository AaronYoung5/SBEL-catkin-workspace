// Auto-generated. Do not edit!

// (in-package common_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Cone = require('./Cone.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ConeMap {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.blue_cones = null;
      this.yellow_cones = null;
      this.orange_cones = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('blue_cones')) {
        this.blue_cones = initObj.blue_cones
      }
      else {
        this.blue_cones = [];
      }
      if (initObj.hasOwnProperty('yellow_cones')) {
        this.yellow_cones = initObj.yellow_cones
      }
      else {
        this.yellow_cones = [];
      }
      if (initObj.hasOwnProperty('orange_cones')) {
        this.orange_cones = initObj.orange_cones
      }
      else {
        this.orange_cones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConeMap
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [blue_cones]
    // Serialize the length for message field [blue_cones]
    bufferOffset = _serializer.uint32(obj.blue_cones.length, buffer, bufferOffset);
    obj.blue_cones.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yellow_cones]
    // Serialize the length for message field [yellow_cones]
    bufferOffset = _serializer.uint32(obj.yellow_cones.length, buffer, bufferOffset);
    obj.yellow_cones.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [orange_cones]
    // Serialize the length for message field [orange_cones]
    bufferOffset = _serializer.uint32(obj.orange_cones.length, buffer, bufferOffset);
    obj.orange_cones.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConeMap
    let len;
    let data = new ConeMap(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [blue_cones]
    // Deserialize array length for message field [blue_cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.blue_cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.blue_cones[i] = Cone.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yellow_cones]
    // Deserialize array length for message field [yellow_cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.yellow_cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.yellow_cones[i] = Cone.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [orange_cones]
    // Deserialize array length for message field [orange_cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.orange_cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.orange_cones[i] = Cone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 25 * object.blue_cones.length;
    length += 25 * object.yellow_cones.length;
    length += 25 * object.orange_cones.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'common_msgs/ConeMap';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a5312a83690b1beb889fc4f459cf44b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    common_msgs/Cone[] blue_cones
    common_msgs/Cone[] yellow_cones
    common_msgs/Cone[] orange_cones
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: common_msgs/Cone
    uint8 BLUE=0
    uint8 YELLOW=1
    uint8 ORANGE=2
    
    geometry_msgs/Point position
    uint8 color
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new ConeMap(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.blue_cones !== undefined) {
      resolved.blue_cones = new Array(msg.blue_cones.length);
      for (let i = 0; i < resolved.blue_cones.length; ++i) {
        resolved.blue_cones[i] = Cone.Resolve(msg.blue_cones[i]);
      }
    }
    else {
      resolved.blue_cones = []
    }

    if (msg.yellow_cones !== undefined) {
      resolved.yellow_cones = new Array(msg.yellow_cones.length);
      for (let i = 0; i < resolved.yellow_cones.length; ++i) {
        resolved.yellow_cones[i] = Cone.Resolve(msg.yellow_cones[i]);
      }
    }
    else {
      resolved.yellow_cones = []
    }

    if (msg.orange_cones !== undefined) {
      resolved.orange_cones = new Array(msg.orange_cones.length);
      for (let i = 0; i < resolved.orange_cones.length; ++i) {
        resolved.orange_cones[i] = Cone.Resolve(msg.orange_cones[i]);
      }
    }
    else {
      resolved.orange_cones = []
    }

    return resolved;
    }
};

module.exports = ConeMap;
