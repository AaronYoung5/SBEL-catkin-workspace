// Auto-generated. Do not edit!

// (in-package chrono_ros_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Cones {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.blue_cones = null;
      this.yellow_cones = null;
      this.size = null;
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
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new std_msgs.msg.Int16();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cones
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [blue_cones]
    // Serialize the length for message field [blue_cones]
    bufferOffset = _serializer.uint32(obj.blue_cones.length, buffer, bufferOffset);
    obj.blue_cones.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yellow_cones]
    // Serialize the length for message field [yellow_cones]
    bufferOffset = _serializer.uint32(obj.yellow_cones.length, buffer, bufferOffset);
    obj.yellow_cones.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [size]
    bufferOffset = std_msgs.msg.Int16.serialize(obj.size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cones
    let len;
    let data = new Cones(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [blue_cones]
    // Deserialize array length for message field [blue_cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.blue_cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.blue_cones[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yellow_cones]
    // Deserialize array length for message field [yellow_cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.yellow_cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.yellow_cones[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [size]
    data.size = std_msgs.msg.Int16.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.blue_cones.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.yellow_cones.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chrono_ros_interface/Cones';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66302a9cec1053e5e1d2dec8fe6c53d1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Similar to nav_msgs/Path.msg, but has two paths (blue and yellow)
    # Positions are relative to the vehicle, not global
    Header header
    geometry_msgs/PoseStamped[] blue_cones
    geometry_msgs/PoseStamped[] yellow_cones
    std_msgs/Int16 size
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: std_msgs/Int16
    int16 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cones(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.blue_cones !== undefined) {
      resolved.blue_cones = new Array(msg.blue_cones.length);
      for (let i = 0; i < resolved.blue_cones.length; ++i) {
        resolved.blue_cones[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.blue_cones[i]);
      }
    }
    else {
      resolved.blue_cones = []
    }

    if (msg.yellow_cones !== undefined) {
      resolved.yellow_cones = new Array(msg.yellow_cones.length);
      for (let i = 0; i < resolved.yellow_cones.length; ++i) {
        resolved.yellow_cones[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.yellow_cones[i]);
      }
    }
    else {
      resolved.yellow_cones = []
    }

    if (msg.size !== undefined) {
      resolved.size = std_msgs.msg.Int16.Resolve(msg.size)
    }
    else {
      resolved.size = new std_msgs.msg.Int16()
    }

    return resolved;
    }
};

module.exports = Cones;
