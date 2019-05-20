// Auto-generated. Do not edit!

// (in-package keyboard_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class key_in {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.keycode = null;
    }
    else {
      if (initObj.hasOwnProperty('keycode')) {
        this.keycode = initObj.keycode
      }
      else {
        this.keycode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type key_in
    // Serialize message field [keycode]
    bufferOffset = _serializer.int8(obj.keycode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type key_in
    let len;
    let data = new key_in(null);
    // Deserialize message field [keycode]
    data.keycode = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'keyboard_control/key_in';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '622b0edfae335c0cbebcebf432e2cce0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 keycode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new key_in(null);
    if (msg.keycode !== undefined) {
      resolved.keycode = msg.keycode;
    }
    else {
      resolved.keycode = 0
    }

    return resolved;
    }
};

module.exports = key_in;
