// Auto-generated. Do not edit!

// (in-package integration_test.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class testmsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sequenceNum = null;
    }
    else {
      if (initObj.hasOwnProperty('sequenceNum')) {
        this.sequenceNum = initObj.sequenceNum
      }
      else {
        this.sequenceNum = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type testmsg
    // Serialize message field [sequenceNum]
    bufferOffset = _serializer.uint8(obj.sequenceNum, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type testmsg
    let len;
    let data = new testmsg(null);
    // Deserialize message field [sequenceNum]
    data.sequenceNum = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'integration_test/testmsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '96f4abd1a5050e1e1afc651209403d91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 sequenceNum
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new testmsg(null);
    if (msg.sequenceNum !== undefined) {
      resolved.sequenceNum = msg.sequenceNum;
    }
    else {
      resolved.sequenceNum = 0
    }

    return resolved;
    }
};

module.exports = testmsg;
