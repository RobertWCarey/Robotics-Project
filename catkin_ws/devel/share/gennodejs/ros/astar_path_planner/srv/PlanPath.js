// Auto-generated. Do not edit!

// (in-package astar_path_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PlanPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.diagonal_movement = null;
      this.heuristic_cost_weight = null;
    }
    else {
      if (initObj.hasOwnProperty('diagonal_movement')) {
        this.diagonal_movement = initObj.diagonal_movement
      }
      else {
        this.diagonal_movement = false;
      }
      if (initObj.hasOwnProperty('heuristic_cost_weight')) {
        this.heuristic_cost_weight = initObj.heuristic_cost_weight
      }
      else {
        this.heuristic_cost_weight = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanPathRequest
    // Serialize message field [diagonal_movement]
    bufferOffset = _serializer.bool(obj.diagonal_movement, buffer, bufferOffset);
    // Serialize message field [heuristic_cost_weight]
    bufferOffset = _serializer.float64(obj.heuristic_cost_weight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanPathRequest
    let len;
    let data = new PlanPathRequest(null);
    // Deserialize message field [diagonal_movement]
    data.diagonal_movement = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [heuristic_cost_weight]
    data.heuristic_cost_weight = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'astar_path_planner/PlanPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '268bfe2029457d71986716aa0420efe4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool diagonal_movement
    float64 heuristic_cost_weight
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanPathRequest(null);
    if (msg.diagonal_movement !== undefined) {
      resolved.diagonal_movement = msg.diagonal_movement;
    }
    else {
      resolved.diagonal_movement = false
    }

    if (msg.heuristic_cost_weight !== undefined) {
      resolved.heuristic_cost_weight = msg.heuristic_cost_weight;
    }
    else {
      resolved.heuristic_cost_weight = 0.0
    }

    return resolved;
    }
};

class PlanPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.length_of_path = null;
      this.number_of_nodes_in_closed_set = null;
    }
    else {
      if (initObj.hasOwnProperty('length_of_path')) {
        this.length_of_path = initObj.length_of_path
      }
      else {
        this.length_of_path = 0;
      }
      if (initObj.hasOwnProperty('number_of_nodes_in_closed_set')) {
        this.number_of_nodes_in_closed_set = initObj.number_of_nodes_in_closed_set
      }
      else {
        this.number_of_nodes_in_closed_set = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanPathResponse
    // Serialize message field [length_of_path]
    bufferOffset = _serializer.int32(obj.length_of_path, buffer, bufferOffset);
    // Serialize message field [number_of_nodes_in_closed_set]
    bufferOffset = _serializer.int32(obj.number_of_nodes_in_closed_set, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanPathResponse
    let len;
    let data = new PlanPathResponse(null);
    // Deserialize message field [length_of_path]
    data.length_of_path = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [number_of_nodes_in_closed_set]
    data.number_of_nodes_in_closed_set = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'astar_path_planner/PlanPathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '63b9c3f614e9dd9877a69c4e1441def7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 length_of_path
    int32 number_of_nodes_in_closed_set
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanPathResponse(null);
    if (msg.length_of_path !== undefined) {
      resolved.length_of_path = msg.length_of_path;
    }
    else {
      resolved.length_of_path = 0
    }

    if (msg.number_of_nodes_in_closed_set !== undefined) {
      resolved.number_of_nodes_in_closed_set = msg.number_of_nodes_in_closed_set;
    }
    else {
      resolved.number_of_nodes_in_closed_set = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: PlanPathRequest,
  Response: PlanPathResponse,
  md5sum() { return '64724dde033fbe8f19a82fd1a78b4bdc'; },
  datatype() { return 'astar_path_planner/PlanPath'; }
};
