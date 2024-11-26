
"use strict";

let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let SVADC = require('./SVADC.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let SaveSensorData = require('./SaveSensorData.js');
let GhostMessage = require('./GhostMessage.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let RadarDetections = require('./RadarDetections.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let ERP42Info = require('./ERP42Info.js');
let SensorPosControl = require('./SensorPosControl.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let EventInfo = require('./EventInfo.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let RadarDetection = require('./RadarDetection.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let ObjectStatus = require('./ObjectStatus.js');
let PRStatus = require('./PRStatus.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let WaitForTick = require('./WaitForTick.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let PREvent = require('./PREvent.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let IntscnTL = require('./IntscnTL.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let ReplayInfo = require('./ReplayInfo.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let CollisionData = require('./CollisionData.js');
let IntersectionControl = require('./IntersectionControl.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let GPSMessage = require('./GPSMessage.js');
let DillyCmd = require('./DillyCmd.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let Lamps = require('./Lamps.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let MapSpec = require('./MapSpec.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let CtrlCmd = require('./CtrlCmd.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let VehicleCollision = require('./VehicleCollision.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let VehicleSpec = require('./VehicleSpec.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let TrafficLight = require('./TrafficLight.js');

module.exports = {
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  DillyCmdResponse: DillyCmdResponse,
  FaultInjection_Controller: FaultInjection_Controller,
  SVADC: SVADC,
  SyncModeAddObject: SyncModeAddObject,
  SaveSensorData: SaveSensorData,
  GhostMessage: GhostMessage,
  WoowaDillyStatus: WoowaDillyStatus,
  MoraiSimProcHandle: MoraiSimProcHandle,
  RadarDetections: RadarDetections,
  SetTrafficLight: SetTrafficLight,
  NpcGhostInfo: NpcGhostInfo,
  ObjectStatusExtended: ObjectStatusExtended,
  VehicleSpecIndex: VehicleSpecIndex,
  ObjectStatusListExtended: ObjectStatusListExtended,
  ScenarioLoad: ScenarioLoad,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  ERP42Info: ERP42Info,
  SensorPosControl: SensorPosControl,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  MultiEgoSetting: MultiEgoSetting,
  SkateboardStatus: SkateboardStatus,
  SyncModeResultResponse: SyncModeResultResponse,
  EgoVehicleStatus: EgoVehicleStatus,
  MultiPlayEventResponse: MultiPlayEventResponse,
  MoraiSrvResponse: MoraiSrvResponse,
  FaultStatusInfo: FaultStatusInfo,
  EventInfo: EventInfo,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  RadarDetection: RadarDetection,
  FaultInjection_Response: FaultInjection_Response,
  MapSpecIndex: MapSpecIndex,
  FaultInjection_Tire: FaultInjection_Tire,
  ObjectStatus: ObjectStatus,
  PRStatus: PRStatus,
  MoraiTLInfo: MoraiTLInfo,
  MultiPlayEventRequest: MultiPlayEventRequest,
  WaitForTick: WaitForTick,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  NpcGhostCmd: NpcGhostCmd,
  PREvent: PREvent,
  SyncModeCmd: SyncModeCmd,
  IntscnTL: IntscnTL,
  IntersectionStatus: IntersectionStatus,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  ReplayInfo: ReplayInfo,
  DdCtrlCmd: DdCtrlCmd,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  CollisionData: CollisionData,
  IntersectionControl: IntersectionControl,
  MoraiSimProcStatus: MoraiSimProcStatus,
  GPSMessage: GPSMessage,
  DillyCmd: DillyCmd,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  Lamps: Lamps,
  GetTrafficLightStatus: GetTrafficLightStatus,
  FaultInjection_Sensor: FaultInjection_Sensor,
  MapSpec: MapSpec,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  WaitForTickResponse: WaitForTickResponse,
  PRCtrlCmd: PRCtrlCmd,
  VehicleCollisionData: VehicleCollisionData,
  CtrlCmd: CtrlCmd,
  ObjectStatusList: ObjectStatusList,
  SyncModeRemoveObject: SyncModeRemoveObject,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  VehicleCollision: VehicleCollision,
  MoraiTLIndex: MoraiTLIndex,
  SyncModeInfo: SyncModeInfo,
  VehicleSpec: VehicleSpec,
  SyncModeCmdResponse: SyncModeCmdResponse,
  SyncModeSetGear: SyncModeSetGear,
  TrafficLight: TrafficLight,
};
