const comp = require('../../../common/ComponentManager.js');

const idUtils = require('../../../common/utils/nameid_utils.js');
const bbUtils = require('../../../common/utils/blackboard_utils.js');
const jsUtils = require('../../../common/utils/js_utils.js');
const threeDUtils = require('../../../common/utils/3d_utils.js');
const BbConst = require('../../../common/constants/BlackboardConstants.js');
const VtConst = require('../../../common/constants/ValueTypeConstants.js');
const RmConst = require('../../../common/constants/RobotModelConstants.js');
const TmConst = require('../../../common/constants/TaskManagerConstants.js');
const WpfConst = require('../../../common/constants/WpfManagerConstants.js');
const AfConst = require('./AffordanceConstants.js');
const AmConst = require('../../../common/constants/ArmMotionControllerConstants.js');
const EdConst = require('../../../common/constants/EventDispatcherConstants.js');
const EeConst = require('../../../common/constants/EndEffectorManagerConstants.js');
const EndpointConst = require('../../../common/constants/EndpointsManagerConstants.js');
const { NodeTypes } = require('../../../common/constants/NodeTypeConstants.js');
const EventHelper = require('../../../common/eventdispatcher/EventHelper.js');
const three = require('three');

const Frame = require('../../../common/hybrid/Frame.js');
const Waypoint = require('../../../common/hybrid/Waypoint.js');
const Pose = require('../../../common/hybrid/Pose.js');

const AffordanceUtils = require('./AffordanceUtils.js');

const Entity = require('./Entity.js');


// all data for toolplate in base frame
// [{ <Three.vector>position, <Three.quaternion> rotation, jointHints }]
let data = [];

// data index is the last index in the data array that this node "consumes"
// [{ <int>dataIndex, <string>nodeId }]
let nodeRecords = [];

let DISTANCE_THRESHOLD = 0.002;      // record live data at 2 mm resolution
let RADIAN_THRESHOLD = (Math.PI / 180.0);  // record at 1 degree resolution

let MOVE_DISTANCE_THRESHOLD = 0.08;      // place a move node every 8 cm.
let MOVE_RADIAN_THRESHOLD = 10*(Math.PI / 180.0);  // record at 10 degree resolution

let SAMPLE_RESOLUTION_DISTANCE = 0.002;                 // 2 mm resolution
let SAMPLE_RESOLUTION_RADIANS = (Math.PI / 180.0);      // 1 degree resolution

const NODE_RECORD_MOVE = 'move';
const NODE_RECORD_AFFORDANCE = 'affordance';


var AffordanceRecorder = function () {

  this.recording = false;
  this.parentNodeId = null;
  this.armMotionManager = null;
  this.behaviorTree = null;
  this.blackboard = null;
  this.wpfManager = null;
  this.endEffectorManager = null;
  this.eventHelper = null;
  this.moveIndex= 0;

  this.processedTrajectories = {};
  this.entityRecords = [];

};


AffordanceRecorder.prototype = {

  init: function (options) {
    this.armMotionManager = comp.get(AmConst.CompIds.armMotionManager);
    this.wpfManager = comp.get(WpfConst.CompIds.engWpfManager);
    this.endEffectorManager = comp.get(EeConst.CompIds.engEndEffectorManager);
    this.eventHelper = new EventHelper();

    this.refreshAffordanceTrajectories();
  },

  getComponentId: function () {
    return AfConst.CompIds.engAffordanceRecorder;
  },

  recordingStart: function(parent) {

    let taskManager = comp.get(TmConst.CompIds.engTaskManager);
    this.behaviorTree = taskManager.getMainTask().getBehaviorTree();
    this.blackboard = this.behaviorTree.getBlackboard();

    const sequenceNodeInfo = this.behaviorTree.createNode(NodeTypes.Sequence, parent, null, null, -1, null);

    this.eventHelper.postRemoteEvent(EdConst.Dispatchers.localAndClients, AfConst.Events.affordanceRecordingStarted, null,
      AfConst.Topics.affordances, this);

    // clear out existing values
    data = [];
    nodeRecords = [];
    this.entityRecords = [];

    this.parentNodeId = sequenceNodeInfo.id;
    this.recording = true;
    this.moveIndex = 0;

    // find all the entities
    let entities = this.blackboard.getAllValues(BbConst.ItemContexts.entity);
    this.entityRecords = entities.map((entity) => {
      let frame = this.wpfManager.getFrame(entity.frameId);
      return { entity, frame };
    });

    this.refreshAffordanceTrajectories();

    this.recordingTick();
  },

  insertMoveNode : function(newData) {

    // add a pose and waypoint
    const poseName = 'endpoint_' + this.moveIndex;

    let pose = new Pose(null, WpfConst.FrameIds.baseFrame);
    pose.setFromTranslationRotation(newData.position, newData.rotation);
    pose.id = poseName + '_' + idUtils.generateShortId();
    pose.name = poseName;
    this.wpfManager.addPose(pose);

    let update = false;

    // decide whether to add a new waypoint or update the last one

    if (nodeRecords.length > 1) {
      let lastRecord = nodeRecords[nodeRecords.length-1];
      if (lastRecord.nodeType === NODE_RECORD_MOVE) {
        let penultimateRecord = nodeRecords[nodeRecords.length-2];
        let lastData = data[lastRecord.dataIndex];
        let penultimateData = data[penultimateRecord.dataIndex];

        let v = newData.position.clone();
        let u = lastData.position.clone();

        v.sub(penultimateData.position);
        u.sub(penultimateData.position);

        v.normalize();
        v.multiplyScalar(v.dot(u));
        u.sub(v);

        if (u.length() <= 0.02) {
          update = true;
        }
      }
    }

    if (update) {
      let lastRecord = nodeRecords[nodeRecords.length-1];
      let oldNode = this.behaviorTree._idToNodeMap[lastRecord.nodeId];
      let oldWaypoint = this.wpfManager.getWaypoint(oldNode.waypointId);
      let oldPoseId = oldWaypoint.poseId;

      lastRecord.dataIndex = data.length-1;

      this.behaviorTree.addOrSetNodeWaypointToPose(lastRecord.nodeId, pose.id, newData.jointHints).then((waypointId) => {
        // do anything here?
        this.wpfManager.removePose(oldPoseId);
      });

      return null;
    }
    else {
      const nodeInfo = this.behaviorTree.createNode(NodeTypes.MoveTo, this.parentNodeId, null, null, -1, null);
      if (nodeInfo) {
        let nodeId = nodeInfo.id;

        // FIXME: this shouldn't need to be a promise, since we don't need to solve IK
        this.behaviorTree.addOrSetNodeWaypointToPose(nodeId, pose.id, newData.jointHints, RmConst.DefaultRobotArmEndpoint).then((waypointId) => {
          // do anything here?
        });

        return nodeId;
      }
      else {
        return null;
      }
    }
  },


  recordingTick : function() {
    if (!this.recording) {
      return this.recordingFinish();
    }

    // tick!
    let pose = this.armMotionManager.getEndEffectorPose(RmConst.DefaultRobotArmEndpoint);

    let position = pose.pos;
    let rotation = pose.rotation;

    let addData = false;
    let dl = data.length;

    if (dl === 0) {
      addData = true;
    }
    else {
      let lastData = data[dl-1];
      if (position.distanceTo(lastData.position) > DISTANCE_THRESHOLD) {
        addData = true;
      }
      else if (2 * Math.acos(rotation.clone().multiply(lastData.rotation.clone().inverse()).w) > RADIAN_THRESHOLD) {
        addData = true;
      }
    }

    if (addData) {
      let newData = {
        position: position,     // do we need to keep these?
        rotation: rotation,
        matrix: threeDUtils.posRotToMatrix4({ pos: position, rotation: rotation}),
        jointHints: this.armMotionManager.getJointHints()
      };

      data.push(newData);


      // try and match against something
      let matched = this.findAffordanceTrajectory();
      if (!matched) {

        // default: move node
        let placeMove = false;

        if (nodeRecords.length) {
          let lastNode = nodeRecords[nodeRecords.length - 1];
          let lastNodeData = data[lastNode.dataIndex];

          if (position.distanceTo(lastNodeData.position) > MOVE_DISTANCE_THRESHOLD) {
            placeMove = true;
          }
          else if (2 * Math.acos(rotation.clone().multiply(lastNodeData.rotation.clone().inverse()).w) > MOVE_RADIAN_THRESHOLD) {
            placeMove = true;
          }
        }
        else {
          placeMove = true;
        }

        if (false){//placeMove) {
          let nodeId = this.insertMoveNode(newData);
          if (nodeId) {
            nodeRecords.push({
              dataIndex: data.length - 1,
              nodeId: nodeId,
              nodeType: NODE_RECORD_MOVE
            });
          }
        }
      }
    }

    // schedule the next tick
    setTimeout(() => {                // tick at 100 hz
      this.recordingTick();
    }, 10);

  },

  recordingFinish : function() {
    this.eventHelper.postRemoteEvent(EdConst.Dispatchers.localAndClients, AfConst.Events.affordanceRecordingFinished, null,
      AfConst.Topics.affordances, this);
  },

  toggleRecording : function(parent) {
    if (this.recording) {
      // stop!
      this.recording = false;
    }
    else if (parent) {
      this.recordingStart(parent);
    }
  },

  // --------------------- Trajectory Matching ---------------------


  refreshAffordanceTrajectories() {
    Object.keys(AfConst.Affordances).forEach((entityType) => {    // e.g. entityType = button

      if (!this.processedTrajectories[entityType]) {
        this.processedTrajectories[entityType] = {};
      }
      let preProcessedEntityType = this.processedTrajectories[entityType];

      let entityAffordances = AfConst.Affordances[entityType];
      Object.keys(entityAffordances).forEach((affordanceId) => {    // e.g. affordanceId = push

        if (!preProcessedEntityType[affordanceId]) {
          preProcessedEntityType[affordanceId] = {};
        }
        let preProcessedTrajectory = preProcessedEntityType[affordanceId];

        let affordanceConfig = entityAffordances[affordanceId];
        let template = affordanceConfig.templateJson;

        // cache tooltip
        preProcessedTrajectory.toolTip = template.toolTip;
        preProcessedTrajectory.matchRotation = template.matchRotation;

        if (!preProcessedTrajectory.samples) {
          let targetEeGeometry = this.endEffectorManager.getEndpointGeometry(template.toolTip);
          if (targetEeGeometry) {
            let targetEeMatrix = threeDUtils.posRotToMatrix4({
              pos: new three.Vector3(targetEeGeometry.pos.x, targetEeGeometry.pos.y, targetEeGeometry.pos.z),
              rotation: new three.Quaternion().setFromEuler(new three.Euler(targetEeGeometry.rotation.rx, targetEeGeometry.rotation.ry, targetEeGeometry.rotation.rz, 'ZYX'))
            });

            preProcessedTrajectory.plateToEeMatrix = targetEeMatrix.clone();

            let remappedWaypoints = template.waypointSequence.map((waypointId) => {
              let waypoint = template.templateWaypoints[waypointId];

              let pose = Pose.fromJSON(template.templatePoses[waypoint.poseId]);
              let position = null;
              let rotation = null;

              if (waypoint.endpointId === template.toolTip) {
                position = pose.pos;
                rotation = pose.rotation;
              }
              else {
                // transform the pose into the given tooltip

                let poseObject3d = pose.getTransformAsObject3d();
                let toolPlateMatrix = null;

                if (waypoint.endpointId === RmConst.DefaultRobotArmEndpoint) {
                  toolPlateMatrix = poseObject3d.matrix.clone();
                }
                else {
                  let sourceEeGeometry = this.endEffectorManager.getEndpointGeometry(waypoint.endpointId);
                  let sourceEeMatrix = threeDUtils.posRotToMatrix4({
                    pos: new three.Vector3(sourceEeGeometry.pos.x, sourceEeGeometry.pos.y, sourceEeGeometry.pos.z),
                    rotation: new three.Quaternion().setFromEuler(new three.Euler(sourceEeGeometry.rotation.rx, sourceEeGeometry.rotation.ry, sourceEeGeometry.rotation.rz, 'ZYX'))
                  });
                  let sourceInverse = new three.Matrix4().getInverse(sourceEeMatrix);
                  toolPlateMatrix = new three.Matrix4();
                  toolPlateMatrix.multiplyMatrices(poseObject3d.matrix, sourceInverse);
                }

                let targetMatrix = null;

                if (template.toolTip === RmConst.DefaultRobotArmEndpoint) {
                  targetMatrix = toolPlateMatrix;
                }
                else {
                  targetMatrix = new three.Matrix4();
                  targetMatrix.multiplyMatrices(toolPlateMatrix, targetEeMatrix);
                }

                position = new THREE.Vector3();
                rotation = new THREE.Quaternion();
                let scale = new THREE.Vector3();
                targetMatrix.decompose(position, rotation, scale);
              }

              return {position, rotation};
            });

            // store these backwards

            let samples = [remappedWaypoints[remappedWaypoints.length - 1]];    // seed with first one

            for (let i = remappedWaypoints.length - 1; i > 0; i--) {

              let d0 = remappedWaypoints[i];
              let d1 = remappedWaypoints[i - 1];

              let v = d1.position.clone();
              v.sub(d0.position);
              let distanceDelta = v.length();
              let dStepCount = Math.ceil(distanceDelta / SAMPLE_RESOLUTION_DISTANCE);

              let relativeRotation = d0.rotation.clone();
              relativeRotation.multiply(d1.rotation.clone().inverse());
              let w = Math.max(-1, Math.min(1, relativeRotation.w));    // clamp to -1, 1
              let angleDelta = (2 * Math.acos(w)); // different between two angles
              let rStepCount = Math.ceil(angleDelta / SAMPLE_RESOLUTION_RADIANS);
              let stepCount = Math.max(dStepCount, rStepCount);
              let tFraction = 1.0 / stepCount;

              if (stepCount > 0) {

                // create an linearly interpolated trajectory

                for (let tIndex = 0; tIndex < stepCount; tIndex++) {
                  let t = (tIndex + 1) * tFraction;

                  let newPosition = v.clone();
                  newPosition.multiplyScalar(t);
                  newPosition.add(d0.position);

                  let newRotation = new three.Quaternion();
                  THREE.Quaternion.slerp(d0.rotation, d1.rotation, newRotation, t);

                  samples.push({position: newPosition, rotation: newRotation});
                }
              }
            }

            preProcessedTrajectory.samples = samples;
          }
        }
      });
    });
  },

  matchAffordanceTrajectory(trajectory, entity, affordanceId, frame) {

    let lastDataIndex = 0;      // TODO: only go as far back as the last node

    for (let n = nodeRecords.length-1; n >= 0; n--) {
      let nodeRecord = nodeRecords[n];
      if (nodeRecord.nodeType === NODE_RECORD_AFFORDANCE) {
        lastDataIndex = nodeRecord.dataIndex+1;
      }
      else {
        // keep going
      }
    }

    if (nodeRecords.length > 0) {
      if (nodeRecords[nodeRecords.length-1].entityId === entity.id) {
        return false;
      }
    }

    // REMEMBER: data represents the TOOLPLATE state in the BASE frame
    let entityMatrix = this.wpfManager.getFrameToBaseMatrix(frame);
    let inverseEntityMatrix = new three.Matrix4().getInverse(entityMatrix);
    let postCorrectionMatrix = null;
    let preCorrectionMatrix = null;
    let inverseAlignmentMatrix = null;

    {
      let firstData = data[data.length-1];
      let firstDataPlateMatrix = threeDUtils.posRotToMatrix4({ pos: firstData.position, rotation: firstData.rotation});
      let firstDataEeMatrix = firstDataPlateMatrix.clone().multiply(trajectory.plateToEeMatrix);
      let firstDataEeLocal = firstDataEeMatrix.clone().premultiply(inverseEntityMatrix);

      let p = new three.Vector3();
      let q = new three.Quaternion();

      p.setFromMatrixPosition(firstDataEeLocal);
      q.setFromRotationMatrix(firstDataEeLocal);

      // REMEMBER: samples represent the EE state in the ENTITY frame

      let firstSample = trajectory.samples[0];

      // we want trajectory matches to be rotation-invariant relative to the entity frame
      // match the rotation of the data to the stored trajectory
      // we'll convert the rotation delta to euler angles, then zero out all but the yaw, and then recompose

      let rotationDiff = firstSample.rotation.clone().multiply(q.clone().inverse());
      let rotationDiffEuler = new three.Euler().setFromQuaternion(rotationDiff, 'ZXY');
      rotationDiffEuler.x = 0;
      rotationDiffEuler.y = 0;
      rotationDiff.setFromEuler(rotationDiffEuler);
      let alignmentMatrix = new three.Matrix4().makeRotationFromQuaternion(rotationDiff);
      inverseAlignmentMatrix = new three.Matrix4().getInverse(alignmentMatrix);

      // store off the required transformations as compactly as possible

      postCorrectionMatrix = trajectory.plateToEeMatrix.clone();
      preCorrectionMatrix = inverseEntityMatrix.clone().premultiply(alignmentMatrix);

      /*
      let testMatrix = firstDataPlateMatrix.multiply(postCorrectionMatrix).premultiply(preCorrectionMatrix);
      let testPosition = new three.Vector3();
      let testRotation = new three.Quaternion();
      testPosition.setFromMatrixPosition(testMatrix);
      testRotation.setFromRotationMatrix(testMatrix);
      */
    }


    let trajectoryIndex= 0;
    let furthestTrajectoryIndex = 0;
    const trajectoryRange = 3;
    let totalError = 0;
    let firstDatum = true;
    let i;

    for (i = data.length-1; i >= lastDataIndex; i--) {
      let d = data[i];
      let m = d.matrix;

      let localM = m.clone().multiply(postCorrectionMatrix).premultiply(preCorrectionMatrix);

      // JUST T FOR NOW

      let localP = new three.Vector3();
      localP.setFromMatrixPosition(localM);
      let localQ = new three.Quaternion();
      localQ.setFromRotationMatrix(localM);

      // find the best trajectory range
      let bestD = 100;
      let bestT = -1;
      let maxt = Math.min(trajectoryIndex+trajectoryRange, trajectory.samples.length-1);
      for (let t = trajectoryIndex; t < maxt; t++) {

        let threshold = 0.03;     // keep within 2 cm
        let aThreshold = 5*(Math.PI / 180.0);

        if (t === trajectoryIndex && !firstDatum) {
          threshold = 0.04; // special case
          aThreshold = 10*(Math.PI / 180.0);
        }

        let d = localP.distanceTo(trajectory.samples[t].position);

        let a = 0;
        if (trajectory.matchRotation) {
          a = 2 * Math.acos(localQ.clone().multiply(trajectory.samples[t].clone().inverse()).w);
        }

        if ((d < threshold) &&
            (a < aThreshold))
        {
          furthestTrajectoryIndex = Math.max(furthestTrajectoryIndex, t);

          let totalScore = d + 0.01*a/(5*(Math.PI / 180.0));    // 5 degrees equivalent to 1cm
          if (totalScore < bestD) {
            bestD = totalScore;
            bestT = t;
          }
        }
      }

      firstDatum = false;

      if (bestT >= 0) {
        // yes, we're still within range of the stored trajectory
        trajectoryIndex = bestT;
        totalError = bestD;
        if (trajectoryIndex >= trajectory.samples.length-1) {
          break;
        }
      }
      else {
        break;
      }
    }

    if ((furthestTrajectoryIndex + trajectoryRange) > trajectory.samples.length) {
      // we go close enough to the end to be considered good!

      // first clear any nodes (and node records) that we overlap with

      for (let r = nodeRecords[nodeRecords.length-1]; r >= 0; r--) {
        let nr = nodeRecords[r];
        if ((nr.dataIndex >= i) && nr.nodeId && (nr.nodeType === NODE_RECORD_MOVE)) {
          this.behaviorTree.removeChild(this.parentNodeId, nr.nodeId);
        }
      }

      let record = {
        dataIndex: data.length - 1,
        nodeId: null,
        nodeType: NODE_RECORD_AFFORDANCE,
        entityId: entity.id
      };
      nodeRecords.push(record);

      AffordanceUtils.createEntityAffordance(entity.id, affordanceId, this.parentNodeId, inverseAlignmentMatrix).then((nodeInfo) => {
        record.nodeId = nodeInfo.id;
      });

      return true;
    }

    return false;

  },

  findAffordanceTrajectory() {

    let result = false;

    if (data.length === 0) {
      return false;
    }

    for (let i = 0; i < this.entityRecords.length; i++) {

      let entityRecord = this.entityRecords[i];
      let entity = entityRecord.entity;

      let ppEntityType = this.processedTrajectories[entity.type];
      if (ppEntityType) {
        result = Object.keys(ppEntityType).some((affordanceId) => {

          let ppTrajectory = ppEntityType[affordanceId];

          // try each of these preprocessed trajectories
          if (this.matchAffordanceTrajectory(ppTrajectory, entity, affordanceId, entityRecord.frame)) {
            return true;
          }
        });

        if (result) {
          break;
        }
      }
    }
    return result;
  }



};

module.exports = AffordanceRecorder;
