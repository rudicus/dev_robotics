const comp = require('../../../common/ComponentManager.js');

const idUtils = require('../../../common/utils/nameid_utils.js');
const bbUtils = require('../../../common/utils/blackboard_utils.js');
const BbConst = require('../../../common/constants/BlackboardConstants.js');
const VtConst = require('../../../common/constants/ValueTypeConstants.js');
const RmConst = require('../../../common/constants/RobotModelConstants.js');
const TmConst = require('../../../common/constants/TaskManagerConstants.js');
const WpfConst = require('../../../common/constants/WpfManagerConstants.js');
const EndpointConst = require('../../../common/constants/EndpointsManagerConstants.js');
const AfConst = require('./AffordanceConstants.js');
const AffordanceTemplate = require('./AffordanceTemplates.js');


const NodeTypes = require('../../../common/constants/NodeTypeConstants.js').NodeTypes;


const Frame = require('../../../common/hybrid/Frame.js');

const Entity = require('./Entity.js');


let affordanceObjectCount = 0;

// Bueller? Bueller?

function generateEntityNameAndId(type) {
  let gname = `${type}_${affordanceObjectCount}`;
  let id = `${gname}_${idUtils.generateShortId()}`;
  affordanceObjectCount++;
  return { gname, id };
}

/**
 *
 * @param {Task} task
 * @param {string} name
 * @param {string} type
 * @param {string} subType
 * @param {Frame} frame
 * @returns {string} id blackboard key of the entity item that was created
 */
function addEntityToTask(task, name, type, subType, frame) {
  let blackboard = task.getUserBlackboard();
  let { gname, id } = generateEntityNameAndId(type);
  name = name || gname;
  frame = frame.clone();
  frame.name = name + '_frame';
  frame.id = frame.name + '_' + idUtils.generateShortId();
  let wpfManager = comp.get(WpfConst.CompIds.engWpfManager);

  wpfManager.addFrame(frame).then((frame) => {
    let entity = new Entity(null, id, name, type, subType, frame.id);
    let entityItem = blackboard.defineItem(BbConst.ItemContexts.entity, id, name, VtConst.Objects.Entity, entity);
    if (entityItem) {
      return id;
    }
    return false;
  });

}

function removeEntityFromTask(task, id) {
  let blackboard = task.getUserBlackboard();
  let entityKey = bbUtils.composeKey(BbConst.ItemContexts.entity, id);
  let result = false;

  let entity = blackboard.getValue(entityKey);
  if (entity) {
    let wpfManager = comp.get(WpfConst.CompIds.engWpfManager);
    wpfManager.removeFrame(entity.frameId);
    blackboard.removeItem(entityKey);
  }

  return result;
}

/**
 * Creates an affordance object of the given type at the given location
 * @param {string} name
 * @param [type]
 * @param [subType]
 */
function testAddEntity(name, type, subType) {
  type = type || AfConst.Types.button;
  subType = subType || AfConst.Subtypes.subtype1;

  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  const endpointsManager = comp.get(EndpointConst.CompIds.engEndpointsManager);
  const robotModelMgr  = comp.get(RmConst.CompIds.robotModelMgr);

  let task = taskManager.getMainTask();
  let endPointId = endpointsManager.getActiveEndpoint();
  let currentPose = robotModelMgr.getRobotModelEndpointPose(RmConst.EngineInstances.referenceJoints, endPointId);
  let frame = Frame.fromPose(currentPose);

  return addEntityToTask(task, name, type, subType, frame);
}

function createEntityWithCurrentPose(entityName, type, subType, frameId) {

  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  const endpointsManager = comp.get(EndpointConst.CompIds.engEndpointsManager);
  const robotModelMgr  = comp.get(RmConst.CompIds.robotModelMgr);
  let amanager = comp.get(AfConst.CompIds.engAffordanceManager);

  let data = amanager.getCurrentAffordances(frameId);
  ////
  for (let i = 0; i < data.length; i++ ) {

    let task = taskManager.getMainTask();
    let endPointId = endpointsManager.getActiveEndpoint();

    let deltaR = new THREE.Matrix4();
    deltaR.makeRotationX(3.14159);

    let v = new THREE.Vector3().copy(data[i].frame.pos);
    v.applyMatrix4(deltaR);

    let R = new THREE.Matrix4().makeRotationFromQuaternion(data[i].frame.rotation);
    R.multiplyMatrices(deltaR, R);

    //calculate a pose from the new position and orientation, update the state of whatever you are editing
    //data[i].frame.pos = v;
    data[i].frame.rotation = new THREE.Quaternion().setFromRotationMatrix(R);

    let frame = new Frame();
    frame.setFromTranslationRotation(data[i].frame.pos,data[i].frame.rotation);




    addEntityToTask(task, data[i].subtype, data[i].type, data[i].subtype, frame);
  }

  return
}

function updateEntity(entity) {
  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  let task = taskManager.getMainTask();
  let blackboard = task.getUserBlackboard();

  const key = bbUtils.composeKey(BbConst.ItemContexts.entity, entity.id);
  return blackboard.setValue(key, entity, true);
}

function updateEntityWithCurrentPose(entityId) {
  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  let task = taskManager.getMainTask();
  let blackboard = task.getUserBlackboard();
  let entityKey = bbUtils.composeKey(BbConst.ItemContexts.entity, entityId);

  let entity = blackboard.getValue(entityKey);
  if (entity) {
    let wpfManager = comp.get(WpfConst.CompIds.engWpfManager);
    let frame = wpfManager.getFrame(entity.frameId);
    if (frame) {
      return wpfManager.updateFrameWithCurrentPose(frame);
    }
    else {
      Promise.reject(new Error('Entity has invalid frame id'));
    }
  }
  else {
    Promise.reject(new Error('Invalid entity key'));
  }
}

function removeEntity(entityKey) {
  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  let task = taskManager.getMainTask();
  let id = bbUtils.parseKey(entityKey).keyBody;
  return removeEntityFromTask(task, id);
}

function createEntityAffordance(entityId, affordanceId, parentNodeId, modMatrix) {
  const taskManager = comp.get(TmConst.CompIds.engTaskManager);
  let task = taskManager.getMainTask();
  let blackboard = task.getUserBlackboard();
  let entityKey = bbUtils.composeKey(BbConst.ItemContexts.entity, entityId);

  let entity = blackboard.getValue(entityKey);
  if (entity) {
    return AffordanceTemplate.generateAffordanceTemplate(entity, affordanceId, modMatrix).then((templateJson) => {
      let behaviorTree = task.getBehaviorTree();
      let childIndex = -1;
      return behaviorTree.createNode(NodeTypes.Affordance, parentNodeId, null, null, childIndex, null, {templateJson: templateJson});
    });
  }
}



module.exports = {
  addEntityToTask,
  createEntityWithCurrentPose,
  updateEntity,
  updateEntityWithCurrentPose,
  removeEntity,
  createEntityAffordance,
};