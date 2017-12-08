const comp = require('../../../common/ComponentManager.js');

const three = require('three');
const idUtils = require('../../../common/utils/nameid_utils.js');
const bbUtils = require('../../../common/utils/blackboard_utils.js');
const jsUtils = require('../../../common/utils/js_utils.js');
const threeDUtils = require('../../../common/utils/3d_utils.js');
const BbConst = require('../../../common/constants/BlackboardConstants.js');
const AmcConst = require('../../../common/constants/ArmMotionControllerConstants.js');
const VtConst = require('../../../common/constants/ValueTypeConstants.js');
const RmConst = require('../../../common/constants/RobotModelConstants.js');
const TmConst = require('../../../common/constants/TaskManagerConstants.js');
const WpfConst = require('../../../common/constants/WpfManagerConstants.js');
const EndpointConst = require('../../../common/constants/EndpointsManagerConstants.js');
const AfConst = require('./AffordanceConstants.js');

const Frame = require('../../../common/hybrid/Frame.js');
const Waypoint = require('../../../common/hybrid/Waypoint.js');
const Pose = require('../../../common/hybrid/Pose.js');

const Entity = require('./Entity.js');


function generateAffordanceTemplate(entity, affordanceId, modMatrix) {
  let wpfManager = comp.get(WpfConst.CompIds.engWpfManager);
  let entityConfig = AfConst.Affordances[entity.type];
  if (entityConfig) {
    let affordance = entityConfig[affordanceId];
    let templateJson = jsUtils.cloneObject(affordance.templateJson);

    let oldToNewPoseId = {};
    let oldToNewWaypointId = {};
    let poses = {};
    let waypoints = {};
    let waypointPromises = [Promise.resolve()];

    Object.keys(templateJson.templatePoses).forEach((poseKey) => {
      let pose = Pose.fromJSON(templateJson.templatePoses[poseKey]);
      let oldPoseId = pose.id;

      pose.id = oldPoseId + '_' + idUtils.generateShortId();
      oldToNewPoseId[oldPoseId] = pose.id;
      if (pose.parentFrameId === "ENTITY_BASE") {
        pose.parentFrameId = entity.frameId;      // the magic: relative to entity frame
      }
      poses[pose.id] = pose;

      if (modMatrix) {
        // modify the pose location/rotation by the modMatrix

        let poseMatrix = threeDUtils.posRotToMatrix4({pos: pose.pos, rotation: pose.rotation});
        poseMatrix.premultiply(modMatrix);

        let {pos, rotation} = threeDUtils.matrix4ToPosRot(poseMatrix);
        pose.pos = pos;
        pose.rotation = rotation;
      }

      // add to wpf
      wpfManager.addPose(pose);
    });

    Object.keys(templateJson.templateWaypoints).forEach((waypointKey) => {
      let waypoint = Waypoint.fromJSON(templateJson.templateWaypoints[waypointKey]);
      let oldWaypointId = waypoint.id;

      waypoint.id = oldWaypointId + '_' + idUtils.generateShortId();
      oldToNewWaypointId[oldWaypointId] = waypoint.id;
      waypoint.poseId = oldToNewPoseId[waypoint.poseId];
      waypoints[waypoint.id] = waypoint;

      // reset the joint

      let armMotionManager = comp.get(AmcConst.CompIds.armMotionManager);

      waypoint.jointHints = armMotionManager.getJointHints();
      waypoint.bias = armMotionManager.getJointHints();

      // add to wpf
      waypointPromises.push(wpfManager.addWaypoint(waypoint, true));
    });

    return Promise.all(waypointPromises).then(() => {
      // finally, fix up template variables and remove templatePoses and templateWaypoints

      Object.keys(templateJson.templateVariables).forEach((variableKey) => {
        let templateVariable = templateJson.templateVariables[variableKey];
        if (templateVariable.originalValue) {
          templateVariable.value = oldToNewWaypointId[templateVariable.originalValue];
          delete templateVariable.originalValue;
        }
      });

      delete templateJson.templatePoses;
      delete templateJson.templateWaypoints;

      return templateJson;
    });
  }
}


module.exports = {
  generateAffordanceTemplate
};