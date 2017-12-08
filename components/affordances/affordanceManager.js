'use strict';

const EventHelper = require('../../../common/eventdispatcher/EventHelper.js');

const REngConst = require('../../../common/constants/RobotEngineConstants.js');
const EdConst = require('../../../common/constants/EventDispatcherConstants.js');
const AMConst = require('./AffordanceConstants.js');
const ConnConst = require('../../../common/constants/ConnectionManagerConstants.js');
const WpfConst = require('../../../common/constants/WpfManagerConstants.js');
const CameraUtils = require('../../../common/utils/camera_utils.js');
const comp = require('../../../common/ComponentManager.js');
const rosnodejs = require('rosnodejs');
const AmcConst = require('../../../common/constants/ArmMotionControllerConstants.js');
const TBConst = require('../../constants/TaskBrokerConstants.js');


/**
 * Manages RuntimeSettings (things that SHOULDN'T persist across reboot)
 */
class AffordanceManager {
  constructor() {
    this._eventHelper = new EventHelper();

    this.searching = false;
    this.landmarkId = null;
    this.subscriber = null;

    this._wpfManager = null;
    this._armMotionManager = null;

    this.affordanceList = [];

        ///affordances

  }

  init(options) {

    this._wpfManager = comp.get(WpfConst.CompIds.engWpfManager);
    this._armMotionManager = comp.get(AmcConst.CompIds.armMotionManager);

    this._taskBroker = comp.get(TBConst.CompIds.engTaskBroker);

    // for now, just turn streaming on all the time
    //this._taskBroker.setCameraStream('right_hand_camera',true,{ id : this.getComponentId()});

    const rosConn = comp.get(ConnConst.CompIds.connectionManager).getConnectionClient(ConnConst.Connections.ros);
    if (rosConn.isConnected()) {
      this._initializeRosConnections();
    }
    else {
      this._eventHelper.once(REngConst.Events.connectionChanged, REngConst.Topics.robotRos, this, this._handleRosConnectionChangedEvent);
    }

  }


  _handleRosConnectionChangedEvent(event) {
    this._initializeRosConnections()
  }

  _initializeRosConnections() {
    this.nh = rosnodejs.nh;
    this._setupAffordanceListener()
  }

  _setupAffordanceListener() {

    this.referenceJointStateListener = this.nh.subscribe('/affordances', 'std_msgs/String',
        (message) => {
          this.affordanceList = JSON.parse(message.data);


         // let data = this.getCurrentAffordances("landmark_2");
        }
    );

  //  this._log.info('Subscribing to Reference JointState ROS messages');
  }

  classToSubType(index) {

    if(index == 1) {
      return 'big_red_button';
    }
    if(index == 2) {
      return 'dull_green_button';
    }
    if(index == 3) {
      return 'estop_button';
    }
    if(index == 4) {
      return 'shiny_green_button';
    }
    if(index == 5) {
      return 'small_red_button';
    }
  }

  getFrameOffset(index) {

    if(index == 1) {
      //return 'big_red_button';
      return 10;
    }
    if(index == 2) {
      //return 'dull_green_button';
      return 5;
    }
    if(index == 3) {
      //return 'estop_button';
      return 70;
    }
    if(index == 4) {
      //return 'shiny_green_button';
      return 5;
    }
    if(index == 5) {
      //return 'small_red_button';
      return 5;
    }
  }

  getCurrentAffordances(landmarkId) {

    const cameraPose = this._armMotionManager.getCameraPose();
    const frame = this._wpfManager.getFrame(landmarkId);

    if(!frame) {
      return [];
    }

    // 752 x 480
    const imHeight = 480;
    const imWidth  = 752;

    let Affordances = [];

    for(let i = 0; i < this.affordanceList.length; i++) {

      const aData = this.affordanceList[i];

      let ymin = aData.box['ymin'] * imHeight;
      let xmin = aData.box['xmin'] * imWidth;
      let ymax = aData.box['ymax'] * imHeight;
      let xmax = aData.box['xmax'] * imWidth;

      const score = aData.score * 100;
      let bclass = aData.class;



      let yCenter = ((( ymin + ymax ) / 2) - (imHeight/2)) / 600;
      let xCenter = ((( xmin + xmax ) / 2) - (imWidth/2)) / 600;

      const detectionData = { x : xCenter, y : yCenter, angle : 0 };

      frame.transformTo(WpfConst.FrameIds.baseFrame)
      let nframe = CameraUtils.normalizedImageToWorld(detectionData, cameraPose, frame);
      if(!nframe) {
        continue;
      }

      nframe.pos.z = nframe.pos.z + (this.getFrameOffset(bclass) / 1000);

      Affordances.push( {
        score : score,
        type : AMConst.Types.button,
        subtype : this.classToSubType(bclass),
        frame : nframe
      });

    }

    return Affordances;
  }

  getComponentId() {
    return AMConst.CompIds.engAffordanceManager;
  }

}

module.exports = AffordanceManager;