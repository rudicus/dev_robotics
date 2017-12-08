"use strict";

const apiUtils = require('../../../common/utils/api_utils.js');
const EngineCommands = require('../../../common/constants/EngineCommandConstants.js');


class AffordanceManagerApi {

  constructor() {
  }

  getComponentId() {
    return 'affordanceManager.api';
  }

  getCurrentAffordances(frameId) {
    return apiUtils.sendCommand(EngineCommands.Affordance.getCurrentAffordances, { frameId }, null, this);
  }

}

module.exports = AffordanceManagerApi;

