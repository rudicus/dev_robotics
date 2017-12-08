const deepFreeze = require('deep-freeze');

module.exports = deepFreeze({

  Types : {
    button : 'button',
    toggle : 'toggle'
  },

  Subtypes : {
    subtype1 : 'subtype1'
  },

  CompIds : {
    engAffordanceRecorder : 'affordanceRecorder',
    engAffordanceManager: 'affordanceManager',
    apiAffordanceManager: 'affordanceManager.api',
  },

  Events : {
    affordanceRecordingStarted : 'affordanceRecordingStarted',
    affordanceRecordingFinished : 'affordanceRecordingFinished',
    affordanceRecordingToggleRequest : 'affordanceRecordingToggleRequest',
  },

  Topics: {
    affordances: 'affordances',
    uiAffordances: 'ui.affordances'
  },

  Affordances : {
    'button' : {
      'push' : {
        templateJson : require('../behaviortree/templates/affordances/ButtonPush.js')
      }
    }
  }


});