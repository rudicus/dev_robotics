
const VtConst = require('../../../common/constants/ValueTypeConstants.js');
const AfConst = require('./AffordanceConstants.js');

const SchemaModule = require('../../../common/schemas/Schema.js');
const { Schema, MEMBER, REFERENCEID, ENUMERATION } = SchemaModule;
const SchemaBase = require('../../../common/schemas/SchemaBase.js');
const SchemaConst = require('../../../common/schemas/SchemaConstants.js');
const SchemaRegistry = require('../../../common/schemas/SchemaRegistry.js');



/**
 * (Don't call constructor. Get it from the component manager.)
 * @constructor
 */
class Entity extends SchemaBase {

  constructor(context, id, name, type, subType, frameId) {

    super(context);

    if (!context) {
      this.id = id;
      this.name = name;
      this.type = type;
      this.subType = subType;
      this.frameId = frameId;
    }
  }

  toJSON() {
    let result = {};

    result.id = this.id;
    result.name = this.name;
    result.type = this.type;
    result.subType = this.subType;
    result.frameId = this.frameId;

    return result;
  }

  static fromJSON(data) {
    let entity = new Entity();
    entity.id = data.id;
    entity.name = data.name;
    entity.type = data.type;
    entity.subType = data.subType;
    entity.frameId = data.frameId;
    return entity;
  }
}

const EntitySchema = new Schema('Entity', Entity, 0, null);
EntitySchema.fields = [
  MEMBER('id', VtConst.Primitives.string, SchemaConst.SchemaAccess.readwrite, true),
  MEMBER('name', VtConst.Primitives.string, SchemaConst.SchemaAccess.readwrite, true),
  ENUMERATION('type', AfConst.Types, SchemaConst.SchemaAccess.readwrite, true),
  ENUMERATION('subType', AfConst.Subtypes, SchemaConst.SchemaAccess.readwrite, true),
  REFERENCEID('frameId', VtConst.Objects.Frame, SchemaConst.SchemaAccess.readwrite, true),
];
SchemaRegistry.RegisterSchema(EntitySchema);


module.exports = Entity;
