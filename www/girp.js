/**
 * Created with JetBrains PhpStorm.
 * User: steven
 * Date: 28/05/12
 * Time: 19:24
 */

var girpgame = function() {
    return this;
};

girpgame.prototype.initWorld = function(world) {
    var shape;
    var body;
    var prevBody;

    /* all sizes used to define the shape of the player. */
    this.bodyCenter = { x: 350, y: 150 };
    this.bodySize = { w: 80, h: 120 };
    this.upperArmLength = 60;
    this.upperArmPos = { x: this.upperArmLength, y: 0.8 * this.bodySize.h / 2 };
    this.lowerArmLength = 80;

    this.world = world;

    /* Create a floor */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(1000, 5);
    shape.restitution = 0.7;
    body.AddShape(shape);
    body.position.Set(0, 600);
    this.world.CreateBody(body);

    /* Create the player */
    this.player = {};

    /* start with a torso */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(this.bodySize.w / 2, this.bodySize.h / 2);
    shape.density = 1;
    body.AddShape(shape);
    body.position.Set(this.bodyCenter.x, this.bodyCenter.y);
    this.player.torso = this.world.CreateBody(body);

    this.player.left = {};
    this.player.right = {};

    this.initArm(this.player.left, -1);
    this.initArm(this.player.right, 1);
};

girpgame.prototype.initArm = function(dest, dir) {
    var body;
    var shape;
    var rjd;
    var prevBody;

    /* make the upper arm */
    body = new b2BodyDef();
    body.position.Set(50, 50);
    shape = new b2BoxDef();
    shape.extents.Set(this.upperArmLength / 2, 6);
    shape.density = 1;
    body.AddShape(shape);
    body.position.Set(
        this.bodyCenter.x + dir * this.upperArmPos.x,
        this.bodyCenter.y - this.upperArmPos.y
    );
    dest.upperArm = this.world.CreateBody(body);

    /* connect it to the body */
    rjd = new b2RevoluteJointDef();
    rjd.anchorPoint.Set(
        this.bodyCenter.x + dir * this.upperArmPos.x - dir * this.upperArmLength / 2 * 0.9,
        this.bodyCenter.y - this.upperArmPos.y
    );
    rjd.body1 = this.player.torso;
    rjd.body2 = dest.upperArm;
    rjd.enableMotor = false;
    rjd.lowerAngle = -0.5 * 3.14159;
    rjd.upperAngle = 0.5 * 3.14159;
    rjd.enableLimit = true;
    this.world.CreateJoint(rjd);

    /* keep the ref to the upper arm def around so we can use it for position */
    prevBody = body;

    /* make the lower arm */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(40, 6);
    shape.density = 1;
    body.AddShape(shape);
    body.position.Set(
        prevBody.position.x + dir * this.upperArmLength / 2 + dir * this.lowerArmLength / 2,
        prevBody.position.y
    );
    dest.lowerArm = this.world.CreateBody(body);

    /* and connect it to the upper arm. */
    rjd = new b2RevoluteJointDef();
    rjd.anchorPoint.Set(
        body.position.x - dir * this.lowerArmLength / 2,
        body.position.y
    );
    rjd.body1 = dest.upperArm;
    rjd.body2 = dest.lowerArm;
    rjd.enableMotor = false;
    rjd.lowerAngle = -0.5 * 3.14159;
    rjd.upperAngle = 0.5 * 3.14159;
    rjd.enableLimit = true;
    this.world.CreateJoint(rjd);

};