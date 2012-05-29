/*
 * This file is part of jsgirp
 * Copyright (c) 2003-2012 Steven Dickinson
 * Released under the terms of the GNU GPL version 3
 */

var girpgame = function() {
    return this;
};

girpgame.prototype.initWorld = function(world) {
    var shape;
    var body;
    var prevBody;

    /* all sizes used to define the shape of the player. */
    this.bodyCenter = { x: 220, y: 100 };
    this.bodySize = { w: 80, h: 120 };
    this.bodyAngularDamping = 0.9999;
    this.upperArmLength = 60;
    this.upperArmDensity = 0.6;
    this.upperArmPos = { x: this.upperArmLength, y: 0.8 * this.bodySize.h / 2 };
    this.lowerArmLength = 80;
    this.lowerArmDensity = 0.3;
    this.armAngularDamping = 1;

    /* input flags */
    this.leftArm = this.rightArm = 0;

    this.world = world;

    /* Create a floor */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(1000, 5);
    shape.restitution = 0.7;
    body.AddShape(shape);
    body.position.Set(0, 600);
    this.world.CreateBody(body);

    /* create a little target too */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(10, 10);
    body.AddShape(shape);
    body.position.Set(50, 50);
    this.goal = this.world.CreateBody(body);

    /* create another little target too */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(10, 10);
    body.AddShape(shape);
    body.position.Set(550, 50);
    this.goal2 = this.world.CreateBody(body);


    /* Create the player */
    this.player = {};

    /* start with a torso */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(this.bodySize.w / 2, this.bodySize.h / 2);
    shape.density = 1;
    body.AddShape(shape);
    body.angularDamping = this.bodyAngularDamping;
    body.position.Set(this.bodyCenter.x, this.bodyCenter.y);
    this.player.torso = this.world.CreateBody(body);

    this.player.left = {};
    this.player.right = {};

    this.initArm(this.player.left, -1);
    this.initArm(this.player.right, 1);

    /* hax0r fix left arm to a position */
    var rjd = new b2RevoluteJointDef();
    rjd.anchorPoint.Set(55, 55);
    rjd.body1 = this.player.left.lowerArm;
    rjd.body2 = this.goal;
    rjd.enableMotor = false;
    rjd.lowerAngle = -0.5 * 3.14159;
    rjd.upperAngle = 0.5 * 3.14159;
    rjd.enableLimit = true;
    this.world.CreateJoint(rjd);

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
    shape.density = this.upperArmDensity;
    body.AddShape(shape);
    body.position.Set(
        this.bodyCenter.x + dir * this.upperArmPos.x,
        this.bodyCenter.y - this.upperArmPos.y
    );
    body.angularDamping = this.armAngularDamping;
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
    shape.density = this.lowerArmDensity;
    body.AddShape(shape);
    body.position.Set(
        prevBody.position.x + dir * this.upperArmLength / 2 + dir * this.lowerArmLength / 2,
        prevBody.position.y
    );
    //body.angularDamping = this.armAngularDamping;
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

girpgame.prototype.tick = function() {
    if (this.leftArm) {
        this.reachFor(this.player.left.lowerArm, this.lowerArmLength, this.goal);
        this.reachFor(this.player.left.upperArm, this.upperArmLength, this.goal);
    }
    if (this.rightArm) {
        this.reachFor(this.player.right.lowerArm, this.lowerArmLength, this.goal2);
        this.reachFor(this.player.right.upperArm, this.upperArmLength, this.goal2);
    }
}

girpgame.prototype.reachFor = function(arm, armLength, goal) {
    var forcePos = b2Math.AddVV(arm.m_position, b2Math.b2MulMV(arm.m_R, { y: 0, x: armLength / 2}));
    var goalPosition = goal.m_position.Copy();
    var force = b2Math.SubtractVV(goalPosition, arm.m_position);
    force.Normalize();

    var armDir = b2Math.b2MulMV(arm.m_R, { y: 0, x: armLength / 2});
    armDir.Normalize();
    drawVector(forcePos.x, forcePos.y, forcePos.x + 50 * armDir.x, forcePos.y + 50 * armDir.y, "#ff00ff");

    //var dp = 1 - Math.abs(b2Math.b2Dot(force, armDir));
    var dp = 1;

    //window.console.log(dp);

    drawVector(forcePos.x, forcePos.y, forcePos.x + dp * 50 * force.x, forcePos.y + dp * 50 * force.y, "#ff");

    force.Multiply(dp * 325000);

    arm.WakeUp();

    arm.ApplyForce(force, forcePos);
};

girpgame.prototype.bindInput = function(el) {
    Event.observe(window, 'keydown', function(event) {
        this.input(event.keyCode, 1);
    }.bind(this));
    Event.observe(window, 'keyup', function(event) {
        this.input(event.keyCode, 0);
    }.bind(this));
};


girpgame.prototype.input = function(keyCode, down) {
    switch(keyCode) {
    case 82:
        this.leftArm = down;
        break;
    case 84:
        this.rightArm = down;
        break;
    default:
        window.console.log("skipped " + event.keyCode);
    }

};