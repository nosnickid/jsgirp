/*
 * This file is part of jsgirp
 * Copyright (c) 2003-2012 Steven Dickinson
 * Released under the terms of the GNU GPL version 3
 */

(function($) {

    var b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2World = Box2D.Dynamics.b2World,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2Body = Box2D.Dynamics.b2Body,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
        b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
        b2Math = Box2D.Common.Math.b2Math,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        CATEGORY_HANDHOLD = 0x1,
        CATEGORY_PLAYER = 0x2,
        CATEGORY_STARTING_WELD = 0x4,
        undefined
        ;

    window.GirpGame = function() {
        this._renderCallback = undefined;
        this._frame = undefined;

        return this;
    };

    /**
     * Initialisation function to set up input listeners.
     * @param el
     */
    GirpGame.prototype.bindInput = function(el) {
        var that = this;
        $(window).bind('keydown', function(event) {
            that._onInput(event.keyCode, 1);
        });
        $(window).bind('keyup', function(event) {
            that._onInput(event.keyCode, 0);
        });
    };

    GirpGame.prototype._onInput = function(keyCode, down) {
        switch(keyCode) {
            case 82:
                this.input.leftArm = down;
                break;
            case 84:
                this.input.rightArm = down;
                break;
            case 17:
            case 32:
                this.heave = down;
                break;
            default:
                window.console.log("skipped " + event.keyCode);
        }

    };

    GirpGame.prototype.initWorld = function(playerDef) {
        var fixture;
        var body;
        var gravity;
        var doSleep;

        this.playerDef = playerDef;

        /* input flags */
        this.input = {
            leftArm: 0,
            rightArm: 0,
            heave: 0
        };

        gravity = new b2Vec2(0, 300);
        doSleep = true;

        this.world = new b2World(gravity, doSleep);

        this.listener = new b2ContactListener();
        this.listener.BeginContact = this._onBeginContact.bind(this);
        this.world.SetContactListener(this.listener);

        this.goal = new handhold(this.world, 50, 50);
        this.goal2 = new handhold(this.world, 350, 50);

        //new handhold(this.world, 0, 0);

         /* Create the player */
        this.player = {};
        this.player.left = { dir: 1 };
        this.player.right = { dir: -1 };

        /* nodes held by the relevant arm */
        this.player.left.armNode = this.player.right.armNode = undefined;

        /* start with a torso */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.bodySizeWidth / 2, this.playerDef.bodySizeHeight / 2);
        fixture.density = this.playerDef.torsoDensity;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        body.angularDamping = this.playerDef.bodyAngularDamping;
        body.position.Set(this.playerDef.bodyCenterX, this.playerDef.bodyCenterY);
        this.player.torso = this.world.CreateBody(body);
        this.player.torso.CreateFixture(fixture);

        this._initArm(this.player.left, -1);
        this._initArm(this.player.right, 1);

        this._initLeg(this.player.left, -1);
        this._initLeg(this.player.right, 1);

        /* start the game with the body welded to a fixed spot. */
        this.startingWeldA = new handhold(this.world, this.playerDef.bodyCenterX + 5, this.playerDef.bodyCenterY, CATEGORY_STARTING_WELD);
        this.startingWeldB = new handhold(this.world, this.playerDef.bodyCenterX - 5, this.playerDef.bodyCenterY, CATEGORY_STARTING_WELD);

        var rjd = new b2WeldJointDef();
        rjd.Initialize(this.player.torso, this.startingWeldA.body, this.startingWeldA.body.m_xf.position);
        this.startingWeldAJoint = this.world.CreateJoint(rjd);
        rjd.Initialize(this.player.torso, this.startingWeldB.body, this.startingWeldB.body.m_xf.position);
        this.startingWeldBJoint = this.world.CreateJoint(rjd);
    };

    GirpGame.prototype.setRenderCallback = function(fn) {
        this._renderCallback = fn;
    };

    /**
     * Start the animation loop and play the game.
     */
    GirpGame.prototype.start = function() {
        this.runFrame(true);
    };

    /**
     * Cancel the animation loop if it's running.
     */
    GirpGame.prototype.stop = function() {
        if (this._frame) window.cancelAnimationFrame(this._frame);
    };

    /**
     * Tick the world and do a render if a callback is present.
     *
     * @param runAnother
     */
    GirpGame.prototype.runFrame = function(runAnother) {
        /* horrendous? */
        if (runAnother) this._frame = window.requestAnimationFrame(function() { this.runFrame(true)}.bind(this));

        this._tick();
        if (this._renderCallback) this._renderCallback();
    };

    GirpGame.prototype._onBeginContact = function(contact, manifold) {
        var hold, arm;

        if (contact.m_manifold == undefined) return;

        if (contact.m_fixtureA.m_body == this.goal.body || contact.m_fixtureA.m_body == this.goal2.body) {
            hold = contact.m_fixtureA;
            arm = contact.m_fixtureB.m_body;
        } else if (contact.m_fixtureB.m_body == this.goal.body || contact.m_fixtureB.m_body == this.goal2.body) {
            hold = contact.m_fixtureB;
            arm = contact.m_fixtureA.m_body;
        } else {
            hold = 0;
        }

        if (hold) {
            var wm = new b2WorldManifold();
            wm.Initialize(contact.m_manifold, contact.m_fixtureA.m_body.m_xf, contact.m_fixtureA.m_shape.m_radius, contact.m_fixtureB.m_body.m_xf, contact.m_fixtureB.m_shape.m_radius);
            if (arm == this.player.left.lowerArm && this.input.leftArm) {
                this.player.left.armNode = hold;
                this.player.left.armJointPos = wm.m_points[0];
            } else if (arm == this.player.right.lowerArm && this.input.rightArm) {
                this.player.right.armNode = hold;
                this.player.right.armJointPos = wm.m_points[0];
            }
        }

    };

    /**
     * Look out for contacts to attach hands to, and do the heaving / reaching
     */
    GirpGame.prototype._tick = function() {
        var timeStep = 1.0/60;
        var velocityIterations = 5;
        var positionIterations = 5;

        this._tickArm(this.input.leftArm, this.player.left, this.goal);
        this._tickArm(this.input.rightArm, this.player.right, this.goal2);

        this._doHeave(this.player.left, this.player.left.armNode && this.heave);
        this._doHeave(this.player.right, this.player.right.armNode && this.heave);

        this.world.Step(timeStep, velocityIterations, positionIterations);
        this.world.ClearForces();
    };

    /**
     * Manage reaching for a node, creating a rotate joint if we grabbed a hold in the last frame, and removing
     * the joint if we let go.
     *
     * @param input boolean whether the key is down
     * @param side  this.player.left or .right, all the relevant box2d objects.
     * @param goal  the goal that the player should reach for with this arm.
     */
    GirpGame.prototype._tickArm = function(input, side, goal) {
        if (input && side.armNode && !side.armJoint) {
            // last tick they hit something, so connect them up!
            var rjd;
            rjd = new b2RevoluteJointDef;
            rjd.Initialize(side.armNode.m_body, side.lowerArm, side.armJointPos);

            side.armNode.m_body.m_color = "#ee00ee";
            side.armJoint = this.world.CreateJoint(rjd);

            this._destroyWelds();
        } else if (input && !side.armNode) {
            // they're reaching!
            this._reachForWithHand(side, goal);
        } else if (!input && side.armJoint) {
            // they let go!
            this.world.DestroyJoint(side.armJoint);
            side.armJoint = side.armNode = undefined;
        }
    };

    /**
     * Apply a force on the specified arm to reach for the specified goal.
     *
     * @param side
     * @param goal
     * @private
     */
    GirpGame.prototype._reachForWithHand = function(side, goal) {
        var forceDir;
        var pos = side.lowerArm.m_xf.position.Copy();

        pos.Add(b2Math.MulMV(side.lowerArm.m_xf.R, side.handShape.GetLocalPosition()));

        forceDir = b2Math.SubtractVV(goal.body.m_xf.position, pos);
        forceDir.Normalize();
        forceDir.Multiply(this.playerDef.reachForce);

        side.lowerArm.ApplyForce(forceDir, pos);
    };

    /**
     * if heave, pull up on the specified side.
     * @param side
     * @param heave
     */
    GirpGame.prototype._doHeave = function(side, heave) {
        side.elbow.m_enableMotor = heave;
        //side.shoulder.m_enableMotor = heave;
    };

    GirpGame.prototype._destroyWelds = function() {
        if (this.startingWeldAJoint) {
            this.world.DestroyJoint(this.startingWeldAJoint);
            this.world.DestroyJoint(this.startingWeldBJoint);
            this.world.DestroyBody(this.startingWeldA.body);
            this.world.DestroyBody(this.startingWeldB.body);

            this.startingWeldAJoint = this.startingWeldBJoint = this.startingWeldA = this.startingWeldB = undefined;
        }
    };


    /**
     * Create an arm and attach it to the torso.
     * @param dest an object that will hold the upperArm and lowerArm bodies.
     * @param dir  -1 or 1 for the left or right arm respectively.
     */
    GirpGame.prototype._initArm = function(dest, dir) {
        var body;
        var fixture;
        var rjd;
        var anchor;
        var prevBody;

        //<editor-fold desc="Upper arm and shoulder">
        /* make the upper arm */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.playerDef.bodyCenterX + dir * this.playerDef.upperArmPosX,
            this.playerDef.bodyCenterY - this.playerDef.upperArmPosY
        );
        body.angularDamping = this.playerDef.armAngularDamping;
        dest.upperArm = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.upperArmLength / 2, 6);
        fixture.density = this.playerDef.upperArmDensity;
        fixture.filter.maskBits = 0;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.upperArm.CreateFixture(fixture);

        /* connect it to the body - SHOULDER joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            this.playerDef.bodyCenterX + dir * this.playerDef.upperArmPosX - dir * this.playerDef.upperArmLength / 2 * 0.9,
            this.playerDef.bodyCenterY - this.playerDef.upperArmPosY
        );
        rjd.Initialize(this.player.torso, dest.upperArm, anchor);
        rjd.enableMotor = false;
        rjd.lowerAngle = -0.9 * 3.14159;
        rjd.upperAngle = 0.9 * 3.14159;
        rjd.enableLimit = true;
        //rjd.motorSpeed = -0.1;
        rjd.maxMotorTorque = this.playerDef.elbowMaxTorque;
        dest.shoulder = this.world.CreateJoint(rjd);
        //</editor-fold>

        /* keep the ref to the upper arm def around so we can use it for position */
        prevBody = body;

        //<editor-fold desc="lower arm and elbow">
        /* make the lower arm */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            prevBody.position.x + dir * this.playerDef.upperArmLength / 2 + dir * this.playerDef.lowerArmLength / 2,
            prevBody.position.y
        );
        dest.lowerArm = this.world.CreateBody(body);

        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.lowerArmLength / 2, 6);
        fixture.density = this.playerDef.lowerArmDensity;
        fixture.filter.maskBits = 0;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        //body.angularDamping = this.playerDef.armAngularDamping;
        dest.lowerArm.CreateFixture(fixture);

        /* and connect it to the upper arm. - ELBOW joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x - dir * this.playerDef.lowerArmLength / 2,
            body.position.y
        );
        rjd.Initialize(dest.upperArm, dest.lowerArm, anchor);
        rjd.enableMotor = false;
        rjd.enableLimit = true;
        rjd.motorSpeed = -dir * this.playerDef.elbowMotorSpeed;
        rjd.maxMotorTorque = this.playerDef.elbowMaxTorque;
        if (dir < 0) {
            rjd.lowerAngle = 0;
            rjd.upperAngle = 0.5 * 3.14159;
        } else {
            rjd.lowerAngle = -0.5 * 3.14159;
            rjd.upperAngle = 0;
        }
        rjd.enableLimit = true;
        dest.elbow = this.world.CreateJoint(rjd);
        //</editor-fold>

        /* keep the ref for the lower arm around to find the hand position */
        prevBody = body;

        /* create a hand sensor, so that he only grabs hold using the hand. */
        /* position it right at the end of the arm. */
        fixture = new b2FixtureDef();
        fixture.shape = new b2CircleShape();
        fixture.shape.SetRadius(5);
        fixture.shape.density = 0;
        fixture.shape.SetLocalPosition(new b2Vec2(
            dir * this.playerDef.lowerArmLength / 2,
            0
        ));
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        fixture.filter.isSensor = true;
        dest.handShape = dest.lowerArm.CreateFixture(fixture).GetShape();
    };

    /**
     * Create a leg and attach it to the torso.
     * @param dest an object that will hold the arm bodies.
     * @param dir  -1 or 1 for the left or right leg respectively.
     */

    GirpGame.prototype._initLeg = function(dest, dir) {
        var body;
        var fixture;
        var rjd;
        var prevBody;
        var anchor;

        /* make the thigh */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.playerDef.bodyCenterX + dir * this.playerDef.thighPosX,
            this.playerDef.bodyCenterY + this.playerDef.thighPosY
        );
        body.angularDamping = this.playerDef.thighAngularDamping;
        dest.thigh = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.thighWidth, this.playerDef.thighLength / 2);
        fixture.density = this.playerDef.thighDensity;
        fixture.filter.maskBits = 0;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.thigh.CreateFixture(fixture);


        /* connect it to the body - HIP JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.playerDef.thighLength / 2
        );
        rjd.Initialize(this.player.torso, dest.thigh, anchor);
        rjd.enableMotor = false;
        if (dir < 0) {
            rjd.lowerAngle = dir * this.playerDef.hipMinAngle;
            rjd.upperAngle = dir * this.playerDef.hipMaxAngle;
        } else {
            rjd.lowerAngle = dir * this.playerDef.hipMaxAngle;
            rjd.upperAngle = dir * this.playerDef.hipMinAngle;
        }
        rjd.enableLimit = true;

        dest.hip = this.world.CreateJoint(rjd);

        /* keep the ref to the thigh def around so we can use it for position */
        prevBody = body;

        /* make the calf */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.playerDef.bodyCenterX + dir * this.playerDef.thighPosX,
            this.playerDef.bodyCenterY + this.playerDef.thighPosY + this.playerDef.thighLength / 2 + this.playerDef.calfLength / 2
        );
        body.angularDamping = this.playerDef.calfAngularDamping;
        dest.calf = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.calfWidth, this.playerDef.calfLength / 2);
        fixture.density = this.playerDef.calfDensity;
        fixture.filter.maskBits = 0;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.calf.CreateFixture(fixture);

        /* and connect it to the thigh. - KNEE JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.playerDef.calfLength / 2
        );
        rjd.Initialize(dest.thigh, dest.calf, anchor);
        rjd.enableMotor = false;
        if (dir < 0) {
            rjd.lowerAngle = -0.5 * 3.14159;
            rjd.upperAngle = 0;
        } else {
            rjd.lowerAngle = 0;
            rjd.upperAngle = 0.5 * 3.14159;
        }
        rjd.enableLimit = true;
        dest.knee = this.world.CreateJoint(rjd);

    };


    /**
     * Encapsulate a hand hold. Doesn't actually do much and should probably just be a function
     * in GirpGame. Ahem.
     *
     * @param world  box2d physics world
     * @param x      xpos
     * @param y      ypos
     */
    var handhold = function(world, x, y, category_bits) {
        var body;
        var fixture;

        if (category_bits == undefined) category_bits = CATEGORY_HANDHOLD;

        body = new b2BodyDef();
        body.type = b2Body.b2_staticBody;
        fixture = new b2FixtureDef();
        fixture.shape = new b2CircleShape();
        fixture.shape.SetRadius(5);
        fixture.filter.categoryBits = category_bits;
        fixture.filter.isSensor = true;
        body.position.Set(x, y);
        this.body = world.CreateBody(body);
        this.body.CreateFixture(fixture);
    };

    window.GirpPlayerDef = function() {
        /* all sizes used to define the shape of the player. */
        this.bodyCenterX = 220;
        this.bodyCenterY = 100;
        this.bodySizeWidth = 80;
        this.bodySizeHeight = 120;
        this.torsoDensity = 1;

        this.bodyAngularDamping = 0.9999;
        /* arm setup */
        this.upperArmLength = 60;
        this.upperArmDensity = 0.6;
        this.upperArmPosX = this.upperArmLength;
        this.upperArmPosY = 0.8 * this.bodySizeHeight / 2;

        this.lowerArmLength = 80;
        this.lowerArmDensity = 0.3;
        this.armAngularDamping = 1;

        this.elbowMaxTorque = 100000000000;
        this.elbowMotorSpeed = 15;
        this.reachForce = 425000;

        /* leg setup */
        this.thighLength = 80;
        this.thighWidth = 6;
        this.thighDensity = 0.6;
        this.thighAngularDamping = 1;
        this.thighPosX = 0.7 * this.bodySizeWidth / 2;
        this.thighPosY = 1.4 * this.bodySizeHeight / 2

        this.hipMinAngle = 0;
        this.hipMaxAngle = -4;

        this.calfLength = 80;
        this.calfWidth = 6;
        this.calfDensity = 1;
        this.calfAngularDamping = 0;
    };

})(jQuery);

