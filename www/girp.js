/*
 * This file is part of jsgirp
 * Copyright (c) 2003-2012 Steven Dickinson
 * Released under the terms of the GNU GPL version 3
 */

(function($) {

    var b2Vec2 = Box2D.Common.Math.b2Vec2,
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
        return this;
    };

    /**
     * Initialisation function to set up input listeners.
     * @param el
     */
    GirpGame.prototype.bindInput = function(el) {
        var that = this;
        $(window).bind('keydown', function(event) {
            that.onInput(event.keyCode, 1);
        });
        $(window).bind('keyup', function(event) {
            that.onInput(event.keyCode, 0);
        });
    };


    GirpGame.prototype.onInput = function(keyCode, down) {
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

    GirpGame.prototype.initWorld = function(world) {
        var fixture;
        var body;
        var prevBody;

        /* all sizes used to define the shape of the player. */
        this.bodyCenter = { x: 220, y: 100 };
        this.bodySize = { w: 80, h: 120 };
        this.bodyAngularDamping = 0.9999;
        /* arm setup */
        this.upperArmLength = 60;
        this.upperArmDensity = 0.6;
        this.upperArmPos = {
            x: this.upperArmLength,
            y: 0.8 * this.bodySize.h / 2
        };

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
        this.thighPos = {
            x: 0.7 * this.bodySize.w / 2,
            y: 1.4 * this.bodySize.h / 2
        };
        this.calfLength = 80;
        this.calfWidth = 6;
        this.calfDensity = 1;

        /* input flags */
        this.input = {
            leftArm: 0,
            rightArm: 0,
            heave: 0
        };

        this.world = world;

        this.listener = new b2ContactListener();
        this.listener.BeginContact = this.onBeginContact.bind(this);
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
        fixture.shape.SetAsBox(this.bodySize.w / 2, this.bodySize.h / 2);
        fixture.density = 1;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        body.angularDamping = this.bodyAngularDamping;
        body.position.Set(this.bodyCenter.x, this.bodyCenter.y);
        this.player.torso = this.world.CreateBody(body);
        this.player.torso.CreateFixture(fixture);

        this.initArm(this.player.left, -1);
        this.initArm(this.player.right, 1);

        this.initLeg(this.player.left, -1);
        this.initLeg(this.player.right, 1);

        /* start the game with the body welded to a fixed spot. */
        this.startingWeldA = new handhold(this.world, this.bodyCenter.x + 5, this.bodyCenter.y, CATEGORY_STARTING_WELD);
        this.startingWeldB = new handhold(this.world, this.bodyCenter.x - 5, this.bodyCenter.y, CATEGORY_STARTING_WELD);

        var rjd = new b2WeldJointDef();
        rjd.Initialize(this.player.torso, this.startingWeldA.body, this.startingWeldA.body.m_xf.position);
        this.startingWeldAJoint = this.world.CreateJoint(rjd);
        rjd.Initialize(this.player.torso, this.startingWeldB.body, this.startingWeldB.body.m_xf.position);
        this.startingWeldBJoint = this.world.CreateJoint(rjd);
    };

    GirpGame.prototype.onBeginContact = function(contact, manifold) {
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
    GirpGame.prototype.tick = function() {

        this.tickArm(this.input.leftArm, this.player.left, this.goal);
        this.tickArm(this.input.rightArm, this.player.right, this.goal2);

        this.doHeave(this.player.left, this.player.left.armNode && this.heave);
        this.doHeave(this.player.right, this.player.right.armNode && this.heave);
    };

    /**
     * Manage reaching for a node, creating a rotate joint if we grabbed a hold in the last frame, and removing
     * the joint if we let go.
     *
     * @param input boolean whether the key is down
     * @param side  this.player.left or .right, all the relevant box2d objects.
     * @param goal  the goal that the player should reach for with this arm.
     */
    GirpGame.prototype.tickArm = function(input, side, goal) {
        if (input && side.armNode && !side.armJoint) {
            // last tick they hit something, so connect them up!
            var rjd;
            rjd = new b2RevoluteJointDef;
            rjd.Initialize(side.armNode.m_body, side.lowerArm, side.armJointPos);

            side.armNode.m_body.m_color = "#ee00ee";
            side.armJoint = this.world.CreateJoint(rjd);

            this.destroyWelds();
        } else if (input && !side.armNode) {
            // they're reaching!
            this.reachFor(side.lowerArm, this.lowerArmLength, side.dir, goal);
            this.reachFor(side.upperArm, this.upperArmLength, side.dir, goal);
        } else if (!input && side.armJoint) {
            // they let go!
            this.world.DestroyJoint(side.armJoint);
            side.armJoint = side.armNode = undefined;
        }
    };

    /**
     * Makes the specified arm part reach for the specified goal body.
     *
     * @param arm
     * @param armLength
     * @param goal
     */
    GirpGame.prototype.reachFor = function(arm, armLength, dir, goal) {
        var forcePos = b2Math.MulX(arm.m_xf, { y: 0, x: dir * armLength / 2});
        var goalPosition = goal.body.m_xf.position.Copy();
        var force = b2Math.SubtractVV(goalPosition, arm.m_xf.position);
        force.Normalize();

        var armDir = b2Math.MulMV(arm.m_xf.R, new b2Vec2(dir * armLength / 2, 0));
        armDir.Normalize();

        force.Multiply(this.reachForce);

        arm.ApplyForce(force, forcePos);
    };

    /**
     * if heave, pull up on the specified side.
     * @param side
     * @param heave
     */
    GirpGame.prototype.doHeave = function(side, heave) {
        side.elbow.m_enableMotor = heave;
        //side.shoulder.m_enableMotor = heave;
    };

    GirpGame.prototype.destroyWelds = function() {
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
    GirpGame.prototype.initArm = function(dest, dir) {
        var body;
        var fixture;
        var rjd;
        var anchor;
        var prevBody;

        /* make the upper arm */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.bodyCenter.x + dir * this.upperArmPos.x,
            this.bodyCenter.y - this.upperArmPos.y
        );
        body.angularDamping = this.armAngularDamping;
        dest.upperArm = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.upperArmLength / 2, 6);
        fixture.density = this.upperArmDensity;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.upperArm.CreateFixture(fixture);

        /* connect it to the body - SHOULDER joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            this.bodyCenter.x + dir * this.upperArmPos.x - dir * this.upperArmLength / 2 * 0.9,
            this.bodyCenter.y - this.upperArmPos.y
        );
        rjd.Initialize(this.player.torso, dest.upperArm, anchor);
        rjd.enableMotor = false;
        rjd.lowerAngle = -0.9 * 3.14159;
        rjd.upperAngle = 0.9 * 3.14159;
        rjd.enableLimit = true;
        //rjd.motorSpeed = -0.1;
        rjd.maxMotorTorque = this.elbowMaxTorque;
        dest.shoulder = this.world.CreateJoint(rjd);

        /* keep the ref to the upper arm def around so we can use it for position */
        prevBody = body;

        /* make the lower arm */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            prevBody.position.x + dir * this.upperArmLength / 2 + dir * this.lowerArmLength / 2,
            prevBody.position.y
        );
        dest.lowerArm = this.world.CreateBody(body);

        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.lowerArmLength / 2, 6);
        fixture.density = this.lowerArmDensity;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        //body.angularDamping = this.armAngularDamping;
        dest.lowerArm.CreateFixture(fixture);

        /* and connect it to the upper arm. - ELBOW joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x - dir * this.lowerArmLength / 2,
            body.position.y
        );
        rjd.Initialize(dest.upperArm, dest.lowerArm, anchor);
        rjd.enableMotor = false;
        rjd.enableLimit = true;
        rjd.motorSpeed = -dir * this.elbowMotorSpeed;
        rjd.maxMotorTorque = this.elbowMaxTorque;
        if (dir < 0) {
            rjd.lowerAngle = 0;
            rjd.upperAngle = 0.5 * 3.14159;
        } else {
            rjd.lowerAngle = -0.5 * 3.14159;
            rjd.upperAngle = 0;
        }
        rjd.enableLimit = true;
        dest.elbow = this.world.CreateJoint(rjd);
    };

    /**
     * Create a leg and attach it to the torso.
     * @param dest an object that will hold the arm bodies.
     * @param dir  -1 or 1 for the left or right leg respectively.
     */

    GirpGame.prototype.initLeg = function(dest, dir) {
        var body;
        var fixture;
        var rjd;
        var prevBody;
        var anchor;

        /* make the thigh */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.bodyCenter.x + dir * this.thighPos.x,
            this.bodyCenter.y + this.thighPos.y
        );
        body.angularDamping = this.thighAngularDamping;
        dest.thigh = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.thighWidth, this.thighLength / 2);
        fixture.density = this.thighDensity;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.thigh.CreateFixture(fixture);


        /* connect it to the body - HIP JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.thighLength / 2
        );
        rjd.Initialize(this.player.torso, dest.thigh, anchor);
        rjd.enableMotor = false;
        //rjd.lowerAngle = -0.5 * 3.14159;
        //rjd.upperAngle = 0.5 * 3.14159;
        //rjd.enableLimit = true;
        dest.hip = this.world.CreateJoint(rjd);

        /* keep the ref to the thigh def around so we can use it for position */
        prevBody = body;

        /* make the calf */
        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        body.position.Set(
            this.bodyCenter.x + dir * this.thighPos.x,
            this.bodyCenter.y + this.thighPos.y + this.thighLength
        );
        dest.calf = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.calfWidth, this.calfLength / 2);
        fixture.density = this.calfDensity;
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.calf.CreateFixture(fixture);

        /* and connect it to the thigh. - KNEE JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.thighLength / 2
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

        this.world = world;

        /* create a little target too */
        body = new b2BodyDef();
        body.type = b2Body.b2_staticBody;
        fixture = new b2FixtureDef();
        fixture.shape = new b2CircleShape();
        fixture.shape.SetRadius(5);
        fixture.filter.categoryBits = category_bits;
        fixture.filter.isSensor = true;
        body.position.Set(x, y);
        this.body = this.world.CreateBody(body);
        this.body.CreateFixture(fixture);
    };


})(jQuery);

