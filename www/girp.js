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
        b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
        b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
        b2Math = Box2D.Common.Math.b2Math,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        CATEGORY_HANDHOLD = 0x1,
        CATEGORY_PLAYER = 0x2,
        CATEGORY_STARTING_WELD = 0x4,
        undefined
        ;

    window.girpgame = function() {
        return this;
    };

    /**
     * Initialisation function to set up input listeners.
     * @param el
     */
    girpgame.prototype.bindInput = function(el) {
        var that = this;
        $(window).bind('keydown', function(event) {
            that.input(event.keyCode, 1);
        });
        $(window).bind('keyup', function(event) {
            that.input(event.keyCode, 0);
        });
    };


    girpgame.prototype.input = function(keyCode, down) {
        switch(keyCode) {
            case 82:
                this.leftArm = down;
                break;
            case 84:
                this.rightArm = down;
                break;
            case 17:
            case 32:
                this.heave = down;
                break;
            default:
                window.console.log("skipped " + event.keyCode);
        }

    };

    girpgame.prototype.initWorld = function(world) {
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
        this.leftArm = this.rightArm = this.heave = 0;

        /* nodes held by the relevant arm */
        this.leftArmNode = this.rightArmNode = 0;

        this.world = world;

        this.goal = new handhold(this.world, 50, 50);
        this.goal2 = new handhold(this.world, 550, 50);

        new handhold(this.world, 0, 0);

         /* Create the player */
        this.player = {};
        this.player.left = {};
        this.player.right = {};

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

    /**
     * Look out for contacts to attach hands to, and do the heaving / reaching
     */
    girpgame.prototype.tick = function() {
        var contact;
        var hold, arm;

        for(contact = this.world.GetContactList(); contact; contact = contact.next) {
            if (contact.m_fixtureA.m_body == this.goal.body) {
                hold = contact.m_fixtureA;
                arm = contact.m_fixtureB.m_body;
            } else if (contact.m_fixtureB.m_body == this.goal.body) {
                hold = contact.m_fixtureB;
                arm = contact.m_fixtureA.m_body;
            } else {
                hold = 0;
            }

            if (hold) {
                var wm = new b2WorldManifold();
                // wm.Initialize(contact, contact.m_fixtureA.m_body.m_xf, contact.m_fixtureA.m_shape.radius, contact.m_fixtureB.m_body.m_xf, contact.m_fixtureB.m_shape.radius);
                wm.Initialize(contact.m_manifold, contact.m_fixtureA.m_body.m_xf, contact.m_fixtureA.m_shape.radius, contact.m_fixtureB.m_body.m_xf, contact.m_fixtureB.m_shape.radius);
                if (arm == this.player.left.lowerArm && this.leftArm) {
                    var rjd;
                    rjd = new b2RevoluteJointDef;
                    rjd.Initialize(hold.m_body, arm, wm.m_points[0]);

                    this.leftArmNode = hold.m_body;
                    this.leftArmJoint = this.world.CreateJoint(rjd);

                    this.destroyWelds();
                } else if (arm == this.player.right.lowerArm) {
                    this.rightArm = 1;
                    this.rightArmNode = hold.m_body;
                }
            }
        }


        if (this.leftArm && !this.leftArmJoint) {
            this.reachFor(this.player.left.lowerArm, this.lowerArmLength, -1, this.goal);
            this.reachFor(this.player.left.upperArm, this.upperArmLength, -1, this.goal);
        } else if (!this.leftArm && this.leftArmJoint) {
            this.world.DestroyJoint(this.leftArmJoint);
            this.leftArmJoint = this.leftArmNode = undefined;
        }
        if (this.rightArm) {
            this.reachFor(this.player.right.lowerArm, this.lowerArmLength, 1, this.goal2);
            this.reachFor(this.player.right.upperArm, this.upperArmLength, 1, this.goal2);
        } else {
            if (this.rightArmJoint) {
                this.world.DestroyJoint(this.rightArmJoint);
                this.rightArmJoint = this.rightArmNode = undefined;
            }
        }

        this.doHeave(this.player.left, this.leftArmNode && this.heave);
        this.doHeave(this.player.right, this.rightArmNode && this.heave);
    };

    /**
     * Makes the specified arm part reach for the specified goal body.
     *
     * @param arm
     * @param armLength
     * @param goal
     */
    girpgame.prototype.reachFor = function(arm, armLength, dir, goal) {
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
    girpgame.prototype.doHeave = function(side, heave) {
        side.elbow.m_enableMotor = heave;
        //side.shoulder.m_enableMotor = heave;
    };

    girpgame.prototype.destroyWelds = function() {
        if (this.startingWeldAJoint) {
            this.world.DestroyJoint(this.startingWeldAJoint);
            this.world.DestroyJoint(this.startingWeldBJoint);
            this.world.DestroyBody(this.startingWeldA.body);
            this.world.DestroyBody(this.startingWeldB.body);

            this.startingWeldAJoint = this.startingWeldBJoint = this.startingWeldA = this.startingWeldB = undefined;
        }
    }


    /**
     * Create an arm and attach it to the torso.
     * @param dest an object that will hold the upperArm and lowerArm bodies.
     * @param dir  -1 or 1 for the left or right arm respectively.
     */
    girpgame.prototype.initArm = function(dest, dir) {
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

    girpgame.prototype.initLeg = function(dest, dir) {
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
     * in girpgame. Ahem.
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
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(10, 10);
        fixture.filter.categoryBits = category_bits;
        fixture.filter.isSensor = true;
        body.position.Set(x, y);
        this.body = this.world.CreateBody(body);
        this.body.CreateFixture(fixture);
    };


})(jQuery);

