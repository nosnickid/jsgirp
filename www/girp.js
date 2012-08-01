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
        CATEGORY_HAND_LEFT      = 0x1,
        CATEGORY_HAND_RIGHT     = 0x2,
        CATEGORY_PLAYER         = 0x4,
        CATEGORY_STARTING_WELD  = 0x8,
        CATEGORY_WORLD          = 0x10,
        CATEGORY_HANDHOLD       = 0x20,
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

        if (keyCode == 32) {
            this.input.heave = down;
        } else if (down) {
            if (this._keys[keyCode] != undefined) {
                var hold = this._keys[keyCode];

                if (keyCode == this.input.rightButton || keyCode == this.input.leftButton) {
                    // ignore key repeat inputs.
                } else if (this.player.left.armJoint && !this.player.right.armJoint) {
                    // Reach with the right arm.
                    this.goalRight = hold;
                    hold.body.m_fixtureList.m_filter.maskBits |= CATEGORY_HAND_RIGHT;
                    this.input.rightButton = keyCode;
                    this.input.rightArm = true;
                    window.console.log("reaching right to " + String.fromCharCode(keyCode))
                } else if (!this.player.left.armJoint && this.player.right.armJoint) {
                    // Reach with the left arm.
                    this.goalLeft = hold;
                    hold.body.m_fixtureList.m_filter.maskBits |= CATEGORY_HAND_LEFT;
                    this.input.leftButton = keyCode;
                    this.input.leftArm = true;
                    window.console.log("reaching left to " + String.fromCharCode(keyCode))
                } else if (!this.player.left.armJoint && !this.player.right.armJoint) {
                    if (hold.body.m_xf.position.x < this.player.torso.m_xf.position.x) {
                        this.goalLeft = hold;
                        hold.body.m_fixtureList.m_filter.maskBits |= CATEGORY_HAND_LEFT;
                        this.input.leftButton = keyCode;
                        this.input.leftArm = true;
                        window.console.log("reaching left to " + String.fromCharCode(keyCode))
                    } else {
                        this.goalRight = hold;
                        hold.body.m_fixtureList.m_filter.maskBits |= CATEGORY_HAND_RIGHT;
                        this.input.rightButton = keyCode;
                        this.input.rightArm = true;
                        window.console.log("reaching right to " + String.fromCharCode(keyCode))
                    }
                }
            }
        } else if (!down) {
            if (this.input.leftButton == keyCode) {
                this.goalLeft.body.m_fixtureList.m_filter.maskBits = 0;
                this.goalLeft = this.input.leftButton = undefined;
                this.input.leftArm = false;
                window.console.log("released left: " + String.fromCharCode(keyCode))
            } else if (this.input.rightButton == keyCode) {
                this.goalRight.body.m_fixtureList.m_filter.maskBits = 0;
                this.goalRight = this.input.rightButton = undefined;
                this.input.rightArm = false;
                window.console.log("released right: " + String.fromCharCode(keyCode))
            }
        }

        return true;

    };

    GirpGame.prototype.initWorld = function(playerDef) {
        var gravity;
        var doSleep;

        this.playerDef = playerDef;

        this._keysStart = 65;
        this._keysRange = 90 - this._keysStart;
        this._keys = {};
        for (var i = 0; i <= this._keysRange; i++) this._keys[this._keysStart + i] = undefined;
        this._handHolds = [];
        this._handHoldBindDistance = 3;

        /* input flags */
        this.input = {
            leftArm: false,
            rightArm: false,
            heave: 0,
            leftButton: 0,
            rightButton: 0
        };

        gravity = new b2Vec2(0, 9.81);
        doSleep = true;

        this.world = new b2World(gravity, doSleep);

        this.listener = new b2ContactListener();
        this.listener.BeginContact = this._onBeginContact.bind(this);
        this.world.SetContactListener(this.listener);

        /* Create the player */
        this.player = {};
        this.player.left = { dir: 1 };
        this.player.right = { dir: -1 };

        /* nodes held by the relevant arm */
        this.player.left.armNode = this.player.right.armNode = undefined;

        /* start with a torso */
        this._initTorso();

        this._initArm(this.player.left, -1);
        this._initArm(this.player.right, 1);

        this._initLeg(this.player.left, -1);
        this._initLeg(this.player.right, 1);

        /* start the game with the body welded to a fixed spot. */
        this.startingWeldA = new handhold(this.world, this.playerDef.torsoCenterX + .05, this.playerDef.torsoCenterY, this.playerDef.handRadius, CATEGORY_STARTING_WELD);
        this.startingWeldB = new handhold(this.world, this.playerDef.torsoCenterX - .05, this.playerDef.torsoCenterY, this.playerDef.handRadius, CATEGORY_STARTING_WELD);

        var rjd = new b2WeldJointDef();
        rjd.Initialize(this.player.torso, this.startingWeldA.body, this.startingWeldA.body.m_xf.position);
        this.startingWeldAJoint = this.world.CreateJoint(rjd);
        rjd.Initialize(this.player.torso, this.startingWeldB.body, this.startingWeldB.body.m_xf.position);
        this.startingWeldBJoint = this.world.CreateJoint(rjd);
    };

    GirpGame.prototype.makeGridInputs = function() {
        var x, y;
        for(x = -10; x < 20; x++) {
            for(y = -10; y < 14; y++) {
                //this.addHold(x + 0.5, y + 0.5, this.playerDef.handRadius, CATEGORY_HANDHOLD, 0);
                this._addHandhold(new handhold(this.world, x + 0.5, y + 0.5, this.playerDef.handRadius, CATEGORY_HANDHOLD, 0));
            }
        }
    };


    GirpGame.prototype.addHold = function (posX, posY) {
        var hold = new handhold(this.world, posX, posY, this.playerDef.handRadius, CATEGORY_HANDHOLD, 0);

        this._addHandhold(hold);

        return hold;
    };

    GirpGame.prototype._addHandhold = function (hold) {
        hold.bind = undefined;

        this._handHolds.push(hold);
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

        if ((this.goalLeft && contact.m_fixtureA.m_body == this.goalLeft.body) || (this.goalRight && contact.m_fixtureA.m_body == this.goalRight.body)) {
            hold = contact.m_fixtureA;
            arm = contact.m_fixtureB.m_body;
        } else if ((this.goalLeft && contact.m_fixtureB.m_body == this.goalLeft.body) || (this.goalRight && contact.m_fixtureB.m_body == this.goalRight.body)) {
            hold = contact.m_fixtureB;
            arm = contact.m_fixtureA.m_body;
        } else {
            hold = 0;
        }

        if (hold) {
            var wm = new b2WorldManifold();
            wm.Initialize(contact.m_manifold, contact.m_fixtureA.m_body.m_xf, contact.m_fixtureA.m_shape.m_radius, contact.m_fixtureB.m_body.m_xf, contact.m_fixtureB.m_shape.m_radius);
            if (arm == this.player.left.armLower && this.input.leftArm) {
                this.player.left.armNode = hold;
                this.player.left.armJointPos = wm.m_points[0];
            } else if (arm == this.player.right.armLower && this.input.rightArm) {
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
        var velocityIterations = 8;
        var positionIterations = 6;

        this._tickArm(this.input.leftArm, this.player.left, this.goalLeft);
        this._tickArm(this.input.rightArm, this.player.right, this.goalRight);

        this._doHeave(this.player.left, this.player.left.armNode != undefined && this.input.heave);
        this._doHeave(this.player.right, this.player.right.armNode != undefined && this.input.heave);

        this._tickHandholds();

        this.world.Step(timeStep, velocityIterations, positionIterations);
        this.world.ClearForces();
    };

    /**
     * Make sure only hand holds near the player are assigned a letter.
     *
     * @private
     */
    GirpGame.prototype._tickHandholds = function() {
        /* calculate the distance of all the handholds from the player */
        var playerPosition = this.player.torso.GetPosition();
        var i;

        $.each(this._handHolds, function(idx, it) {
            var distance = it.body.GetPosition().Copy();
            distance.Subtract(playerPosition);
            it.distance = distance.Length();
        });

        /* put them in nearest order */
        this._handHolds.sort(function(a, b) { return a.distance - b.distance; });

        /* clear out the ones that are too far away, to free up keys */
        for(i = 0; i < this._handHolds.length; i++) {
            if (this._handHolds[i].distance > this._handHoldBindDistance && this._handHolds[i].bind != undefined) {
                this._keys[this._handHolds[i].bind] = undefined;
                this._handHolds[i].bind = undefined;
            }
        }

        /* and bind ones that are near enough but not bound yet. */
        for(i = 0; i < this._handHolds.length; i++) {
            if (this._handHolds[i].distance >= this._handHoldBindDistance) {
                // early exit! woo!
                break;
            } else if (this._handHolds[i].bind == undefined) {
                var hold = this._handHolds[i];
                var position = hold.body.GetPosition();
                var preferredKey = Math.floor(10 * position.x + 10 * position.y) % this._keysRange;

                for(var j = 0; j < this._keysRange; j++) {
                    var thisKey = this._keysStart + ((j + preferredKey) % this._keysRange);
                    if (this._keys[thisKey] == undefined) {
                        /* hooray a key. bind it. */
                        this._keys[thisKey] = hold;
                        hold.bind = thisKey;
                        break;
                    }
                }
            }
        }


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
            rjd.Initialize(side.armNode.m_body, side.armLower, side.armJointPos);

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
        var pos = side.armLower.m_xf.position.Copy();

        pos.Add(b2Math.MulMV(side.armLower.m_xf.R, side.handShape.GetLocalPosition()));

        forceDir = b2Math.SubtractVV(goal.body.m_xf.position, pos);
        forceDir.Normalize();
        forceDir.Multiply(this.playerDef.armReachForce);

        side.armLower.ApplyForce(forceDir, pos);
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
    * Create a torso
    * @private
    */
    GirpGame.prototype._initTorso = function () {
        var body;
        var fixture;

        body = new b2BodyDef();
        body.type = b2Body.b2_dynamicBody;
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.torsoSizeWidth / 2, this.playerDef.torsoSizeHeight / 2);
        fixture.density = this.playerDef.torsoDensity;
        fixture.filter.maskBits = CATEGORY_WORLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        body.angularDamping = this.playerDef.torsoAngularDamping;
        body.position.Set(this.playerDef.torsoCenterX, this.playerDef.torsoCenterY);
        this.player.torso = this.world.CreateBody(body);
        this.player.torso.CreateFixture(fixture);
    };


    /**
     * Create an arm and attach it to the torso.
     * @param dest an object that will hold the armUpper and armLower bodies.
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
            this.playerDef.torsoCenterX + dir * this.playerDef.armUpperPosX,
            this.playerDef.torsoCenterY - this.playerDef.armUpperPosY
        );
        body.angularDamping = this.playerDef.armAngularDamping;
        dest.armUpper = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.armUpperLength / 2, this.playerDef.armUpperWidth / 2);
        fixture.density = this.playerDef.armUpperDensity;
        fixture.filter.maskBits = CATEGORY_WORLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.armUpper.CreateFixture(fixture);

        /* connect it to the body - SHOULDER joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            this.playerDef.torsoCenterX + dir * this.playerDef.armUpperPosX - dir * this.playerDef.armUpperLength / 2 * 0.9,
            this.playerDef.torsoCenterY - this.playerDef.armUpperPosY
        );
        rjd.Initialize(this.player.torso, dest.armUpper, anchor);
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
            prevBody.position.x + dir * this.playerDef.armUpperLength / 2 + dir * this.playerDef.armLowerLength / 2,
            prevBody.position.y
        );
        dest.armLower = this.world.CreateBody(body);

        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.armLowerLength / 2, this.playerDef.armLowerWidth / 2);
        fixture.density = this.playerDef.armLowerDensity;
        fixture.filter.maskBits = CATEGORY_WORLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        //body.angularDamping = this.playerDef.armAngularDamping;
        dest.armLower.CreateFixture(fixture);

        /* and connect it to the upper arm. - ELBOW joint */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x - dir * this.playerDef.armLowerLength / 2,
            body.position.y
        );
        rjd.Initialize(dest.armUpper, dest.armLower, anchor);
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
        fixture.shape.SetRadius(this.playerDef.handRadius);
        fixture.shape.density = 0;
        fixture.shape.SetLocalPosition(new b2Vec2(dir * this.playerDef.armLowerLength / 2, 0));
        fixture.filter.maskBits = CATEGORY_HANDHOLD;
        fixture.filter.categoryBits = (dir < 0) ? CATEGORY_HAND_LEFT : CATEGORY_HAND_RIGHT;

        fixture.filter.isSensor = true;
        dest.handShape = dest.armLower.CreateFixture(fixture).GetShape();
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
            this.playerDef.torsoCenterX + dir * this.playerDef.legThighPosX,
            this.playerDef.torsoCenterY + this.playerDef.legThighPosY
        );
        body.angularDamping = this.playerDef.legThighAngularDamping;
        dest.thigh = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.legThighWidth, this.playerDef.legThighLength / 2);
        fixture.density = this.playerDef.legThighDensity;
        fixture.filter.maskBits = CATEGORY_WORLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.thigh.CreateFixture(fixture);


        /* connect it to the body - HIP JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.playerDef.legThighLength / 2
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
            this.playerDef.torsoCenterX + dir * this.playerDef.legThighPosX,
            this.playerDef.torsoCenterY + this.playerDef.legThighPosY + this.playerDef.legThighLength / 2 + this.playerDef.legCalfLength / 2
        );
        body.angularDamping = this.playerDef.legCalfAngularDamping;
        dest.calf = this.world.CreateBody(body);
        fixture = new b2FixtureDef();
        fixture.shape = new b2PolygonShape();
        fixture.shape.SetAsBox(this.playerDef.legCalfWidth, this.playerDef.legCalfLength / 2);
        fixture.density = this.playerDef.legCalfDensity;
        fixture.filter.maskBits = CATEGORY_WORLD;
        fixture.filter.categoryBits = CATEGORY_PLAYER;
        dest.calf.CreateFixture(fixture);

        /* and connect it to the thigh. - KNEE JOINT */
        rjd = new b2RevoluteJointDef();
        anchor = new b2Vec2(
            body.position.x,
            body.position.y - this.playerDef.legCalfLength / 2
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
    var handhold = function (world, x, y, radius, category_bits, mask_bits) {
        var body;
        var fixture;

        body = new b2BodyDef();
        body.type = b2Body.b2_staticBody;
        fixture = new b2FixtureDef();
        fixture.shape = new b2CircleShape();
        fixture.shape.SetRadius(radius);
        fixture.filter.categoryBits = category_bits;
        fixture.filter.maskBits = mask_bits;
        fixture.filter.isSensor = true;
        body.position.Set(x, y);
        this.body = world.CreateBody(body);
        this.body.CreateFixture(fixture);
    };

    window.GirpPlayerDef = function() {
        this.torsoCenterX = 2;
        this.torsoCenterY = 1;
        this.torsoSizeWidth = 0.65;
        this.torsoSizeHeight = 1.15;
        this.torsoAngularDamping = 3;
        this.torsoDensity = 1;
        this.armUpperLength = 0.65;
        this.armUpperWidth = 0.14;
        this.armUpperDensity = 1;
        this.armUpperPosX = 0.665;
        this.armUpperPosY = 0.48;
        this.armLowerLength = 0.6;
        this.armLowerWidth = 0.08;
        this.armLowerDensity = 1;
        this.armAngularDamping = 50;
        this.elbowMaxTorque = 12;
        this.elbowMotorSpeed = 20;
        this.armReachForce = 1.4;
        this.legThighLength = 0.8;
        this.legThighWidth = 0.13;
        this.legThighDensity = 2;
        this.legThighAngularDamping = 2;
        this.legThighPosX = 0.17;
        this.legThighPosY = 0.98;
        this.hipMinAngle = 0.2;
        this.hipMaxAngle = -1.7;
        this.legCalfLength = 0.8;
        this.legCalfWidth = 0.08;
        this.legCalfDensity = 2;
        this.legCalfAngularDamping = 1;
        this.handRadius = 0.05;
    };

})(jQuery);


