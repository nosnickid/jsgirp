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
    var rjd;

    var PROPORTIONS = {
        body_height: 1.0/3,
        body_width:  2.0/9,

        hand_size: 1.0 / 20,

        height: 18.0
    };

    /* Create a floor */
    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(1000, 5);
    shape.restitution = 0.2;
    body.AddShape(shape);
    body.position.Set(0, 600);
    world.CreateBody(body);

    var player = {};

    body = new b2BodyDef();
    shape = new b2BoxDef();
    /*shape.extents.Set(
        PROPORTIONS.height * PROPORTIONS.body_width,
        PROPORTIONS.height * PROPORTIONS.body_height
    );*/
    shape.extents.Set(5,5);
    shape.density = 1;
    body.AddShape(shape);
    body.position.Set(50, 50);
    player.body = world.CreateBody(body);

    player.hand = {};

    body = new b2BodyDef();
    shape = new b2BoxDef();
    shape.extents.Set(
        PROPORTIONS.height * PROPORTIONS.hand_size,
        PROPORTIONS.height * PROPORTIONS.hand_size
    );
    body.AddShape(shape);
    player.hand.left = world.CreateBody(body);

    // left: new b2BodyDef(), right: new b2BodyDef() };

    this.player = player;



};