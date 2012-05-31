(function($) {
    var context;

    var b2Shape = Box2D.Collision.Shapes.b2Shape;
    var b2Math = Box2D.Common.Math.b2Math;

    window.d_velocityVector = 0;

    var vectors = [];

    window.setRenderContext = function(ctx) {
        context = ctx;
        ctx.translate(150, 0);
    };

    window.drawWorld = function(world) {
        for (var j = world.m_jointList; j; j = j.m_next) {
            //drawJoint(j, context);
        }
        for (var b = world.m_bodyList; b; b = b.m_next) {
            for (var f = b.GetFixtureList(); f; f = f.m_next) {
                s = f.GetShape();
                drawShape(b, s);
            }
        }

        $.each(function(item) {
            context.strokeStyle = item.col;
            context.beginPath();
            context.moveTo(item.x1, item.y1);
            context.lineTo(item.x2, item.y2);
            context.stroke();
        });

        vectors = [];
    };

    function drawJoint(joint) {
        var b1 = joint.m_body1;
        var b2 = joint.m_body2;
        var x1 = b1.m_position;
        var x2 = b2.m_position;
        var p1 = joint.GetAnchor1();
        var p2 = joint.GetAnchor2();
        context.strokeStyle = '#00eeee';
        context.beginPath();
        switch (joint.m_type) {
        case b2Joint.e_distanceJoint:
            context.moveTo(p1.x, p1.y);
            context.lineTo(p2.x, p2.y);
            break;

        case b2Joint.e_pulleyJoint:
            // TODO
            break;

        default:
            if (b1 == world.m_groundBody) {
                context.moveTo(p1.x, p1.y);
                context.lineTo(x2.x, x2.y);
            }
            else if (b2 == world.m_groundBody) {
                context.moveTo(p1.x, p1.y);
                context.lineTo(x1.x, x1.y);
            }
            else {
                context.moveTo(x1.x, x1.y);
                context.lineTo(p1.x, p1.y);
                context.lineTo(x2.x, x2.y);
                context.lineTo(p2.x, p2.y);
            }
            break;
        }
        context.stroke();
    }
    function drawShape(body, shape) {
        var tV;
        var i, d, v;

        context.strokeStyle = '#ffffff';
        context.beginPath();
        switch (shape.m_type) {
        case b2Shape.e_circleShape:
            var circle = shape;
            var pos = circle.m_position;
            var r = circle.m_radius;
            var segments = 16.0;
            var theta = 0.0;
            var dtheta = 2.0 * Math.PI / segments;
            // draw circle
            context.moveTo(pos.x + r, pos.y);
            for (i = 0; i < segments; i++) {
                d = new b2Vec2(r * Math.cos(theta), r * Math.sin(theta));
                v = b2Math.AddVV(pos, d);
                context.lineTo(v.x, v.y);
                theta += dtheta;
            }
            context.lineTo(pos.x + r, pos.y);

            // draw radius
            context.moveTo(pos.x, pos.y);
            var ax = circle.m_R.col1;
            var pos2 = new b2Vec2(pos.x + r * ax.x, pos.y + r * ax.y);
            context.lineTo(pos2.x, pos2.y);
            break;
        case b2Shape.e_polygonShape:
            var localVertices = shape.GetVertices();
            tV = b2Math.MulX(body.m_xf, localVertices[0]);
            context.moveTo(tV.x, tV.y);
            for (i = 0; i < localVertices.length; i++) {
                v = b2Math.MulX(body.m_xf, localVertices[i]);
                context.lineTo(v.x, v.y);
            }
            context.lineTo(tV.x, tV.y);
            break;
        }
        context.stroke();

        /* draw force and velocity vectors */
        if (d_velocityVector) {
            tV = b2Math.AddVV(body.m_xf.position, b2Math.MulFV(0.5, body.m_linearVelocity));
            context.strokeStyle = "#dd2222";
            context.beginPath();
            context.moveTo(body.m_xf.position.x, body.m_xf.position.y);
            context.lineTo(tV.x, tV.y);
            context.stroke();

            tV = b2Math.AddVV(body.m_xf.position, b2Math.MulFV(0.005, body.m_force));
            context.strokeStyle = "#22dd22";
            context.beginPath();
            context.moveTo(body.m_xf.position.x, body.m_xf.position.y);
            context.lineTo(tV.x, tV.y);
            context.stroke();
        }


    }

    window.drawVector = function(x1, y1, x2, y2, col) {
        if (!col) col = "#4400ff";
        vectors.push({x1: x1, x2: x2, y1: y1, y2: y2, col: col});
    }
})(jQuery);