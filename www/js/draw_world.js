(function($) {
    var context;

    var b2Math = Box2D.Common.Math.b2Math;
    var b2Vec2 = Box2D.Common.Math.b2Vec2;
    var b2Shape = Box2D.Collision.Shapes.b2Shape;
    var b2Joint = Box2D.Dynamics.Joints.b2Joint;
    var b2World = Box2D.Dynamics.b2World;

    window.d_velocityVector = 0;

    var vectors = [];

    window.setRenderContext = function(ctx) {
        context = ctx;
        ctx.scale(100, 100);
        ctx.lineWidth = 0.01;

        ctx.translate(1.50, 1.50);
    };

    window.drawWorld = function(world) {
        var tV;

        for (var j = world.m_jointList; j; j = j.m_next) {
            drawJoint(j);
        }
        for (var body = world.m_bodyList; body; body = body.m_next) {
            for (var f = body.GetFixtureList(); f; f = f.m_next) {
                var s = f.GetShape();
                drawShape(body, s);
            }

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

        $.each(vectors, function(idx, item) {
            context.strokeStyle = item.col;
            context.beginPath();
            context.moveTo(item.x1, item.y1);
            context.lineTo(item.x2, item.y2);
            context.stroke();
        });

        vectors = [];
    };

    function DrawSegment(p1, p2, color) {
        window.drawVector(p1.x, p1.y, p2.x, p2.y, color);
    }

    function drawJoint(joint) {
        var b1 = joint.GetBodyA();
        var b2 = joint.GetBodyB();
        var xf1 = b1.m_xf;
        var xf2 = b2.m_xf;
        var x1 = xf1.position;
        var x2 = xf2.position;
        var p1 = joint.GetAnchorA();
        var p2 = joint.GetAnchorB();

        var color = "#00eeee";
        switch (joint.m_type) {
        case b2Joint.e_distanceJoint:
            DrawSegment(p1, p2, color);
            break;
        case b2Joint.e_pulleyJoint:
            var pulley = ((joint instanceof b2PulleyJoint ? joint : null));
            var s1 = pulley.GetGroundAnchorA();
            var s2 = pulley.GetGroundAnchorB();
            DrawSegment(s1, p1, color);
            DrawSegment(s2, p2, color);
            DrawSegment(s1, s2, color);
            break;
        case b2Joint.e_mouseJoint:
            DrawSegment(p1, p2, color);
            break;
        case b2Joint.e_weldJoint:
            DrawSegment(x1, p1, "#004411");
            DrawSegment(p1, p2, "#004411");
            DrawSegment(x2, p2, "#004411");
            break;
        default:
            DrawSegment(x1, p1, color);
            DrawSegment(p1, p2, color);
            DrawSegment(x2, p2, color);
        }
    }
    function drawShape(body, shape) {
        var tV;
        var i, d, v;

        context.strokeStyle = '#ffffff';
        if (body.m_color) context.strokeStyle = body.m_color;
        context.beginPath();
        switch (shape.m_type) {
        case b2Shape.e_circleShape:
            var pos = body.m_xf.position.Copy();
            var r = shape.m_radius;
            var segments = 16.0;
            var theta = 0.0;
            var dtheta = 2.0 * Math.PI / segments;
            // draw circle
            pos.Add(b2Math.MulMV(body.m_xf.R, shape.GetLocalPosition()));

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
            var ax = body.m_xf.R.col1;
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

    }

    window.drawVector = function(x1, y1, x2, y2, col) {
        var undef;
        if (undef == col) col = "#4400ff";
        vectors.push({x1: x1, x2: x2, y1: y1, y2: y2, col: col});
    }
})(jQuery);