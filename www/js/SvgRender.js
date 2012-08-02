(function($) {

    var cam,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        undefined;

    window.initSvgRender = function(context) {
        cam = new CamPanner(
            new b2Vec2(-1.5, -1.5),
            new b2Vec2(5, 5),
            new b2Vec2(2.5, 2.5)
        );
    };

    window.doSvgRender = function(girpGame) {
        var viewBox;

        cam.update(girpGame.player.torso.m_xf.position);

        viewBox = [ 10 * cam.translate.x, 10 * cam.translate.y, 50, 50].join(",");

        $('#map')[0].contentDocument.getElementsByTagName("svg")[0].setAttribute("viewBox", viewBox)
    }

})(jQuery);