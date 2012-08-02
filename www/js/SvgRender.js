(function($) {

    var b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Math = Box2D.Common.Math.b2Math,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        undefined;

    var SvgRenderer = function(renderSvgEl, playerSvg) {
        this.cam = new CamPanner(
            new b2Vec2(-1.5, -1.5),
            new b2Vec2(5, 5),
            new b2Vec2(2.5, 2.5)
        );

        this.playerSvg = playerSvg;

        this.renderSvg = renderSvgEl;

        this._setupPlayerElements();
    };

    SvgRenderer.prototype._setupPlayerElements = function() {

        this.playerElements = {};

        this.playerElements.torso = $(this.playerSvg).find('[inkscape\\:label*="#torso"]')[0];
        this.playerElements.torso.setAttribute("x", 0);
        this.playerElements.torso.setAttribute("y", 0);

        this._setupSideElements("right");
        this._setupSideElements("left");

        var render = this;
        $.each(this.playerElements, function(idx, item) {
            render.renderSvg.appendChild(item);
        });
    };
    
    SvgRenderer.prototype._setupSideElements = function(side) {
        this.playerElements[side + "arm_upper"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'arm_upper"]')[0];
        this.playerElements[side + "arm_upper"].setAttribute("x", 0);
        this.playerElements[side + "arm_upper"].setAttribute("y", 0);

        this.playerElements[side + "arm_lower"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'arm_lower"]')[0];
        this.playerElements[side + "arm_lower"].setAttribute("x", 0);
        this.playerElements[side + "arm_lower"].setAttribute("y", 0);

        this.playerElements[side + "leg_thigh"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'leg_thigh"]')[0];
        this.playerElements[side + "leg_thigh"].setAttribute("x", 0);
        this.playerElements[side + "leg_thigh"].setAttribute("y", 0);

        this.playerElements[side + "leg_calf"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'leg_calf"]')[0];
        this.playerElements[side + "leg_calf"].setAttribute("x", 0);
        this.playerElements[side + "leg_calf"].setAttribute("y", 0);
    };

    SvgRenderer.prototype.setGame = function(girpGame) {
        this.girpGame = girpGame;

        this.svgPlayerBinds = { };

        this._bindPlayerElements("right", girpGame.player.right);
        this._bindPlayerElements("left", girpGame.player.left);

        this.svgPlayerBinds["torso"] = girpGame.player.torso;
    };

    SvgRenderer.prototype._bindPlayerElements = function(side, playerPxBodies) {
        this.svgPlayerBinds[side + "arm_upper"] = playerPxBodies.armUpper;
        this.svgPlayerBinds[side + "arm_lower"] = playerPxBodies.armLower;
        this.svgPlayerBinds[side + "leg_thigh"] = playerPxBodies.thigh;
        this.svgPlayerBinds[side + "leg_calf"] = playerPxBodies.calf;
    };

    SvgRenderer.prototype.render = function() {
        var viewBox;
        var girpGame = this.girpGame;

        this.cam.update(girpGame.player.torso.m_xf.position);

        viewBox = [ 10 * this.cam.translate.x, 10 * this.cam.translate.y, 50, 50].join(",");

        this.renderSvg.setAttribute("viewBox", viewBox)

        function update(body, svgEl) {
            var trans;
            trans = "translate(" +
                (body.m_xf.position.x * 10) + " " +
                (body.m_xf.position.y * 10) +
            ")";

            trans += " rotate(" + (180.0 / Math.PI * body.m_xf.GetAngle()) + ")";

            svgEl.setAttribute("transform", trans);
        }

        for(var side in {"right":1,"left":1}) {
            update(this.svgPlayerBinds[side + "arm_upper"], this.playerElements[side + "arm_upper"]);
            update(this.svgPlayerBinds[side + "arm_lower"], this.playerElements[side + "arm_lower"]);
            update(this.svgPlayerBinds[side + "leg_thigh"], this.playerElements[side + "leg_thigh"]);
            update(this.svgPlayerBinds[side + "leg_calf"], this.playerElements[side + "leg_calf"]);
        }

        update(this.svgPlayerBinds["torso"], this.playerElements["torso"])
    };

    window.SvgRenderer = SvgRenderer;

})(jQuery);