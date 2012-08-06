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
        var offs;

        this.playerElements = {};

        this.playerElements.torso = $(this.playerSvg).find('[inkscape\\:label*="#torso"]')[0];
        offs = new b2Vec2(
            parseFloat(this.playerElements["torso"].getAttribute("width")) / -2 - parseFloat(this.playerElements["torso"].getAttribute("x")),
            parseFloat(this.playerElements["torso"].getAttribute("height")) / -2 - parseFloat(this.playerElements["torso"].getAttribute("y"))
        );
        this.playerElements["torso"].baseOffs = offs;

        this._setupSideElements("right");
        this._setupSideElements("left");

        var render = this;
        $.each(this.playerElements, function(idx, item) {
            render.renderSvg.appendChild(item);
        });
    };
    
    SvgRenderer.prototype._setupSideElements = function(side) {
        var offs;

        this.playerElements[side + "arm_upper"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'arm_upper"]')[0];
        offs = new b2Vec2(
            parseFloat(this.playerElements[side + "arm_upper"].getAttribute("width")) * -0.5 - parseFloat(this.playerElements[side + "arm_upper"].getAttribute("x")),
            parseFloat(this.playerElements[side + "arm_upper"].getAttribute("height")) * -0.5 - parseFloat(this.playerElements[side + "arm_upper"].getAttribute("y"))
        );
        this.playerElements[side + "arm_upper"].baseOffs = offs;

        this.playerElements[side + "arm_lower"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'arm_lower"]')[0];
        offs = new b2Vec2(
            parseFloat(this.playerElements[side + "arm_lower"].getAttribute("width")) * -0.5 - parseFloat(this.playerElements[side + "arm_lower"].getAttribute("x")),
            parseFloat(this.playerElements[side + "arm_lower"].getAttribute("height")) * -0.5 - parseFloat(this.playerElements[side + "arm_lower"].getAttribute("y"))
        );
        this.playerElements[side + "arm_lower"].baseOffs = offs;

        this.playerElements[side + "leg_thigh"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'leg_thigh"]')[0];
        offs = new b2Vec2(
            parseFloat(this.playerElements[side + "leg_thigh"].getAttribute("width")) * -0.5 - parseFloat(this.playerElements[side + "leg_thigh"].getAttribute("x")),
            parseFloat(this.playerElements[side + "leg_thigh"].getAttribute("height")) * -0.5 - parseFloat(this.playerElements[side + "leg_thigh"].getAttribute("y"))
        );
        this.playerElements[side + "leg_thigh"].baseOffs = offs;


        this.playerElements[side + "leg_calf"] = $(this.playerSvg).find('[inkscape\\:label*="#' + side + 'leg_calf"]')[0];
        offs = new b2Vec2(
            parseFloat(this.playerElements[side + "leg_calf"].getAttribute("width")) * -0.5 - parseFloat(this.playerElements[side + "leg_calf"].getAttribute("x")),
            parseFloat(this.playerElements[side + "leg_calf"].getAttribute("height")) * -0.5 - parseFloat(this.playerElements[side + "leg_calf"].getAttribute("y"))
        );
        this.playerElements[side + "leg_calf"].baseOffs = offs;
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
            var trans = "";
            trans += "translate(" +
                (body.m_xf.position.x * 10) + " " +
                (body.m_xf.position.y * 10) +
            ")";

            trans += " rotate(" + (180.0 / Math.PI * body.m_xf.GetAngle()) + ")";

            if (svgEl.baseOffs != undefined) {
                trans += " translate(";
                trans += svgEl.baseOffs.x + " ";
                trans += svgEl.baseOffs.y + ")";
            }


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