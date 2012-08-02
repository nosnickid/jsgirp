
/**
 * Oh how I love thee, the IIFE
 *
 * This is a simple class that just pans a translation whenever the point
 * used to update it drifts into some scroll borders defined at the edges
 * of the rendering area.
 *
 */
(function($) {
    var b2Vec2 = Box2D.Common.Math.b2Vec2

    /**
     *
     * @param renderAreaDimensions b2Vec2
     * @param desiredScrollBorder b2Vec2
     * @constructor
     */
    window.CamPanner = function(initialTranslate, renderAreaDimensions, desiredScrollBorder) {
        this.translate = initialTranslate;
        this.renderAreaDimensions = renderAreaDimensions;
        this.scrollBorder = desiredScrollBorder;
        this.scrollArea = new b2Vec2(
            (renderAreaDimensions.x - 2 * desiredScrollBorder.x) / 2,
            (renderAreaDimensions.y - 2 * desiredScrollBorder.y) / 2
        );
    };

    window.CamPanner.prototype.update = function(playerPos) {

        var center = {
            x: this.translate.x + this.renderAreaDimensions.x * 0.5,
            y: this.translate.y + this.renderAreaDimensions.y * 0.5
        };

        if (playerPos.x < (center.x - this.scrollArea.x)) {
            this.translate.x = playerPos.x - this.scrollBorder.x;
        } else if (playerPos.x > (center.x + this.scrollArea.x)) {
            this.translate.x = playerPos.x - this.scrollArea.x * 2 - this.scrollBorder.x;
        }
        if (playerPos.y < (center.y - this.scrollArea.y)) {
            this.translate.y = playerPos.y - this.scrollBorder.y;
        } else if (playerPos.y > (center.y + this.scrollArea.y)) {
            this.translate.y = playerPos.y - this.scrollArea.y * 2 - this.scrollBorder.y;
        }

    }

})(jQuery);