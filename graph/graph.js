// function testGraph() {
//     var g = new Graph("frenet");

//     g.add(10, 10);
//     g.add(20, 20);
//     g.add(30, 25);
//     g.add(40, 15);
//     g.add(50, 0);
//     g.add(60, 70);

//     g.draw();
// }

var Graph = function(canvas) {
    //this.canvas = document.getElementById(id);
    this.canvas = canvas;
    this.ctx = this.canvas.getContext("2d");

    this.xRange = [0, 100];
    this.yRange = [-100, 100];
    this.data = []
    this.boundary = [40,20,20,20];
    this.majorTicksX = 8;
    this.majorTicksY = 4;
    this.majorTickSize = 10;
    this.drawArea = undefined;
    this.plotters = [];
    this.scaled = false;
    this.imageData = undefined;
    
    this.addPlotter = function(plotter) {
        this.plotters.push(plotter);
    }

    this.setRange = function(xRange, yRange) {
        this.xRange = xRange;
        this.yRange = yRange;
    }

    this.computeBoundary = function() {
        var height = this.canvas.height;
        var width = this.canvas.width;

        var drawRect = [this.boundary[0], this.boundary[1], 
            width - this.boundary[0] - this.boundary[2], 
            height - this.boundary[1] - this.boundary[3]];

        this.drawArea = {
            left: drawRect[0],
            top: drawRect[1],
            width: drawRect[2],
            height: drawRect[3]
        }

        if (!this.scaled) {
            this.scaled = true;
            // scaling from article: https://www.html5rocks.com/en/tutorials/canvas/hidpi/
            // basic idea, find out ratio of device pixels to backstore canvas image
            // increase height / width by that many pixeld but tell CSS
            // to show the original size. Set scale to the ratio (e.g twice) so that all 
            // drawing code automatically gets multiplied by twice

            devicePixelRatio = window.devicePixelRatio || 1,
            backingStoreRatio = this.ctx.webkitBackingStorePixelRatio ||
                                this.ctx.mozBackingStorePixelRatio ||
                                this.ctx.msBackingStorePixelRatio ||
                                this.ctx.oBackingStorePixelRatio ||
                                this.ctx.backingStorePixelRatio || 1,

            ratio = devicePixelRatio / backingStoreRatio;

            // upscale the canvas if the two ratios don't match
            if (devicePixelRatio !== backingStoreRatio) {
                var oldWidth = this.canvas.width;
                var oldHeight = this.canvas.height;

                this.canvas.width = oldWidth * ratio;
                this.canvas.height = oldHeight * ratio;

                this.canvas.style.width = oldWidth + 'px';
                this.canvas.style.height = oldHeight + 'px';

                // now scale the this.ctx to counter
                // the fact that we've manually scaled
                // our this.canvas element
                this.ctx.scale(ratio, ratio);
            }
        }
    }

    this.clear = function() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        // this.canvas.width = this.canvas.width;
    }

    this.drawBoundary = function() {
        var ctx = this.ctx;
        ctx.beginPath();
        ctx.strokeStyle="#000000";
        
        ctx.rect(this.drawArea.left, this.drawArea.top, this.drawArea.width, this.drawArea.height);

        ctx.font = "10px Arial";

        var sizeEach = 1.0 / (this.majorTicksX + 1.0);

        // horizontal axis
        ctx.textAlign = "center";
        var distance = this.drawArea.width * sizeEach;
        
        for (var i = 1; i <= this.majorTicksX; i++ ){
            var x = this.drawArea.left + i * distance;
            var y = this.drawArea.top + this.drawArea.height;
            ctx.moveTo(x,y);

            var label = parseInt(this.xRange[1] * i * sizeEach);
            ctx.fillText(label, x, y + 10);

            y -= this.majorTickSize;
            ctx.lineTo(x,y);
        }

        sizeEach = 1.0 / (this.majorTicksY + 1.0);
        distance = (this.yRange[1] - this.yRange[0]) * sizeEach;
        var start = this.yRange[0];

        ctx.textAlign = "right";

        for (var i = 0; i <= this.majorTicksY + 1; i++ ){
            var x = this.drawArea.left;
            var y = this.getY(start);

            ctx.moveTo(x,y);

            var label = parseInt(start);
            ctx.fillText(label, x - 5, y + 2.5);

            x += this.majorTickSize;
            ctx.lineTo(x,y);

            start += distance;
        }

        ctx.stroke();
    }

    this.draw = function() {
        if (this.drawArea === undefined)
            this.computeBoundary();

        this.ctx.imageSmoothingEnabled = true;
            
        this.drawBoundary();
        this.drawPoints(this.data, "#FF0000");
    }

    this.saveImageData = function() {
        this.imageData = this.ctx.getImageData(0, 0, this.canvas.width, this.canvas.height);
    }

    this.restoreImageData = function() {
        this.ctx.putImageData(this.imageData, 0, 0);
    }

    this.drawPoints = function(data, color, lineWidth=1) {
        if (data.length > 0) {
            var ctx = this.ctx;
            ctx.beginPath();
            
            var pt = data[0];
            var x1 = this.getX(pt[0]);
            var y1 = this.getY(pt[1]);
            ctx.moveTo(x1,y1);

            for (var i = 1; i < data.length; i++){
                pt = data[i];

                var x2 = this.getX(pt[0]);
                var y2 = this.getY(pt[1]);

                ctx.lineTo(x2,y2);
            }

            ctx.strokeStyle = color;
            ctx.lineWidth = lineWidth;
            ctx.stroke();

            for (var i = 0; i < data.length; i++){
                pt = data[i];
                this.drawCircle(pt[0], pt[1], 2, 'green');
            }                
        }
    }

    this.add = function(worldX, worldY) {
        this.data.push([worldX, worldY]);
    }

    this.getY = function(worldY) {
        //var diff = this.yRange[1] - worldY;
        var diff = worldY - this.yRange[0];
        var worldHeight = this.yRange[1] - this.yRange[0];
        var percent = diff / worldHeight;

        // for negative y ranges starting from bottom of screen to top, the percent has to be reversed
        percent = 1 - percent;
        // var y = this.drawArea.height - this.drawArea.height * percent;
        var y = this.drawArea.top + this.drawArea.height * percent;
        return y;
    }

    this.getX = function(worldX) {
        var diff = worldX - this.xRange[0];
        var worldWidth = this.xRange[1] - this.xRange[0];
        var percent = diff / worldWidth;

        // var x = this.canvas.width * percent;
        var x = this.drawArea.left + this.drawArea.width * percent;
        return x;
    }

    this.drawCircle = function(worldX, worldY, radius, color) {
        var ctx = this.ctx;

        var x = this.getX(worldX);
        var y = this.getY(worldY);

        ctx.beginPath();
        ctx.arc(x, y, radius, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.fill();
        ctx.stroke();
    }
}

var CarPlotter = function(message) {

    const model = message.model;

    this.plot = function(graph) {
        // this.graph.restoreImageData();

        this.graph = graph;
        this.ctx = graph.ctx;

        // draw a circle where the car is (car_x, car_y)
        this.ctx.beginPath();
        this.graph.drawCircle(model.x, model.y, 3, 'blue');
        this.drawArrow(model.x, model.y, 360.0 - model.yaw);
        
        this.ctx.strokeStyle = 'black';
        this.ctx.lineWidth = 4;
        this.ctx.stroke();

        // draw all other cars
        const colors = ['red', 'orange', 'pink'];

        this.ctx.beginPath();
        for (let otherCar of message.sensorFusion) {
            if (otherCar.d >= 0) {
                const laneColor = parseInt(otherCar.d / 4);
                var x = otherCar.x;
                var y = otherCar.y;

                this.drawCar(x, y, colors[laneColor]);
            }
        }

        this.ctx.stroke();
    }

    this.drawCar = function(x, y, color) {
        this.graph.drawCircle(x, y, 4, color);
    }

    this.drawArrow = function(x, y, angleDeg) {
        this.ctx.beginPath();

        // draw a line indicating the yaw
        var angle = angleDeg * Math.PI / 180.0;
        var x = this.graph.getX(x);
        var y = this.graph.getY(y);
        var x2 = x + 10 * Math.cos(angle);
        var y2 = y + 10 * Math.sin(angle);

        // straight line
        this.ctx.moveTo(x, y);
        this.ctx.lineTo(x2, y2);

        // down arrow
        x = x2 - 4 * Math.cos(angle - Math.PI / 6);
        y = y2 - 4 * Math.sin(angle - Math.PI / 6);
        this.ctx.lineTo(x,y);

        // up arrow
        this.ctx.moveTo(x2,y2);
        x = x2 - 4 * Math.cos(angle + Math.PI / 6);
        y = y2 - 4 * Math.sin(angle + Math.PI / 6);
        this.ctx.lineTo(x,y);
    }

    this.drawSplineBasis = function() {
        // draw line for the spline pts
        this.graph.drawPoints(item.splinePts, "#00FF00", 3);
    }
}
