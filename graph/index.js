'strict on';

var GraphComponent = Vue.extend({
    template: '<canvas id="frenetGraph" class="drawingBoard" width="1200px" height="450px"></canvas>',

    created: function() {
    },

    mounted: function() {
        this.graph = new Graph(this.$el);
        this.graph.draw();
    },

    setRanges: function(worldX, worldY) {
        this.graph.setRange(worldX, worldY);
        this.graph.clear();
        this.graph.draw();
    }
});

Vue.component('graph', GraphComponent);

var app = new Vue({
    el: '#graph-app',
    data:{
        messages: [],
        selectedMessage: null,
        serverIp: "127.0.0.1"
    },
    computed: {

    },
    watch: {
        selectedMessage : function(val) {
            // console.log('message has been selected');

            var fg = this.getGraph();
            var cp = new CarPlotter(this.selectedMessage);

            fg.restoreImageData();
            cp.plot(fg);
        }
    },
    methods: {
        connect: function() {
            var socket = new WebSocket('ws://127.0.0.1:4568');

            socket.onopen = function(event) {
                this.handler = new MessageHandler(app, socket);
            };

            socket.onerror = function(error) {
                alert('Could not connect to server');
            }
        },

        drawSpline: function() {
            var fg = this.getGraph();
            fg.restoreImageData();
            fg.drawPoints(this.selectedMessage.splinePts, 'green', 2);
        },

        drawSplinePts: function() {
            var fg = this.getGraph();
            fg.drawPoints(this.selectedMessage.splinePts, 'green', 2);
        },

        getGraph: function() {
            return this.$refs.frenetGraph.graph;
        },

        messageSelected: function() {
        }
    },

    created: function() {
        console.log('created:', this);
    }
});

var MessageHandler = function(app, socket){
    console.log(app);
    console.log('Connected to: ' + event.currentTarget.url);
    
    socket.onerror = function(error) {
        console.log(error);
    }

    socket.onmessage = function(event) {
        var message = JSON.parse(event.data);
        console.log(message.type);
        
        if (message.type == "worldmap") {
            this.worldmapArrived(message);
        }
        else if (message.type == "debug") {
            var model = message.model;
            var id = model.acqTime;
            var text = 'car_s: ' + message.model.s + ' d:' + message.model.d;

            message['id'] = id;
            message['text'] = text;

            app.messages.push(message);
            app.selectedMessage = message;
        }
    }.bind(this)

    this.worldmapArrived = function(message) {
        worldmap_x = message.worldmap_x;
        worldmap_y = message.worldmap_y;
    
        var xRange = [999999, -999999]
        var yRange = [999999, -999999]

        console.log(this);

        var fg = app.getGraph();
    
        for (var i = 0; i < worldmap_x.length; i++) {
            fg.add(worldmap_x[i], worldmap_y[i]);
    
            if (worldmap_x[i] < xRange[0])
                xRange[0] = worldmap_x[i];
            else if (worldmap_x[i] > xRange[1])
                xRange[1] = worldmap_x[i];
    
            if (worldmap_y[i] < yRange[0])
                yRange[0] = worldmap_y[i];
            else if (worldmap_y[i] > yRange[1])
                yRange[1] = worldmap_y[i];
        }
    
        // lets set the X and Y ranges as per the data we have received, keeping about 2% on both
        var extra = (xRange[1] - xRange[0]) * 0.02;
        xRange[0] -= extra;
        xRange[1] += extra;
    
        extra = (yRange[1] - yRange[0]) * 0.02;
        yRange[0] -= extra;
        yRange[1] += extra;
    
        fg.setRange(xRange, yRange);
        fg.clear();
        fg.draw();

        fg.saveImageData();
    }
};

// function itemSelected() {
//     var index = $(this).val();
//     if (index >= 0) {
//         var item = window.dv[index];
//         window.item = item;

//         // tell car plotter to plot the information
//         window.cp.addItem(item);
//         window.cp.plot(window.fg);

//         // show information in the select box
//         var model = item.model;
//         $("#car_d").html(model.d);
//         $("#car_s").html(model.s);
//         $("#car_xy").html(`${model.x}, ${model.y}`);
//         $("#car_yaw").html(model.yaw);
//         $("#speed_mph").html(model.speedMph + ' mph');
//         $("#ref_yaw").html(item.refYaw.angleRad + ' rad (' + item.refYaw.basedOn + ')');

//         $(".itemButton").show();
//         showSplineBasisPts();
//     }
// }

// function showSplineBasisPts() {
//     var splinePts = window.item.splinePts;
//     var html = splinePts.map((pt) => {
//         var x = parseInt((pt[0] * 100)) / 100;
//         var y = parseInt((pt[1] * 100)) / 100;

//         return `<tr><td>${x}</td><td>${y}</td></tr>`;
//     });
    
//     console.log(html);
//     $('#splineBasisPts > tbody:last-child').append(html);
    

//     // for (var i = 0; i < splinePts.length; i++) {
//     //     var pt = splinePts[i];
//     //     var row = `<tr><td>${pt[0]}</td><td>${pt[1]}</td></tr>`;
//     //     $('#splineBasisPts > tbody:last-child').append('<tr>...</tr><tr>...</tr>');
//     // }
// }

// function drawSpline() {
//     console.log(window.item.model.d);
// }
