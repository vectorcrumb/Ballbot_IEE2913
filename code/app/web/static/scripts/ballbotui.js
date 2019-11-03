$(document).ready(function() {

    // Obtain canvas object for battery voltage display
    var canvasBattery = document.getElementById("battery-animation");
    var canvas2DBattery = canvasBattery.getContext("2d");
    // Connect to websockets server with namespace /webui
    namespace = '/ballbot';
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + namespace, { 'sync disconnect on unload': true });

    // Plotly JS plots
    var arrayLength = 30;
    var plot_config = {
        title: "Plot",
        autosize: false,
        width: 500,
        height: 400,
        margin:{
            l: 47,
            r: 34,
            b: 30,
            t: 40,
            pad: 4
        }
    };

    var plot_config_imu = {...plot_config};
    var plot_config_vtorq = {...plot_config};
    var plot_config_rtorq = {...plot_config};
    plot_config_imu.title = "Roll (X), Pitch (Y) & Yaw (Z) angles"; 
    plot_config_vtorq.title = "Virtual motor torques";
    plot_config_rtorq.title = "Real motor torques";

    var stateTraces = {
        'imu_x': new Array(arrayLength).fill(0),
        'imu_y': new Array(arrayLength).fill(0),
        'imu_z': new Array(arrayLength).fill(0),
        'tx': new Array(arrayLength).fill(0),
        'ty': new Array(arrayLength).fill(0),
        'tz': new Array(arrayLength).fill(0),
        't1': new Array(arrayLength).fill(0),
        't2': new Array(arrayLength).fill(0),
        't3': new Array(arrayLength).fill(0)
    }

    Plotly.newPlot('imu_plot', [
            {y: stateTraces.imu_x, name: "Roll"},
            {y: stateTraces.imu_y, name: "Pitch"},
            {y: stateTraces.imu_z, name: "Yaw"}
        ], plot_config_imu
    );

    Plotly.newPlot('vtorq_plot', [
            {y: stateTraces.tx, name: "Motor X Torque"},
            {y: stateTraces.ty, name: "Motor Y Torque"},
            {y: stateTraces.tz, name: "Motor Z Torque"}
        ], plot_config_vtorq
    );

    Plotly.newPlot('rtorq_plot', [
            {y: stateTraces.t1, name: "Motor 1 Torque"},
            {y: stateTraces.t2, name: "Motor 2 Torque"},
            {y: stateTraces.t3, name: "Motor 3 Torque"}
        ], plot_config_rtorq
    );

    // On connect listener: Triggers upon a client connecting to the server
    socket.on('connect', function() {
        $("#connection_state").text('Connected');
        $("#connection_state").css('color', 'green');
        // Battery canvas animation: Create battery on connect, set apparent voltage to 1V
        socket.emit('broadcast', {
            'event': "receive_data_battery",
            'data': 1
        });
    })

    // On disconnect listener: Triggers upon a client disconnecting from the server
    socket.on('disconnect', function() {
        $("#connection_state").text('Disconnected');
        $("#connection_state").css('color', 'red');
    })

    // receive_data listener: Emitted async by the server to transmit information from simulations or roboRIO
    socket.on("receive_data_battery", function (payload){
        // The msg variable is the dictionary payload and contains the keys specified by the server
        var number = payload["data"];
        // Battery canvas animation: Clear canvas, draw battery shape, draw 'charge' of battery proportionally
        canvas2DBattery.clearRect(0, 0, 255, 100);
        canvas2DBattery.fillStyle = "#000000";
        canvas2DBattery.fillRect(0, 0, 240, 100);
        canvas2DBattery.fillRect(240, 28, 15, 50)
        canvas2DBattery.fillStyle = "#40FF00";
        canvas2DBattery.fillRect(10, 10, (220 / 13) * number, 80);
        // Write on canvas
        canvas2DBattery.font = "30px Arial";
        canvas2DBattery.fillStyle = "white";
        canvas2DBattery.textAlign = "center";
        canvas2DBattery.fillText(number + " V", 3 * canvasBattery.width / 8, canvasBattery.height / 2);
    });


    socket.on("update", function(payload){
        if (payload['err'] != 1) {
            var imu = payload['imu'];
            $('#imu_x').html(imu[0] + "°");
            $('#imu_y').html(imu[1] + "°");
            $('#imu_z').html(imu[2] + "°");

            var vtorques = payload['vtorq'];
            var rtorques = payload['rtorq'];

            stateTraces.imu_x = stateTraces.imu_x.concat(imu[0]);
            stateTraces.imu_x.splice(0, 1);
            stateTraces.imu_y = stateTraces.imu_y.concat(imu[1]);
            stateTraces.imu_y.splice(0, 1);
            stateTraces.imu_z = stateTraces.imu_z.concat(imu[2]);
            stateTraces.imu_z.splice(0, 1);

            stateTraces.tx = stateTraces.tx.concat(vtorques[0]);
            stateTraces.tx.splice(0, 1);
            stateTraces.ty = stateTraces.ty.concat(vtorques[1]);
            stateTraces.ty.splice(0, 1);
            stateTraces.tz = stateTraces.tz.concat(vtorques[2]);
            stateTraces.tz.splice(0, 1);

            stateTraces.t1 = stateTraces.t1.concat(rtorques[0]);
            stateTraces.t1.splice(0, 1);
            stateTraces.t2 = stateTraces.t2.concat(rtorques[1]);
            stateTraces.t2.splice(0, 1);
            stateTraces.t3 = stateTraces.t3.concat(rtorques[2]);
            stateTraces.t3.splice(0, 1);

            Plotly.update('imu_plot', {
                y: [stateTraces.imu_x, stateTraces.imu_y, stateTraces.imu_z]
            });
            Plotly.update('vtorq_plot', {
                y: [stateTraces.tx, stateTraces.ty, stateTraces.tz]
            });
            Plotly.update('rtorq_plot', {
                y: [stateTraces.t1, stateTraces.t2, stateTraces.t3]
            });
        } else {
            console.log("An error with the serial reception code on serverside has ocurred.");
        }
    });

});