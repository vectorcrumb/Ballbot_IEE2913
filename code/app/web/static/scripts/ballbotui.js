$(document).ready(function() {

    // Obtain canvas object for battery voltage display
    var canvasBattery = document.getElementById("battery-animation");
    var canvas2DBattery = canvasBattery.getContext("2d");
    // Connect to websockets server with namespace /webui
    namespace = '/ballbot';
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + location.port + namespace, { 'sync disconnect on unload': true });
    
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
        var imu = payload['imu'];
        console.log(imu);
    })

});