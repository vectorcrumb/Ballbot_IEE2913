from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from threading import Lock
import serial

thread_ser = None
thread_lock = Lock()
freq = 1
name_space = '/ballbot'
n_clients = 0
# ser_port = '/dev/ttyS0'
ser_port = "COM9"
ser = serial.Serial(ser_port, baudrate=115200, timeout=freq/4)

def serial_thread():
    imu_vals = [0, 0, 0]
    while True:
        print("hi!")
        socketio.sleep(freq)
        try:
            data = ser.readline()
            imu_vals = list(map(float, data.decode('utf-8').strip().split(',')))
        except:
            if data:
                print(data)
            imu_vals = [0, 0, 0]
        data = None
        socketio.emit("update", {
            'imu': imu_vals,
        }, namespace=name_space)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'ballbot1'
app.config['TEMPLATES_AUTO_RELOAD'] = True
socketio = SocketIO(app, async_mode=None)
with thread_lock:
    if thread_ser is None:
        thread_ser = socketio.start_background_task(serial_thread)


@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect', namespace=name_space)
def connect_event():
    global n_clients#, thread_ser
    # with thread_lock:
    #     if thread_ser is None:
    #         thread_ser = socketio.start_background_task(serial_thread)
    n_clients += 1
    print("Client connected.")
    emit('clients', {
        'data': n_clients
    }, namespace=name_space, broadcast=True)

@socketio.on('disconnect', namespace=name_space)
def disconnect_event():
    global n_clients
    n_clients -= 1
    print("Client disconnected.")
    emit('clients', {
        'data': n_clients
    }, namespace=name_space, broadcast=True)

@socketio.on('broadcast', namespace=name_space)
def broadcast_echo(payload):
    print("Received broadcast request!\n{}".format(payload))
    emit(payload['event'], {
        'data': payload['data']
    }, namespace=name_space, broadcast=True)


if __name__ == '__main__':
    print("Initializing BallBot Server.")
    socketio.run(app, host='0.0.0.0', debug=True, port=5800)

