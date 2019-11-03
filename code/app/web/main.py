from flask import Flask, render_template, flash, redirect, request
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename
from threading import Lock
import time, platform, os
import serial


thread_ser = None
thread_lock = Lock()
freq = 1
name_space = '/ballbot'
n_clients = 0
UPLOAD_FOLDER = 'uploads'
ALLOWED_EXTENSIONS = {'txt', 'csv', 'bin'}

if platform.system() == 'Windows':
    ser_port = "COM9"
    UPLOAD_FOLDER = "E:\\Users\\lucas\\GoogleDrive\\PUC\\CURSOS\\2019-2\\IEE2913 - Diseño eléctrico\\Ballbot_IEE2913\\code\\app\\web\\uploads"
else:
    ser_port = '/dev/ttyS0'
    UPLOAD_FOLDER = "/home/pi/ballbot/web/uploads"


def serial_thread():
    ser = serial.Serial(ser_port, baudrate=115200, timeout=freq/4)
    ser.flush()
    while True:
        socketio.sleep(freq)
        try:
            data = ser.readline().decode()
            data_comm = data[0]
            data_info = data[1:].strip()
            if (data_comm == 'U'):
                # This is a serial update packet!
                values = list(map(float, data_info.split(',')))
                print("Data update command: {}".format(values))
                socketio.emit("update", {
                    'imu': values[:3],
                    'rtorq': values[3:6],
                    'vtorq': values[6:],
                    'err': 0
                }, namespace=name_space)
        except:
            print("Ooops! {}".format(data if data else "Nothing received!"))
            socketio.emit("update", {
                'err': 1
            }, namespace=name_space)
        

app = Flask(__name__)
app.config['SECRET_KEY'] = 'ballbot1'
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
socketio = SocketIO(app, async_mode=None)

with thread_lock:
    if thread_ser is None:
        thread_ser = socketio.start_background_task(serial_thread)
        print("Started background serial thread")

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'GET':
        return render_template('index.html')
    elif request.method == 'POST':
        if 'file' not in request.files:
            flash("No file uploaded")
            return redirect(request.url)
        ffile = request.files['file']
        if ffile.filename == '':
            flash("No file selected")
            return redirect(request.url)
        if ffile and allowed_file(ffile.filename):
            filename = secure_filename(ffile.filename)
            ffile.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return redirect(request.url)

@socketio.on('connect', namespace=name_space)
def connect_event():
    global n_clients
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



def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


if __name__ == '__main__':
    print("Initializing BallBot Server.")
    socketio.run(app, host='0.0.0.0', debug=True, port=5800)



