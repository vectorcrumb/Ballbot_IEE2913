import serial, time

ser = serial.Serial("COM10", baudrate=115200)

while True:
    ser.flush()
    ser.write("U10.00,-3.40,43.52,2.0,3.0,0.5,0.1,0.8,1.3\n".encode('utf-8'))
    print("{}: Sent packet!".format(time.time()))
    time.sleep(1)