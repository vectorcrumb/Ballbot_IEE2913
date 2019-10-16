import serial, time

ser = serial.Serial("COM9", baudrate=115200)

while True:
    ser.write("U10.00,-3.40,43.52\n".encode('utf-8'))
    print("{}: Sent packet!".format(time.time()))
    time.sleep(0.5)