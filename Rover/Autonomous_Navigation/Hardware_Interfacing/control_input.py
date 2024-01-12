import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

try:
    while True:
        control_input = ser.readline()#.decode('latin-1').strip('\r\n').split(',')
        #control_input = [int(value) for value in control_input]
        #x = int(control_input[0])
        print(control_input)
        '''if line.startswith('(') and line.endswith(')'):
            values = line[1:-1].split(',')
            print(values)
            x = int(values[0])
            y = int(values[1])
            
            print(f"({x}, {y})")'''
        
        time.sleep(0.01)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed!")
