import serial
import numpy as np
Supatipunno = serial.Serial(port= 'COM8', baudrate=576000,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)

try:
    
    Supatipunno.is_open
    print('\nCOMMUNICATION WITH PORT NAME : ' + Supatipunno.portstr)
    print('UART PORT IS OPEN. READY TO COMMUNICATION \n')
    
except:
    Supatipunno.open()
    print('UART PORT IS CLOSE. PLEASE TRY AGAIN LATER \n')

data = [255,16,2,0,0,0,0,0,0,0,0,0,0,0,0,160,105]
Supatipunno.write(serial.to_bytes(data))
print(serial.to_bytes(data))