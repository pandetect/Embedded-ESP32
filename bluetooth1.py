import bluetooth
import time 


mac = '3C:61:05:17:04:76'
target_name = mac
target_address = None
sock = None 
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

def findDevices( timeout ,target ):
    first_time = time.time()
    while True: 
        second_time = time.time()
        if ( second_time - first_time < timeout):
            target_name = bluetooth.lookup_name(target)
            if target_name != None:
                return target_name
        else:
            return target_name 
def connectToDevice(timeout , target):
    first_time = time.time()
    while True: 
        second_time = time.time()
        #sock.connect((mac , 1) )
        sock.send("G")
        
        #
        break



x = findDevices(10 , mac)
sock.connect((mac , 1))
for i in range(3):
    
    connectToDevice(0 , 0 )
sock.close()
print(x)
if target_address is not None:
    print("hlloo")
    print ("found target bluetooth device with address ", target_address[1])
else:
    print ("could not find target bluetooth device nearby")
