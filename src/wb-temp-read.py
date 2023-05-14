import sys
import binascii
import struct
import time
import requests
import subprocess
from bluepy.btle import UUID, Peripheral, DefaultDelegate, BTLEDisconnectError

led_service_uuid = UUID(0xAAA0)
led_char_uuid = UUID(0xAAA1)
mac_address = "aa:bb:cc:dd:ee:ff" # you should put your device (NRF51) mac address here in order to connect to it 
apikey = "aaaaaaaaaaa"

def connect():
    print("Connecting...")
    global p, LedService, svc1, svc2, svc3, ch1, ch2, ch3
    try:
        p = Peripheral(mac_address, "random")
        LedService=p.getServiceByUUID(led_service_uuid)
    except BTLEDisconnectError:
        print("Connection failed, retrying...")
        connect()
        return
    except KeyboardInterrupt:
        p.disconnect()
        exit()
    print("Connected")

    svc1 = p.getServiceByUUID( UUID(0xCCC0) )
    ch1 = svc1.getCharacteristics()[0]
    svc2 = p.getServiceByUUID( UUID(0xDDD0) )
    ch2 = svc2.getCharacteristics()[0]
    svc3 = p.getServiceByUUID( UUID(0xEEE0) )
    ch3 = svc3.getCharacteristics()[0]
    print(ch1.valHandle, ch2.valHandle)
    #p.disconnect()


#p.writeCharacteristic(ch1.valHandle+1, b"\x01\x00")
#p.writeCharacteristic(ch2.valHandle+1, b"\x01\x00")
    
while 1:
    try:
        connect()
        wake = LedService.getCharacteristics(led_char_uuid)[0]
        wake.write(struct.pack('<B', 0x00))
        temp = round(struct.unpack('<f', ch1.read())[0], 2)    
        hum = round(struct.unpack('<f', ch2.read())[0], 2)
        vcc = round(struct.unpack('<f', ch3.read())[0], 2)
        if temp==0.00 and hum == 0.00:
            print("Skipping...")
        else:
            p.disconnect()
            print(temp, hum, vcc)
            req = "https://www.example.com/emoncms/input/post?node=10&apikey="+apikey+"&csv="+str(temp)+","+str(hum)+","+str(vcc)         
            try:	
                r=requests.get(req)
                print(r)
            except:
                print(r)
                print("Error...")
        time.sleep(30)
    except KeyboardInterrupt:
        p.disconnect()
        exit()
    
