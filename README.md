# Wunderbar NRF51822 modules reverse-engineering
That's a work-in-progress, please help us contributing with more info and sketches!
## How it works
### NRF side
NRF code is quite easy and it basically runs standalone, waiting for new data request possibly sent by Raspberry over BLE
1. Device initializes various ble services: Temperature and humidity data (thanks to connected HTU21D sensor) and VCC voltage (provided by CR2032 battery). No external hardware is required to measure VCC voltage as NRF51822 has an internal voltage reference which can compare to its own supply voltage (selecting the appropriate 1/3 prescaler). You can find more info [here](https://os.mbed.com/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/)
2. Device sets power mode (actually I discovered `NRF_POWER_MODE_LOWPWR` is anyway selected but it's good to leave the instruction there)
3. Device goes to sleep and awaits a change event on the LedService (you can send any value to that service in order to wake up the NRF and do its things)
4. When the LedService changes its value it gets sensor data and VCC voltage and updates services value accordingly
5. Device goes back to sleep again and waits for a new event change to wake up

**NOTE!** you can change `ADVERTISING_INTERVAL` in order to get faster connection from Raspberry Pi (in some cases I tested, setting a higher advertising interval value to more than 3000 makes the Raspberry not connect at all). Reducing the value of course increases power consumption as NRF has to send advertising packets more often.
With the current setup I measured with a shunt resistor and oscilloscope a average current (Raspberry requests data each 30 seconds) of ~3.5µA, the CR2032 battery (210 mAh rated) should last 7 years __without considering__ self-discharge rate 

### Raspberry side
You'll need `bluepy` python module: `pip install bluepy`
1. Raspberry tries to connect to NRF51 through BLE, if it fails it tries again infinitely (definitely something you should avoid but that's fine for me)
2. Once it connect it updates LedService value in order to trigger a new measurement from the NRF
3. Waits for a new value thanks to BLE service notification system. This allows to get the latest value from the device. If you wanted to get the new value without notification system check [`wb-temp-read.py`](src/wb-temp-read.py).
4. Performs HTTP get request to upload data to my Dashboard (Emoncms in my case), but you can of course do whatever you want when you get your data

**NOTE!** If you are using `wb-temp-read.py` example when you read data you are actually getting the previous value because the measure and update service on the NRF takes time and you read data from the service when it hasn't updated yet. That's why I use the notify example for this kind of task.

## NRF51822 code
That's the [code](src/sleep_greenhouse/sleep_greenhouse.ino) actually running on my NRF51822 custom board:

```C
#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"


#include <Wire.h>
#include <HTU21D.h>

HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);

#define ADVERTISING_INTERVAL 3000

///
void characteristicWrittenCallback(BLECentral&, BLECharacteristic&);
///

BLEPeripheral blePeripheral = BLEPeripheral();

//BLEService ledService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLEService ledService = BLEService("AAA0");
BLECharCharacteristic ledCharacteristic = BLECharCharacteristic("AAA1", BLERead | BLEWrite);

BLEService tempService = BLEService("CCC0");
BLEFloatCharacteristic tempCharacteristic = BLEFloatCharacteristic("CCC1", BLERead | BLENotify);
BLEDescriptor tempDescriptor = BLEDescriptor("2901", "Temp Celsius");

BLEService humidityService = BLEService("DDD0");
BLEFloatCharacteristic humidityCharacteristic = BLEFloatCharacteristic("DDD1", BLERead | BLENotify);
BLEDescriptor humidityDescriptor = BLEDescriptor("2901", "Humidity Percent");

BLEService voltageService = BLEService("EEE0");
BLEFloatCharacteristic voltageCharacteristic = BLEFloatCharacteristic("EEE1", BLERead | BLENotify);
BLEDescriptor voltageDescriptor = BLEDescriptor("2901", "Supply Voltage");

void setup() {
  //Serial.begin(115200);

  myHTU21D.begin();
  myHTU21D.setResolution(HTU21D_RES_RH12_TEMP14);

  //Serial.println(F("HTU21D, SHT21 sensor is active"));

  my_analogin_init();
  
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());
  blePeripheral.setTxPower(0);
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(ledCharacteristic);
  blePeripheral.setLocalName("WB_TH");


  blePeripheral.setAdvertisedServiceUuid(tempService.uuid());
  blePeripheral.addAttribute(tempService);
  blePeripheral.addAttribute(tempCharacteristic);
  blePeripheral.addAttribute(tempDescriptor);

  blePeripheral.setAdvertisedServiceUuid(humidityService.uuid());
  blePeripheral.addAttribute(humidityService);
  blePeripheral.addAttribute(humidityCharacteristic);
  blePeripheral.addAttribute(humidityDescriptor);

  blePeripheral.setAdvertisedServiceUuid(voltageService.uuid());
  blePeripheral.addAttribute(voltageService);
  blePeripheral.addAttribute(voltageCharacteristic);
  blePeripheral.addAttribute(voltageDescriptor);

  blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);
  ledCharacteristic.setEventHandler(BLEWritten, characteristicWrittenCallback);

  blePeripheral.begin();
  
  float vcc;
  vcc = (float)my_analogin_read_u16();
  vcc = (vcc * 3.6) / 1024.0; //to read VCC. Copied from https://os.mbed.com/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/

  myHTU21D.begin();
  myHTU21D.setResolution(HTU21D_RES_RH12_TEMP14);
  // central wrote new value to characteristic, update LED
  float temp = myHTU21D.readTemperature(HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD);
  float hum = myHTU21D.readCompensatedHumidity(HTU21D_TRIGGER_HUMD_MEASURE_NOHOLD);
  myHTU21D.setResolution(HTU21D_RES_RH11_TEMP11);
  Wire.end();
  pinMode(25, INPUT); // set SDA pin to input mode
  pinMode(24, INPUT); // set SCL pin to input mode
  //float temp = 11.21, hum=12.32; //for testing
  tempCharacteristic.setValue(temp);
  humidityCharacteristic.setValue(hum);
  voltageCharacteristic.setValue(vcc);
  
  // enable low power mode without interrupt

  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
}

void loop() {
  // Enter Low power mode
  sd_app_evt_wait();
  // Exit Low power mode

  // Clear IRQ flag to be able to go to sleep if nothing happens in between
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

  // poll peripheral
  blePeripheral.poll();
}

void characteristicWrittenCallback(BLECentral& central, BLECharacteristic& characteristic) {
  float vcc;
  vcc = (float)my_analogin_read_u16();
  vcc = (vcc * 3.6) / 1024.0; //to read VCC. Copied from https://os.mbed.com/users/MarceloSalazar/notebook/measuring-battery-voltage-with-nordic-nrf51x/

  myHTU21D.begin();
  myHTU21D.setResolution(HTU21D_RES_RH12_TEMP14);
  // central wrote new value to characteristic, update LED
  float temp = myHTU21D.readTemperature(HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD);
  float hum = myHTU21D.readCompensatedHumidity(HTU21D_TRIGGER_HUMD_MEASURE_NOHOLD);
  myHTU21D.setResolution(HTU21D_RES_RH11_TEMP11);
  Wire.end();
  pinMode(25, INPUT); // set SDA pin to input mode
  pinMode(24, INPUT); // set SCL pin to input mode
  //float temp = 11.21, hum=12.32; //for testing
  tempCharacteristic.setValue(temp);
  humidityCharacteristic.setValue(hum);
  voltageCharacteristic.setValue(vcc);

}

void my_analogin_init(void)
{
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
}

uint16_t my_analogin_read_u16(void)
{
  NRF_ADC->CONFIG     &= ~ADC_CONFIG_PSEL_Msk;
  NRF_ADC->CONFIG     |= ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos;
  NRF_ADC->TASKS_START = 1;
  while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {};
  return (uint16_t)NRF_ADC->RESULT; // 10 bit
}
```

## Raspberry Pi 3B as BLE Gateway
That's the [code](src/wb-temp-notify.py) running on my Raspberry Pi, which serves as a Bluetooth BLE gateway:
![[Microcontrollori/NRF51-Wunderbar/nrf51-resurfacing/src/wb-temp-notify.py]]
```python
import struct
import time
import requests
from bluepy.btle import UUID, Peripheral, DefaultDelegate, BTLEDisconnectError

led_service_uuid = UUID(0xAAA0)
led_char_uuid = UUID(0xAAA1)
mac_address = "aa:bb:cc:dd:ee:ff" # you should put your device (NRF51) mac address here in order to connect to it 
apikey = "aaaaaaaaaaa"

class MyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        print("A notification was received: ", cHandle)
        temp = round(struct.unpack('<f', ch1.read())[0], 2)    
        hum = round(struct.unpack('<f', ch2.read())[0], 2)
        vcc = round(struct.unpack('<f', ch3.read())[0], 2)
        if temp==0.00 and hum == 0.00: #to clean bad data (could be teoretically left here)
            print("Skipping...")
        else:
            print(temp, hum, vcc)
            req = "https://www.example.com/emoncms/input/post?node=10&apikey="+apikey+"&csv="+str(temp)+","+str(hum)+","+str(vcc)         
            try:	
                r=requests.get(req)
                print(r)
            except:
                print(r)
                print("Error...")
        p.disconnect()


def countdown(t):    
    while t:
        mins, secs = divmod(t, 60)
        timer = '{:02d}:{:02d}'.format(mins, secs)
        print(timer, end="\r")
        time.sleep(1)
        t -= 1

def connect():
    print("Connecting...")
    global p, LedService, svc1, svc2, svc3, ch1, ch2, ch3
    try:
        p = Peripheral(mac_address, "random")
        p.setDelegate( MyDelegate() )
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
    
    #p.writeCharacteristic(ch1.valHandle+1, b"\x01\x00")
    #p.writeCharacteristic(ch2.valHandle+1, b"\x01\x00")
    p.writeCharacteristic(ch3.valHandle+1, b"\x01\x00") #used to subscribe to ch3 notification (if NRF51 sends it) that could be handled thanks to handeNotification above
    print(ch1.valHandle, ch2.valHandle, ch3.valHandle)
    #p.disconnect()


#p.writeCharacteristic(ch1.valHandle+1, b"\x01\x00")
#p.writeCharacteristic(ch2.valHandle+1, b"\x01\x00")
    
while 1:
    try:
        connect()
        wake = LedService.getCharacteristics(led_char_uuid)[0]
        wake.write(struct.pack('<B', 0x00))
        p.waitForNotifications(5.0)
        countdown(30)
    except BTLEDisconnectError:
        print("Error, waiting for next connection...")
    except KeyboardInterrupt:
        p.disconnect()
        exit()
    
```
