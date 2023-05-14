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
