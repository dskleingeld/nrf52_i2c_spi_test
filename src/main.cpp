#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "SparkFunHTU21D.h"
#include "Adafruit_SHT31.h"
#include "AD7793.hpp"

/*typedef enum {
  PIN_SPI1_MISO = 15,  //(DOUT/~RDY), Master In Slave Out (MISO) + another pin
  PIN_SPI1_SCK = 16,   //(CLK), Serial Clock (SCK),
  PIN_SPI1_MOSI = 19,  //(DIN), Master Out Slave In (MOSI)
};*/

/*
The TWI[N] and SPI[N] shares registers and other resources with eachter.
You can not use TWI[0] and SPI[0] at the same time.
Here we configure the nrf for 2 TWI interfaces using master 0 and 1
And one SPI master using master 2
*/

//SPI uses nrf_spi0
SPIClass SPI1(NRF_SPI2, 15, 16, 19);

typedef enum {
  PIN_WIRE_SDA2 = 11, 
  PIN_WIRE_SCL2 = 12,
};

//pins for hardcoded normal I2C
//PIN_WIRE_SDA (25u)
//PIN_WIRE_SCL (26u)

TwoWire Wire2(NRF_TWIM0, NRF_TWIS0, 
  SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, PIN_WIRE_SDA2, PIN_WIRE_SCL2);

extern "C"
{
  void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
  {
    Wire2.onService();
  }
}

SoftwareSerial soft_serial(A0, A1); // RX, TX
Adafruit_SHT31 sht31 = Adafruit_SHT31();
HTU21D myHumidity;
AD7793 adc;

void setup() {
  Serial.begin(9600);
  Serial.println("starting up");

  Wire.begin();
  Wire2.begin();
  myHumidity.begin(Wire2);
  
  SPI1.begin();
  auto res = adc.begin(22,23, SPI1);
  if (res != 0) {
    while (true){
      Serial.println("Could not find adc"); 
      delay(1);
    }
  }
  Serial.print("AD7793 status = ");
  Serial.println(res); /* Answer is 1 when the device is initialized and the ID is read and recognized */ 

  adc.Reset();
  adc.SetChannel(AD7793_CH_AVDD_MONITOR);
  adc.SetGain(AD7793_GAIN_1);
  adc.SetIntReference(AD7793_REFSEL_INT); /* Sets the reference source for the ADC. */

  Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    while (true) {
      delay(1);
      Serial.println("Couldn't find SHT31");
    }
  }

  delay(2000);
}

void loop() {
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();

  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
  Serial.println();

  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  Serial.println();

  Serial.println("trying to set channel");
  adc.SetChannel(AD7793_CH_AVDD_MONITOR); 
  Serial.println("set channel");

  unsigned long conv = adc.SingleConversion();
  Serial.print("raw value from chip: ");
  Serial.println(conv);
  float AVDD = ((conv - 8388608.0) / 8388608.0) * 1.17 / (1/6.0) ;
  Serial.print("Analog supply voltage (AVDD) = ");
  Serial.print(AVDD, 8);
  Serial.println(" V");
  
  delay(1000);
}