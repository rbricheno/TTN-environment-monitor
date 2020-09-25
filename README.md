# TTN environment monitor
*By David Batey, Josh Milroy and Rob Bricheno*

## Getting started

For this project you will need the following hardware:

* Arduino Uno - https://store.arduino.cc/arduino-uno-rev3

![Arduino Uno](/images/arduino-uno.jpg?raw=true "Arduino Uno")

* BME280 sensor in a breakout board - https://www.bosch-sensortec.com/bst/products/all_products/bme280 - You can find these on Amazon or eBay. Ours runs at 5V.

![BME280](/images/bme280.jpg?raw=true "BME280")

* UNIROI Starter Kit (or another Arduino starter kit) - http://uniroi.net/a/Products/20190401/70.html
* Pi Supply Arduino LoRa node shield  - https://uk.pi-supply.com/products/iot-lora-node-shield

![Arduino LoRa node shield](/images/pi-supply-shield.jpg?raw=true "Arduino LoRa node shield")

Firstly, you should install the Arduino IDE from the Arduino website (https://www.arduino.cc/en/main/software).

Then you should create a The Things Network account on the website (https://account.thethingsnetwork.org/register).

After this, you will need to register your application at https://console.thethingsnetwork.org/applications/add. Here, you need to fill in the application ID and description.

Next, you will need to register your device at https://console.thethingsnetwork.org/applications/application/devices/register. You have to enter a Device ID and click the symbol next the Device EUI text box.

## Testing LoRaWAN

Now we will test "Over The Air Activation" on the Arduino and shield. Connect the shield to the top of your Arduino Uno as shown here:

![Connecting the shield](/images/arduino-with-shield.jpg?raw=true "Connecting the shield")

In the Arduino IDE, create a new sketch.

You will need to download and install the RAK811 Arduino Library from https://github.com/PiSupply/RAK811-Arduino . Download the RAK811-Arduino repository as a Zip file and in the Arduino IDE use "Sketch -> Add Library -> Add .ZIP library" and add the downloaded zip file.

Now enter this code: 

```c
#include "RAK811.h"
#include "SoftwareSerial.h"
#define WORK_MODE LoRaWAN   //  LoRaWAN or LoRaP2P
#define JOIN_MODE OTAA    //  OTAA or ABP
#if JOIN_MODE == OTAA
String DevEui = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill this out
String AppEui = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill this out
String AppKey = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill This out
#else JOIN_MODE == ABP
String NwkSKey = "";
String AppSKey = "";
String DevAddr = "";
#endif
#define TXpin 11   // Set the virtual serial port pins
#define RXpin 10
#define DebugSerial Serial
SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port
char* buffer = (char*) "72616B776972656C657373";
int RESET_PIN = 12;
bool InitLoRaWAN(void);
RAK811 RAKLoRa(RAKSerial,DebugSerial);
 
 
void setup() {
  // Define Reset Pin
  pinMode(RESET_PIN, OUTPUT);
  
  // Setup Debug Serial on USB Port
  DebugSerial.begin(9600);
  while(DebugSerial.read()>= 0) {}
  while(!DebugSerial);
  //Print debug info
  DebugSerial.println("StartUP");
  DebugSerial.println("Reset");
  //Reset the RAK Module
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
  DebugSerial.println("Success");
  RAKSerial.begin(9600); // Arduino Shield
  delay(100);
  DebugSerial.println(RAKLoRa.rk_getVersion());
  delay(200);
  DebugSerial.println(RAKLoRa.rk_getBand());
  delay(200);
 
  while (!InitLoRaWAN());
}

bool InitLoRaWAN(void)
{
  RAKLoRa.rk_setWorkingMode(WORK_MODE);
  RAKLoRa.rk_recvData();
  RAKLoRa.rk_recvData();
  if ( RAKLoRa.rk_recvData() == "OK")
  {
    if (RAKLoRa.rk_initOTAA(DevEui, AppEui, AppKey))
    {
      DebugSerial.println("You init OTAA parameter is OK!");
      while (RAKLoRa.rk_joinLoRaNetwork(JOIN_MODE))
      {
        bool flag = false;
        for (unsigned long start = millis(); millis() - start < 90000L;)
        {
          String ret = RAKLoRa.rk_recvData();
          if (ret.startsWith(STATUS_JOINED_SUCCESS))
          {
            DebugSerial.println("You join Network success!");
            return true;
          }
          else if (ret.startsWith(STATUS_RX2_TIMEOUT) || ret.startsWith(STATUS_JOINED_FAILED))
          {
            DebugSerial.println("You join Network Fail!");
            flag = true;
            DebugSerial.println("The device will try to join again after 5s");
            delay(5000);
          }
        }
        if (flag == false)
        {
          DebugSerial.println("Pleases Reset the module!");
          delay(1000);
          return false;
        }
      }
    }
  }
  return false;
}
 
void loop() {
  int packetsflag = 1; // 0: unconfirmed packets, 1: confirmed packets
  if (RAKLoRa.rk_sendData(packetsflag, 1, buffer))
  {
    for (unsigned long start = millis(); millis() - start < 90000L;)
    {
      String ret = RAKLoRa.rk_recvData();
      if (ret.startsWith(STATUS_TX_COMFIRMED) || ret.startsWith(STATUS_TX_UNCOMFIRMED))
      {
        DebugSerial.println("Send data ok!");
        delay(5000);
        return;
      }
    }
    DebugSerial.println("Send data error!");
    while (1);
  }
}
```

Fill out the Dev Eui, App Eui and App Key which you can find on the TTN website (https://console.thethingsnetwork.org/applications/application/devices/device ).

Run this code on your Arduino by pressing “Upload” and then open the TTN website. In the TTN website go the data page (https://console.thethingsnetwork.org/applications/application/devices/device/data). Here, the payload should state “72616B776972656C657373”.

## Adding the BME280

Next we will test the BME280 sensor. Either plug the sensor into the arduino directly or the same pins on the shield. The pins should be connected:

| BME280  | Arduino |
| ------------- | ------------- |
| VIN | 5V  |
| GND  | GND |
| SCL | A5  |
| SDA  | A4 |

It should look something like this when all connected up:

![BME280 connections](/images/everything.jpg?raw=true "BME280 connections")

You will need to install the Adafruit BME280 library and Adafruit unified sensors library, both of which should be available to find in the "Add library" tool within the Arduino IDE.

Then type this code in a new Arduino window:

```c
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
 
#define SEALEVELPRESSURE_HPA (1013.25)
 
Adafruit_BME280 bme; // I2C
 
unsigned long delayTime;
 
void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));
 
    unsigned status;
    
    // default settings
    // status = bme.begin();
    // We found that since version 2 of the Adafruit library
    // we have to initialise our (non-Adafruit-branded) I2C
    // BME280 like this:
    status = bme.begin(0x76, &Wire);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 1000;
 
    Serial.println();
}
 
 
void loop() { 
    printValues();
    delay(delayTime);
}
 
 
void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
 
    Serial.print("Pressure = ");
 
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
 
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
 
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
 
    Serial.println();
}
```

This program should return 3 values (Temperature, Humidity and Pressure) into the serial monitor which can be accessed by Ctrl+Shift+M. Just be wary that some shields work at faster speed, so the baud rate must be changed.

## Sending BME280 data into The Things Network

The next step is to send the BME 280 data to TTN in Cayenne format. This is done through this code:

```c
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RAK811.h"
#include "SoftwareSerial.h"
#define WORK_MODE LoRaWAN   //  LoRaWAN or LoRaP2P
#define JOIN_MODE OTAA    //  OTAA or ABP
#if JOIN_MODE == OTAA
String DevEui = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill this out
String AppEui = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill this out
String AppKey = "<PUT YOUR KEY HERE FROM TTN CONSOLE>"; // Fill This out
#endif
#define TXpin 11   // Set the virtual serial port pins
#define RXpin 10
#define DebugSerial Serial
 
Adafruit_BME280 bme; // I2C
CayenneLPP lpp(51);
SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port
 
int RESET_PIN = 12;
bool InitLoRaWAN(void);
RAK811 RAKLoRa(RAKSerial,DebugSerial);
 
void setup() {
  //Define Reset Pin
  pinMode(RESET_PIN, OUTPUT);
  //Setup Debug Serial on USB Port
  DebugSerial.begin(9600);
  while(DebugSerial.read()>= 0) {}
  while(!DebugSerial);
  //Print debug info
  DebugSerial.println("StartUP");
  DebugSerial.println("Reset");
  //Reset the RAK Module
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
  DebugSerial.println("Success");
  RAKSerial.begin(9600); // Arduino Shield
  delay(100);
  DebugSerial.println(RAKLoRa.rk_getVersion());
  delay(200);
  DebugSerial.println(RAKLoRa.rk_getBand());
  delay(200);
 
  while (!InitLoRaWAN());

  bme.begin(0x76, &Wire);  
}

bool InitLoRaWAN(void)
{
  RAKLoRa.rk_setWorkingMode(WORK_MODE);
  RAKLoRa.rk_recvData();
  RAKLoRa.rk_recvData();
  if ( RAKLoRa.rk_recvData() == "OK")
  {
 
    if (RAKLoRa.rk_initOTAA(DevEui, AppEui, AppKey))
    {
      DebugSerial.println("You init OTAA parameter is OK!");
      while (RAKLoRa.rk_joinLoRaNetwork(JOIN_MODE))
      {
        bool flag = false;
        for (unsigned long start = millis(); millis() - start < 90000L;)
        {
          String ret = RAKLoRa.rk_recvData();
          if (ret.startsWith(STATUS_JOINED_SUCCESS))
          {
            DebugSerial.println("You join Network success!");
            return true;
          }
          else if (ret.startsWith(STATUS_RX2_TIMEOUT) || ret.startsWith(STATUS_JOINED_FAILED))
          {
            DebugSerial.println("You join Network Fail!");
            flag = true;
            DebugSerial.println("The device will try to join again after 5s");
            delay(5000);
          }
        }
        if (flag == false)
        {
          DebugSerial.println("Pleases Reset the module!");
          delay(1000);
          return false;
        }
      }
    }
  }
     return false;
}
 
void loop() {
 
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
 
  Serial.print("Pressure = ");
  Serial.print( bme.readPressure());
  Serial.println(" hpa");
 
  Serial.print("Humidity = ");
  Serial.print( bme.readHumidity());
  Serial.println(" rh");
 
 
  lpp.reset();
  lpp.addTemperature(1, bme.readTemperature());
  lpp.addRelativeHumidity(2, bme.readHumidity());
  lpp.addBarometricPressure(3, bme.readPressure());
  
  int packetsflag = 1; // 0: unconfirmed packets, 1: confirmed packets
  if (RAKLoRa.rk_sendBytes(packetsflag, 1, lpp.getBuffer(), lpp.getSize()))
  {
    for (unsigned long start = millis(); millis() - start < 90000L;)
    {
      String ret = RAKLoRa.rk_recvData();
      if (ret.startsWith(STATUS_TX_COMFIRMED) || ret.startsWith(STATUS_TX_UNCOMFIRMED))
      {
        DebugSerial.println("Send data ok!");
        delay(60000);
        return;
      }
    }
    DebugSerial.println("Send data error!");
    while (1);
  }
}
```

Fill out the Dev Eui, App Eui and App Key which you can find on the TTN website (https://console.thethingsnetwork.org/applications/application/devices/device ). Then on the TTN website, click on the payload format tab (https://console.thethingsnetwork.org/applications/application/payload-formats). Then click on the drop-down menu and choose Cayenne. Now, you can upload the program to the Arduino and the data should appear in the payloads.

## Adding a MyDevices dashboard

Finally, click on the Intergrations tab (https://console.thethingsnetwork.org/applications/application/integrations) and click “add integration” (https://console.thethingsnetwork.org/applications/application/integrations/create).

Choose myDevices (https://console.thethingsnetwork.org/applications/application/integrations/create/http-cayenne) and enter a Process ID and click on the access key drop-down box and choose default key.

Create an account at https://accounts.mydevices.com/auth/realms/cayenne/login-actions/registration?client_id=cayenne-web-app&tab_id=27_MIrlvDiU .

Choose LoRa (https://cayenne.mydevices.com/cayenne/dashboard/first-visit/lora) and then scroll down and choose The Things Network (Under the networks sidebar) and then choose Cayenne Cayenne LPP.

Enter your Device Eui from the TTN website (https://console.thethingsnetwork.org/applications/application/devices/device ) and then press add device.

You should then run the program on your Arduino and the data should appear on myDevices dashboard and be updated every minute (https://cayenne.mydevices.com/cayenne/dashboard/lora/).


![MyDevices dashboard](/images/dashboard.png?raw=true "MyDevices dashboard")

## Activation by personalisation method

If you can't get OTAA to work (because you have poor downlink reception from the gateway for example) then you can use the following code instead to activate your node using ABP and still complete the workshop.

```c
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RAK811.h"
#include "SoftwareSerial.h"
#define WORK_MODE LoRaWAN   //  LoRaWAN or LoRaP2P
#define JOIN_MODE ABP   //  OTAA or ABP
#if JOIN_MODE == ABP
String NwkSKey = ""; // Fill this out
String AppSKey = ""; // Fill this out
String DevAddr = ""; // Fill this out
#endif
#define TXpin 11   // Set the virtual serial port pins
#define RXpin 10
#define DebugSerial Serial

Adafruit_BME280 bme; // I2C
CayenneLPP lpp(51);
SoftwareSerial RAKSerial(RXpin,TXpin);    // Declare a virtual serial port

int RESET_PIN = 12;
bool InitLoRaWAN(void);
RAK811 RAKLoRa(RAKSerial,DebugSerial);

void setup() {
  //Define Reset Pin
  pinMode(RESET_PIN, OUTPUT);
  //Setup Debug Serial on USB Port
  DebugSerial.begin(9600);
  while(DebugSerial.read()>= 0) {}
  while(!DebugSerial);
  //Print debug info
  DebugSerial.println("StartUP");
  DebugSerial.println("Reset");
  //Reset the RAK Module
  digitalWrite(RESET_PIN, LOW);   // turn the pin low to Reset
  digitalWrite(RESET_PIN, HIGH);    // then high to enable
  DebugSerial.println("Success");
  RAKSerial.begin(115200); // Arduino Shield. The number can be changed! Search for serial speeds
  delay(100);
  DebugSerial.println(RAKLoRa.rk_getVersion());
  delay(200);
  DebugSerial.println(RAKLoRa.rk_getBand());
  delay(200);

  while (!InitLoRaWAN());

  bme.begin(0x76, &Wire);  


}
bool InitLoRaWAN(void)
{
  RAKLoRa.rk_reset(1);
  RAKLoRa.rk_setWorkingMode(WORK_MODE);
  RAKLoRa.rk_recvData();
  RAKLoRa.rk_recvData();
  if ( RAKLoRa.rk_recvData() == "OK")
  {
    if (RAKLoRa.rk_initABP(DevAddr, NwkSKey, AppSKey))
    {
      Serial.println("You init ABP parameter is OK!");
      if (RAKLoRa.rk_joinLoRaNetwork(JOIN_MODE))
      {
        Serial.println("You join Network success!");
        return true;
      }
    }
  }
  return false;
}

void loop() {

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  lpp.reset();
  lpp.addTemperature(1, bme.readTemperature());
  lpp.addBarometricPressure(2, bme.readPressure());
  lpp.addRelativeHumidity(3, bme.readHumidity());
    
  int packetsflag = 1; // 0: unconfirmed packets, 1: confirmed packets
  if (RAKLoRa.rk_sendBytes(packetsflag, 1, lpp.getBuffer(), lpp.getSize()))
  {
    for (unsigned long start = millis(); millis() - start < 6000L;)
    {
      String ret = RAKLoRa.rk_recvData();
      if (ret.startsWith(STATUS_TX_COMFIRMED) || ret.startsWith(STATUS_TX_UNCOMFIRMED))
      {
        DebugSerial.println("Send data ok!");
        delay(5000);
        return;
      }
    }
    DebugSerial.println("Send data error!");
        delay(5000);
  }
  RAKLoRa.rk_recvData();

  
}
```

## We hope you had fun!

*This work is licensed under Creative Commons Attribution 4.0* https://creativecommons.org/licenses/by/4.0/legalcode
