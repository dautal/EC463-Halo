#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// Bluetooth® Low Energy Battery Service
BLEService VoltageService("75340d9a-b70d-11ed-afa1-0242ac120002");
BLEStringCharacteristic VoltageChar("84244464-b70d-11ed-afa1-0242ac120002", BLERead | BLENotify, 20);


#define AVG_WINDOW 10

float computeAvg(int *myArrg);
void collectPoints(int sampleInterval, int pinNum,int pinNumN, int *myArrg);
void updateVoltage();

//variables
long previousMillis = 0;
int dispRefresh = 450;  //how often the display updates, needs to be >100
int voltInputPinP = 4; //pin connected to voltmeter positive terminal
int voltInputPinN = 5; //pin connected to voltmeter positive terminal
int sampleInterval = 1; // time between multiple samples
int ADCRaw[AVG_WINDOW] = {0};
float ADCconversion = 0.00488;
float myAverage;
float voltage;
float ax, ay, az; // variables to store accelerometer readings

void setup() {
  Serial.begin(9600); // initialize serial communication

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Halo_8"); //set name here
  BLE.setAdvertisedService(VoltageService); // add the service UUID
  VoltageService.addCharacteristic(VoltageChar); // add the battery level characteristic
  BLE.addService(VoltageService); // Add the battery service
  VoltageChar.writeValue("0"); // set initial value for this characteristic

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM9DS1!");
    while (1);
  }
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    delay(dispRefresh);
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (BLE.connected()) {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 300) {
        previousMillis = currentMillis;
        updateVoltage();
      }
    }

    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

float computeAvg(int *myArrg) { 
  int accumulator = 0;
  for (int j = 0; j < AVG_WINDOW; j++) {
    accumulator += myArrg[j];
  }
  return (float)accumulator / AVG_WINDOW;;
}


void collectPoints(int sampleInterval, int pinNumP, int pinNumN, int *myArrg) {
  unsigned long previousMillis = 0;
  int i = 0;
  while (i < AVG_WINDOW) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= sampleInterval) {
      previousMillis = currentMillis;
      if (pinNumN == 'G') {
        myArrg[i] = analogRead(pinNumP);
      } else {
        myArrg[i] = analogRead(pinNumP) - analogRead(pinNumN);
      }
      ++i;
    }
  }
}

void updateVoltage() {
  collectPoints(sampleInterval, voltInputPinP,voltInputPinN, ADCRaw);
  myAverage = computeAvg(ADCRaw);
  float newVoltage = myAverage * ADCconversion * 100;
  IMU.readAcceleration(ax, ay, az);
  Serial.println(az);
  voltage = newVoltage;
  String data = String(voltage) + "," + String(az);
  VoltageChar.writeValue(data);
  Serial.println(data);
  /*Serial.print("Voltage: ");
  Serial.println(newVoltage);
  Serial.print("Old volt: ");
  Serial.println(voltage);*/
}
