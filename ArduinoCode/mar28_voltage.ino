#include <ArduinoBLE.h>

// Bluetooth® Low Energy Battery Service
BLEService VoltageService("75340d9a-b70d-11ed-afa1-0242ac120002");

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic VoltageChar("84244464-b70d-11ed-afa1-0242ac120002",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
long previousMillis = 0;

#define AVG_WINDOW 10
// variables
float computeAvg(int *myArrg);
void collectPoints(int sampleInterval, int pinNum,int pinNumN, int *myArrg);

//variables
int dispRefresh = 450;  //how often the display updates. needs to be >100
int voltInputPinP = 4; //pin connected to voltmeter positive terminal
int voltInputPinN = 5; //pin connected to voltmeter positive terminal
//int rInputPin = 2;  //pin connected to ohm meter
int sampleInterval = 1; // time between multiple samples
int ADCRaw[AVG_WINDOW] = {0};
int controlByte = 'V'; //
int controlFlag = 1;  // inital mode state
float ADCconversion = 0.00488;
float myAverage;
float voltage;
float current_volt=0;
float oldVoltage;  // last battery level reading from analog input


void setup() {
  Serial.begin(9600); // initialize serial communication
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("VoltageMonitor");
  BLE.setAdvertisedService(VoltageService); // add the service UUID
  VoltageService.addCharacteristic(VoltageChar); // add the battery level characteristic
  BLE.addService(VoltageService); // Add the battery service
  VoltageChar.writeValue(0); // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // measure voltage
  collectPoints(sampleInterval, voltInputPinP,voltInputPinN, ADCRaw);
  myAverage = computeAvg(ADCRaw);
  voltage = myAverage * ADCconversion * 100;
  // Serial.println(voltage/100);
  // broadcast voltage over Bluetooth
  VoltageChar.writeValue(voltage);

  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  // if (central) {
    if (1){
    // Serial.print("Connected to central: ");
    // print the central's BT address:
    // Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    // while (central.connected()) {
    while(1){
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        bool val = updateVoltage();
        if (val==false) {
          Serial.println("Cover Gone");
          exit(0);
        }
      }
    }

    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.print("Disconnected from central: ");
    // Serial.println(central.address());
  }
}

float computeAvg(int *myArrg)
{  //compute the average of the array of points
  int accumulator = 0;
  float val;
  for (int j = 0; j < AVG_WINDOW; j++) {
  accumulator = accumulator + myArrg[j];
  }
  val = (float)accumulator / AVG_WINDOW;
  return val;
  
  }

  void collectPoints(int sampleInterval, int pinNumP, int pinNumN, int *myArrg)
  { // collect N points from 2 analog ins and take the difference.
    int i = 0;
    while(i < AVG_WINDOW) {
      if(pinNumN == 'G'){
        myArrg[i] = analogRead(pinNumP);
      }
      else{
        myArrg[i] = analogRead(pinNumP) - analogRead(pinNumN);
      }
      
      //i = (i + 1) % AVG_WINDOW;
      ++i;
      delay(sampleInterval);
  }
    }

bool updateVoltage() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */

  collectPoints(sampleInterval, voltInputPinP,voltInputPinN, ADCRaw);
  myAverage = computeAvg(ADCRaw);
  oldVoltage = voltage;
  voltage = myAverage * ADCconversion * 100;
  if (voltage != oldVoltage) {      // if the battery level has changed

    VoltageChar.writeValue(voltage);  // and update the battery level characteristic
    if (abs(oldVoltage-voltage)>4) {
      return false;
    }
  }
  Serial.print("Voltage: ");
  Serial.println(voltage);
  Serial.print("Old volt: ");
  Serial.println(oldVoltage);

  return true;
}