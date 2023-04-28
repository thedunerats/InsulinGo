/*
 * Copyright (c) 2016 RedBear
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

// A modified version of BLE_HRM.

// Changes from V1:
// took out LED output change to verify edge detaction
// added a laser output that turns on only when paired with a force
// Chanegs from V5:
// Added custom UUIDs for service and characteristics
// renamed device to InsulinGo


#include <BLE_API.h>
#include <Time.h>
#include <TimeLib.h>

#define DEVICE_NAME       "InsulinGo"  //modify this is a future iteration? (this renames the device)
#define INIT_TIME 1497129600 //Initialize time in UNIX time. UPDATE THIS BEFORE programming.
#define DEVICE_UUID 0x4b56a900e7a745579d259d76ce8a1d81
#define DOSE_UUID 0x29a4a1f095b14dd6a23bdf67691b2c58
#define TIME_UUID 0x133d99d81fa346e9b334485c547be2b5


BLE                       ble;
Ticker                    ticker_task1;

int led = 13;

const int readerPin = 14;    // the pin that the photoresistor is attached to
const int ledPin = 3;       // the pin that the LED is attached to
const int forcePin = A0;      // the pin that receives the force input

//const int PR_THRESHOLD = 210; 
const int HIGH_THRESHOLD = 192;
const int LOW_THRESHOLD = 188;

//const int DEADZONE = ;

int barcodeCounter = 0;   // counter for the dose
     // initialize reading for the force sensor
      

uint8_t dose[500];
time_t doseTime[500];
size_t dosePosition = 0;

static uint8_t bpm[1]         = {0};
static uint32_t timeA[1] = {0};

static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

// Create characteristic and service
GattCharacteristic   hrmRate(DOSE_UUID, bpm, sizeof(bpm), sizeof(bpm), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic   hrmLocation(TIME_UUID, (uint8_t *)timeA, sizeof(timeA), sizeof(timeA), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic   *hrmChars[] = {&hrmRate, &hrmLocation, };
GattService          hrmService(DEVICE_UUID, hrmChars, sizeof(hrmChars) / sizeof(GattCharacteristic *));


void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected!");
  Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

void readDose(){
  int barcodeState = 0;         // current state of the barcode, starts off at zero
  int lastButtonState = 0;      // previous state of the barcode
  int photocellReading;         // analog reading of the photocell
  boolean forcePressed = false; // indicates whether or not the sensor is recording, default = false
                                //might take this out, data type might be inappropriate
  boolean doseTaken = false;    // boolean that controls the output
  int k = 0;                    // counter for edge detector, prevents program from reading 1 when 0 state changes occur in loop
  int fsrReading;

  digitalWrite(ledPin, HIGH); // turns on the laser
  fsrReading = analogRead(forcePin);  //initialize a variable to act as a gauge for the force-sensing resistor
   // check to make sure force is always present
   if (fsrReading > 256) {      //consider messing around with the threshold. this might change something, was 192 before, might need to raise the threshold
    forcePressed = true;
    delay(5); // delay to observe a change, might take this out later
      while (forcePressed == true) {
        if (photocellReading < (LOW_THRESHOLD)) {      //consider messing around with the threshold. this might change something (1012?)
          barcodeState = HIGH;   // acquire a digital output, case 1
        }
        else if(photocellReading > (HIGH_THRESHOLD)) {
          barcodeState = LOW; //digital output, case 2
        }    
        
        photocellReading = analogRead(readerPin); // read photoresistor

          // compare the buttonState to its previous state, in turn incrementing the dose counter
        if (barcodeState != lastButtonState) {
          // if the state has changed, increment the counter
          if (k > 0){
            barcodeCounter++;
            // Delay a little bit to avoid bouncing
            delay(80);
            }
            // save the current state as the last state,
            // for next time through the loop
          k++;    // counter to prevent a plus 1 error
          lastButtonState = barcodeState;
         }
          // way to exit loop
          fsrReading = analogRead(forcePin);  //initialize a variable to act as a gauge for the force-sensing resistor, experimental for the time being (SUCCESS!)
              // you actually need to check this in the loop. it wont work without it.
          if (fsrReading <= 256) {      //consider messing around with the threshold. this might change something, was 192 before
                forcePressed = false;
                digitalWrite(ledPin, LOW);
                return;
          }
        }        
     }
     else { 
      digitalWrite(ledPin, LOW); // turn off the laser
      forcePressed = false; // exit the measuring sequence
     }
 
}

void periodicCallback() { // data retrieval function
  if (analogRead(forcePin) > 256) {
    readDose();
    dose[dosePosition] = barcodeCounter;
    doseTime[dosePosition] = now();
    dosePosition++;
    barcodeCounter = 0;   
    if (ble.getGapState().connected && dosePosition > 0) {
      for (int i = 0; i < dosePosition; ++i){
        bpm[0] = dose[i];
        timeA[0] = doseTime[i];
        ble.updateCharacteristicValue(hrmRate.getValueAttribute().getHandle(), bpm, sizeof(bpm));
        delay(50);
        ble.updateCharacteristicValue(hrmLocation.getValueAttribute().getHandle(), (uint8_t *)timeA, sizeof(timeA));
        delay(50);
      }
      dosePosition = 0;      
    } 
  }
}


void setup() {
  // put your setup code here, to run once
  //Initialize Time
  setTime(INIT_TIME);  
  // initialize the button pin as a input:
  pinMode(readerPin, INPUT);  
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  // intialize force pin as an input
  pinMode(forcePin, INPUT);
  Serial.begin(9600);
  Serial.println("Nordic_HRM Demo ");
  // Init timer task
  ticker_task1.attach(periodicCallback, .1);
  // Init ble
  ble.init();
  ble.onDisconnection(disconnectionCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t*)uuid16_list, sizeof(uuid16_list));
  ble.accumulateAdvertisingPayload(GapAdvertisingData::UNKNOWN);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    // add service
  ble.addService(hrmService);
  // set device name
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();  
}

void loop() {
  // put your main code here, to run repeatedly:
  ble.waitForEvent();
}
