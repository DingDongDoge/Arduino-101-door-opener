/*
 * Author: Martin Woolley
 *
 * Version History
 * V1.0: 
 * First version
 *
 *
 */
 
#include <CurieBLE.h>
#include <SoftwareSerial.h>
int link_loss_alert_level = 0;
int immediate_alert_level = 0;
int ledPin1 = 3;
int ledPin2 = 4;
int ledPin3 = 5;
int speakerPin = 9;
int volume[3] = { 0, 30, 90 };
int alert_may_be_required = 0;
int buzzer_alert_level = 0;
#define lcdTxPin = 10;
SoftwareSerial LCD = SoftwareSerial(0, lcdTxPin);
const int LCDdelay=10;
char buf[16];
#define SOUND_BUZZER_FOR_3_SECONDS 3
#define SOUND_BUZZER_FOR_90_SECONDS 90
#define BUZZER_OFF 0
#define ONE_SECOND 1000
#define HALF_SECOND 500
int alert_counter = BUZZER_OFF;

// BLE objects
BLEPeripheral blePeripheral;


// GAP properties
char device_name[] = "BDSK";

// Characteristic Properties
unsigned char LinkLoss_AlertLevel_props = BLERead | BLEWrite | 0;
unsigned char ImmediateAlert_AlertLevel_props = BLEWriteWithoutResponse | 0;
unsigned char TxPower_TxPowerLevel_props = BLERead | 0;
unsigned char ProximityMonitoring_ClientProximity_props = BLEWriteWithoutResponse | 0;

// Services and Characteristics
BLEService LinkLoss("1803");

BLECharacteristic LinkLoss_AlertLevel("2A06", LinkLoss_AlertLevel_props, 1);

BLEService ImmediateAlert("1802");

BLECharacteristic ImmediateAlert_AlertLevel("2A06", ImmediateAlert_AlertLevel_props, 1);

BLEService TxPower("1804");

BLECharacteristic TxPower_TxPowerLevel("2A07", TxPower_TxPowerLevel_props, 1);

BLEService ProximityMonitoring("3E099910293F11E493BDAFD0FE6D1DFD");

BLECharacteristic ProximityMonitoring_ClientProximity("3E099911293F11E493BDAFD0FE6D1DFD", ProximityMonitoring_ClientProximity_props, 2);

void lcdPosition(int row, int col) {
  LCD.write(0xFE); //command flag
  LCD.write((col + row*16 + 32)); //position
  delay(LCDdelay);
}
void lcdText(String text) {
  clearLCD();
  LCD.print(text);
}
void clearLCD(){
  LCD.write(0xFE); //command flag
  LCD.write(0x01); //clear command.
  delay(LCDdelay);
}
void backlightOn() { //turns on the backlight
  LCD.write(0x7C); //command flag for backlight stuff
  LCD.write(157); //light level.
  delay(LCDdelay);
}
void backlightOff(){ //turns off the backlight
  LCD.write(0x7C); //command flag for backlight stuff
  LCD.write(128); //light level for off.
  delay(LCDdelay);
}
void serCommand(){ //a general function to call the command flag for issuing all other commands
  LCD.write(0xFE);
}

void flash(unsigned char led,uint16_t delayms, unsigned char times){
  for (int i=0;i<times;i++) {
    digitalWrite(led, HIGH);
    delay(delayms);
    digitalWrite(led, LOW);
    delay(delayms);
  }
}
void beepAndFlashAll(uint16_t delayms){
  Serial.println(F("BEEP + FLASH ALL"));
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  analogWrite(speakerPin, volume[buzzer_alert_level]);
  delay(delayms);
  analogWrite(speakerPin, volume[0]);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  delay(delayms);
}
void allLedsOff() {
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
}
// determine the pin to write to to control the colour of LED related to an alert level value
int getPinNumber(uint8_t level) {
  switch (level) {
    case 0: return ledPin1;
    case 1: return ledPin2;
    case 2: return ledPin3;
    default: return ledPin1;
  }
}

void alertControl(void) {
  if (alert_counter > BUZZER_OFF && alert_counter-- > BUZZER_OFF) {
    beepAndFlashAll(HALF_SECOND);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("setup()");

// set advertising packet content
  blePeripheral.setLocalName(device_name);
  
  blePeripheral.setAdvertisedServiceUuid("1803");

// add services and characteristics
  blePeripheral.addAttribute(LinkLoss);
  blePeripheral.addAttribute(LinkLoss_AlertLevel);
  blePeripheral.addAttribute(ImmediateAlert);
  blePeripheral.addAttribute(ImmediateAlert_AlertLevel);
  blePeripheral.addAttribute(TxPower);
  blePeripheral.addAttribute(TxPower_TxPowerLevel);
  blePeripheral.addAttribute(ProximityMonitoring);
  blePeripheral.addAttribute(ProximityMonitoring_ClientProximity);
  Serial.println("attribute table constructed");
 
// begin advertising
  blePeripheral.begin();
  Serial.println("advertising");
  Serial.println("Bluetooth Developer Starter Kit");

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  allLedsOff();
  LCD.begin(9600);
  backlightOn() ;
  clearLCD();
  lcdPosition(0,0);
  lcdText("Ready...");
}

void loop() {
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    lcdText("Connected");
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {

        if (LinkLoss_AlertLevel.written()) {
        // application logic for handling WRITE or WRITE_WITHOUT_RESPONSE on characteristic Link Loss Alert Level goes here
         Serial.println("LinkLoss_AlertLevel.written()");
         link_loss_alert_level = LinkLoss_AlertLevel.value()[0];
         int ledpin = getPinNumber(link_loss_alert_level);
         flash(ledpin,250,4);

        }
        if (ImmediateAlert_AlertLevel.written()) {
        // application logic for handling WRITE or WRITE_WITHOUT_RESPONSE on characteristic Immediate Alert Alert Level goes here
         Serial.println("ImmediateAlert_AlertLevel.written()");
         immediate_alert_level = ImmediateAlert_AlertLevel.value()[0];
         buzzer_alert_level = immediate_alert_level;
         // flash more times for higher alert levels
         alert_counter = SOUND_BUZZER_FOR_3_SECONDS + immediate_alert_level;

        }
        if (ProximityMonitoring_ClientProximity.written()) {
        // application logic for handling WRITE or WRITE_WITHOUT_RESPONSE on characteristic Proximity Monitoring Client Proximity goes here
         Serial.println("ProximityMonitoring_ClientProximity.written()");
         int proximity_band = ProximityMonitoring_ClientProximity.value()[0];
         int client_rssi = ProximityMonitoring_ClientProximity.value()[1];
         client_rssi = (256 - (int) client_rssi) * -1;
         allLedsOff();
         if (proximity_band == 0) {
           // means the user has turned off proximity sharing
           // so we just want to switch off the LEDs
           return;
           }
         int ledpin = getPinNumber(proximity_band - 1);
         digitalWrite(ledpin, HIGH);
         char buf[16];
         sprintf(buf, "Client RSSI: %d", client_rssi);
         lcdText(buf);
         }

        alertControl();
        
    }

    lcdText("Disconnected");
    
    allLedsOff();
    
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
