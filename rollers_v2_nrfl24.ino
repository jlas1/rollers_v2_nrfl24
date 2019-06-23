/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_RFM69
//#define MY_IS_RFM69HWw
//#define MY_REPEATER_NODE
#define MY_NODE_ID AUTO
//#define MY_NODE_ID 1

#include <SPI.h>
#include <MySensors.h> 
#include <DallasTemperature.h> 
#include <OneWire.h>
#include <avr/wdt.h>

#define ROLLER_ID 0

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//Sketch information
#define SKETCH_INFO       "Roller Controller"
#define SKETCH_VERSION    "1.1"
#define ROLLER_ID_INFO    "Roller Motor and Relay"
#define ROLLER_ID 0
#define BUTTON_ID_INFO    "Extra relay"
#define BUTTON_ID 2
#define TEMP_ID_INFO      "Case Temperature"
#define TEMP_ID 1

//Auto-reset
#define REBOOT_PIN A2
#define MESSAGES_FAILED_REBOOT 20
#define CYCLES_REBOOT 512

//Configurable ACK Timeout
#define ACK_TIMEOUT 1000

//BAUD RATE
#define BAUD_RATE 115200

//Cycles in between updates
#define SECONDS_PER_CICLE 300
#define CYCLES_PER_PRESENT 288

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 20
#define LOW_PRIORITY_RETRIES 5

//DS18B20 configuration
#define ONE_WIRE_BUS 7 // Pin where dallas sensor is connected 
#define ONE_WIRE_VCC 8 // Pin where the one wire power is controlled
#define TEMP_RESOLUTION 11 //Temperature resolution

//Roller States
#define UP 100
#define DOWN 0
#define STOP -1

//Roller Relay PINs
#define PIN_ROLLER_POWER A0
#define PIN_ROLLER_DIRECTION A1
#define PIN_RELAY_ACTIVE 4
#define PIN_RELAY_INACTIVE 3

//Roller Relay States
#define ROLLER_POWERON HIGH
#define ROLLER_POWEROFF LOW
#define ROLLER_DOWN HIGH
#define ROLLER_UP LOW
#define ROLLER_DIROFF LOW
#define RELAY_ACTIVE 1
#define RELAY_INACTIVE 2
#define RELAY_WAIT 3
#define RELAY_INIT 4

//Roller maxtravel time
#define ROLLER_TIMEOUT 60000
#define ROLLER_DELAY 500

float temperature=-127;
unsigned long sensorreadTime = 900;
unsigned int messagesFailed=0;
volatile int isACKed = false, NewRollerPosition=STOP, RollerPosition=STOP, NewRelayStatus=RELAY_INIT;
unsigned int i;
unsigned int topresent = CYCLES_PER_PRESENT;

boolean ack = true;
boolean metric, tempread, working = false;

unsigned long sleep_time = (SECONDS_PER_CICLE); // Sleep time between reads (in milliseconds)
unsigned long cicles=0, currentMillis,loopMillis, millisRELAY, millisUP, millisDOWN, millisSTOP,millisZero=0UL;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

//Mysensors Messages initialization
MyMessage msgT(TEMP_ID,V_TEMP);

void before() {
  //disable watchdog timer
  MCUSR = 0;

  //setup relay pins
  digitalWrite(PIN_ROLLER_POWER, ROLLER_POWEROFF);
  pinMode(PIN_ROLLER_POWER,OUTPUT);
  digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DIROFF);
  pinMode(PIN_ROLLER_DIRECTION,OUTPUT);

  digitalWrite(PIN_RELAY_ACTIVE, LOW);
  pinMode(PIN_RELAY_ACTIVE,OUTPUT);
  digitalWrite(PIN_RELAY_INACTIVE, LOW);
  pinMode(PIN_RELAY_INACTIVE,OUTPUT);

  //waking temperature sensor
  pinMode(ONE_WIRE_VCC,OUTPUT);
  Serial.println("waking sensor");
  digitalWrite (ONE_WIRE_VCC, HIGH);

  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));

  //blink LED on power up
  pinMode(13,OUTPUT);
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }

  //initialize resolution
  if (sensors.getResolution() != TEMP_RESOLUTION) {
    Serial.println("New temperature resolution configured");
    sensors.setResolution(TEMP_RESOLUTION); 
  } else {
    Serial.println("Temperature resolution not changed");
  }

}

void setup()  
{
  Serial.println("");
  Serial.println(F("Listening for Commands"));

  //Request relay status
  Serial.println(F("Requesting Relay status"));  
  request(BUTTON_ID, V_STATUS, 0);
  wait (500);
  
//  metric = getConfig().isMetric;
  //activate watchdog timer
  //wdt_enable(WDTO_8S);
  Serial.println(F("Starting Loop"));
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  //check if it should present itself
  if (millis() >= 86400000) {
    Serial.println(F("daily Reboot reached"));
    asm volatile ( "jmp 0");
    wait(100);
  }
  cicles++;
  Serial.print(F("Waiting in cicle "));
  //requesting temperature
  sensors.requestTemperatures();
  tempread = false;
  //displays the current cicle count
  Serial.println(cicles);

  //stays in the state machine until end off current loop time
  loopMillis = hwMillis();
  while (hwMillis() - loopMillis < sleep_time*1000) {
    currentMillis = hwMillis(); // assure consistent time comparitions
    //check if the temperature conversion is ended
    if ((long(currentMillis - loopMillis) >= long (sensorreadTime)) and (tempread == false)) {
      temperature = ((metric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)));
      //print Temperature
      Serial.print("Temperature: ");
      Serial.println(temperature,3);
      tempread = true;
      // Only send data if no error
      if (temperature != -127.00 && temperature != 85.00) {
        resend(msgT.set(temperature,3), LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
      }
    }

    switch (NewRelayStatus) {
      wdt_reset();
      case RELAY_ACTIVE:
        Serial.println(F("Activating Relay"));
        digitalWrite(PIN_RELAY_ACTIVE, HIGH);
        millisRELAY = currentMillis + 2000;
        NewRelayStatus=RELAY_WAIT;
        break;
      case RELAY_INACTIVE:
        Serial.println(F("Inactivating Relay"));
        digitalWrite(PIN_RELAY_INACTIVE, HIGH);
        millisRELAY = currentMillis + 2000;
        NewRelayStatus=RELAY_WAIT;
        break;
      case RELAY_WAIT:
        if (long(currentMillis - millisRELAY) > long (millisZero)) { //Stop the Relay
          Serial.println(F("Turning Relay off"));
          digitalWrite(PIN_RELAY_ACTIVE, LOW);
          digitalWrite(PIN_RELAY_INACTIVE, LOW);
          NewRelayStatus=RELAY_INIT;
        }
        break;
      case RELAY_INIT:
        break;
    }
    
    if (NewRollerPosition == RollerPosition) {
      _process();
      wdt_reset();

      //control the relays according to the timers
      if (working == true) {
        if (long(currentMillis - millisSTOP) > long (millisZero)) { //Stop the ROLLER
          Serial.println(F("Stopping"));
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWEROFF);
          wait(100);
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DIROFF);
          working = false;
          RollerPosition=NewRollerPosition=STOP;
          
        } else if (long(currentMillis - millisUP) >  long (millisZero)) { //Go UP
          Serial.println(F("Going up"));
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_UP);
          wait(100);
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWERON);
          millisUP = currentMillis + ROLLER_TIMEOUT + 1;
          
        } else if (long(currentMillis - millisDOWN) > long (millisZero)) { //Go DOWN
          Serial.println(F("Going down"));
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DOWN);
          wait(100);
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWERON);
          millisDOWN = currentMillis + ROLLER_TIMEOUT + 1;
        }
      }  
    } else {     
      RollerPosition=NewRollerPosition;
      switch (NewRollerPosition) {
        case UP: // New position is UP
          Serial.println(F("Preparing UP command"));
          //stop the roller
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWEROFF);
          wait(100);
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DIROFF);
          //reset time variables
          millisUP = currentMillis + ROLLER_DELAY;     //setup up time
          millisSTOP = currentMillis + ROLLER_TIMEOUT; //setup stop time
          millisDOWN = currentMillis + ROLLER_TIMEOUT + 1;              //cancel down time
          working = true;
          break;
          
        case DOWN: // New position is DOWN
          Serial.println(F("Preparing DOWN command"));        
          //stop the roller
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWEROFF);
          wait(100);
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DIROFF);
          //reset time variables
          millisDOWN = currentMillis + ROLLER_DELAY;   //setup down time
          millisSTOP = currentMillis + ROLLER_TIMEOUT; //setup stop time
          millisUP = currentMillis + ROLLER_TIMEOUT + 1;                //cancel up time
          working = true;
          break;
          
        case STOP: // New position is STOPPED
          Serial.println(F("Executing STOP command"));
          //stop the roller
          digitalWrite(PIN_ROLLER_POWER, ROLLER_POWEROFF);
          wait(100);
          digitalWrite(PIN_ROLLER_DIRECTION, ROLLER_DIROFF);
          //stop time variables processing
          working = false;
          break;
      }
    }  
  }
} 

void gwPresent () {
  //present at beggining and every day
  if (topresent < CYCLES_PER_PRESENT) {
    topresent++;
    return;
  }
  Serial.println(F("Presenting"));
  //reset count;
  topresent = 0;

  sendSketchInfo(SKETCH_INFO, SKETCH_VERSION);
  present(ROLLER_ID, S_COVER, ROLLER_ID_INFO);
  present(BUTTON_ID, S_BINARY, BUTTON_ID_INFO);
  present(TEMP_ID, S_TEMP, TEMP_ID_INFO);
}

void resend(MyMessage &msg, int repeats, int timeout)
{
  int repeat = 0;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    send(msg,true);

    if (waitACK(timeout)) {
      sendOK = true;
      messagesFailed = 0;
    } else {
      sendOK = false;
      Serial.print("Retry ");
      Serial.print(repeat);
      Serial.print(" Failed ");
      Serial.println(messagesFailed);
      repeatdelay += 500;
      wdsleep(repeatdelay);
    }
    repeat++; 
  }
  if (sendOK == false) {
    if (messagesFailed > MESSAGES_FAILED_REBOOT) {
      //wdt_enable(WDTO_15MS);
      asm volatile ( "jmp 0");
      wait(100);
    }
    messagesFailed++;
  }
}

boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == true) {
      isACKed = false;
      Serial.print(F("Reply "));
      Serial.print(timeout);
      Serial.print(" ");
      Serial.println((millis() - startTime));
      return true;
    }
  }
  return false;
}

void receive (const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  if (message.isAck()) {
    Serial.println(F("This is an ack from gateway."));
    isACKed = true;
  } else if (message.type == V_UP && message.sensor==ROLLER_ID) {
    Serial.println(F("UP Command Received"));
    if (RollerPosition == DOWN) {
      NewRollerPosition = STOP;
      Serial.println(F("acted as a STOP Command"));
    } else {
      NewRollerPosition = UP;
    }
  } else if (message.type == V_DOWN && message.sensor==ROLLER_ID) {
    Serial.println(F("DOWN Command Received"));
    if (RollerPosition == UP) {
      NewRollerPosition = STOP;
      Serial.println(F("acted as a STOP Command"));
    } else {
      NewRollerPosition = DOWN;
    }
  } else if (message.type == V_STOP && message.sensor==ROLLER_ID) {
    Serial.println(F("STOP Command Received"));
    NewRollerPosition = STOP;
  } else if (message.type == V_STATUS && message.sensor==BUTTON_ID) {
    Serial.println(F("Relay Command Received"));
    if (message.getBool() == true) {
      Serial.println(F("Relay Active"));
      NewRelayStatus = RELAY_ACTIVE;
    } else {
      Serial.println(F("Relay Inactive"));
      NewRelayStatus = RELAY_INACTIVE;
    }
  } else {
    Serial.print(F("Incoming change for sensor:"));
    Serial.println(message.sensor);
  }
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
     _process();
    wdt_reset();
  }
    wdt_reset();
  }
  #else
    sleep(ms);
  #endif
}

