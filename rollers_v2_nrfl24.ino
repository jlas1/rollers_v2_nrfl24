/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_IS_RFM69HWw
//#define MY_REPEATER_NODE
#define MY_NODE_ID AUTO
//#define MY_NODE_ID 5
#define MY_RF24_PA_LEVEL RF24_PA_MAX
#define MY_RF24_DATARATE RF24_250KBPS

#include <SPI.h>
#include <MySensors.h> 
#include <DallasTemperature.h> 
#include <OneWire.h>
#include <avr/wdt.h>

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//pin definitions
#define LED_PIN A3
#define CSENSE_PIN A2
#define UPDOWN_RELAY_PIN A1
#define POWER_RELAY_PIN A0
#define DS18B20_VCC_PIN 8 // Pin where the one wire power is controlled
#define DS18B20_BUS_PIN 7 // Pin where dallas sensor is connected 
#define BUTTON_UP_PIN 4
#define BUTTON_DOWN_PIN 3

//Mysensors Child ID
#define TEMP_ID 1       //Temperature
#define ROLLER_ID 2     //Rollershutter
#define BUTTON_UP_ID 3  //Up button
#define BUTTON_DOWN_ID 4 //Down button
#define CURRENT_SENSE_ID 5 //ACS712 current sense 

//Sketch information
#define SKETCH_INFO       "Roller Controller"
#define SKETCH_VERSION    "0.1"
#define TEMP_INFO         "Temperature Sensor"
#define BUTTON_UP_INFO    "Button Up Actuator"
#define BUTTON_DOWN_INFO  "Button Down Actuator"

//DS18B20 configuration
#define DELTA_TEMP 0.2 // define temperature delta to report to controller (a value of 0 sends everytime)
#define TEMP_RESOLUTION 11 //Temperature resolution

//Configurable ACK Timeout
#define ACK_TIMEOUT 1000

//BAUD RATE
#define BAUD_RATE 115200

//Cycles in between updates
#define SECONDS_PER_CICLE 30
#define CYCLES_PER_UPDATE 10
#define CYCLES_PER_PRESENT 2880

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 20
#define LOW_PRIORITY_RETRIES 5


//Variables

float lastTemperature=-127,temperature=-127,deltatemp;
int messagesFailed=0;
volatile int isACKed = FALSE;
unsigned int nosend = CYCLES_PER_UPDATE, i;
unsigned int topresent = CYCLES_PER_PRESENT;
boolean ack = true;
boolean metric, temperatureError = false; 

unsigned long tempwakeupTime = 1000;
unsigned long sensorreadTime = 900;
unsigned long sleep_time = (SECONDS_PER_CICLE - tempwakeupTime/1000 - sensorreadTime*2.0/1000); // Sleep time between reads (in milliseconds)
unsigned long cicles=0;

OneWire oneWire(DS18B20_BUS_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

//Mysensors Messages initialization
MyMessage msgTemperature(TEMP_ID,V_TEMP);                 // Temperature
MyMessage msgUp(ROLLER_ID,V_UP);                          // Up 
MyMessage msgDown(ROLLER_ID,V_DOWN);                      // Down 
MyMessage msgStop(ROLLER_ID,V_STOP);                      // Stop 
MyMessage msgPosition(ROLLER_ID,V_PERCENTAGE);            // % shutter 
MyMessage msgCurrentSense(CURRENT_SENSE_ID,V_WATT);       // Current sensor

//Debounce
Bounce debounceUp    = Bounce(); 
Bounce debounceDown  = Bounce();

void setup()  
{
  //disable watchdog timer
  MCUSR = 0;
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));
 
  //Setup pin modes for proper operation
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(UPDOWN_RELAY_PIN,LOW);
  pinMode(UPDOWN_RELAY_PIN,OUTPUT);
  digitalWrite(POWER_RELAY_PIN,LOW);
  pinMode(POWER_RELAY_PIN,OUTPUT);
  pinMode(DS18B20_VCC_PIN,OUTPUT);  
  pinMode(DS18B20_BUS_PIN,INPUT);
  pinMode(BUTTON_UP_PIN,INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN,INPUT_PULLUP);

  //debouncer Setup
  debounceUp.attach(BUTTON_UP_PIN);
  debounceUp.interval(5);
  debounceDown.attach(BUTTON_DOWN_PIN);
  debounceDown.interval(5);

  
  //blink LED on power up
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (LED_PIN, HIGH);
    delay(LED_DELAY);
    digitalWrite (LED_PIN, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }
  Serial.println("");

  start_sensors();
  
  analogReference(DEFAULT);
  wdsleep(100);  
  metric = getConfig().isMetric;
  //activate watchdog timer
  #if defined(MY_REPEATER_FEATURE)
    wdt_enable(WDTO_8S);
  #endif
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  Serial.println(F("Starting Loop"));
  //check if it should present itself
  gwPresent ();
  readTemp ();
  sendValues();
  if (cicles >= CYCLES_REBOOT) {
    Serial.println(F("Reboot cicles reached"));
    wdt_enable(WDTO_15MS);
    wait(100);
  }
  cicles++;
  Serial.print(F("Waiting in cicle "));
  //displays the current cicle count
  Serial.println(cicles);
  wdsleep(sleep_time*1000);
} 

void start_sensors () {
  //Wake the temperature sensor
  Serial.println(F("waking sensor"));
  digitalWrite (DS18B20_VCC_PIN, HIGH);
  delay (tempwakeupTime);
  // Startup up the OneWire library
  sensors.begin();
  sensors.setWaitForConversion(false);

  //initialize temperature resolution
  if (sensors.getResolution() != TEMP_RESOLUTION) {
    Serial.println(F("New temperature resolution configured"));
    sensors.setResolution(TEMP_RESOLUTION);
  } else {
    Serial.println(F("Temperature resolution not changed"));
  }
  Serial.println(F("sleeping sensor"));
  digitalWrite (DS18B20_VCC_PIN, LOW);
}


void readTemp () {
  // power up sensor
  Serial.println(F("Reading Temperature"));
  Serial.println(F("waking sensor"));
  digitalWrite (DS18B20_VCC_PIN, HIGH);
  wdsleep(tempwakeupTime);

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();
  wdsleep(sensorreadTime);
  // Fetch
  temperature = ((metric ? sensors.getTempCByIndex(0) : sensors.getTempFByIndex(0)));
  //Fetch again
  sensors.requestTemperatures();
  wdsleep(sensorreadTime);
  temperature = (temperature + (metric ? sensors.getTempCByIndex(0) : sensors.getTempFByIndex(0))) / 2.0;
  // disable sensor
  Serial.println(F("sleeping sensor"));
  digitalWrite (DS18B20_VCC_PIN, LOW);
  //print Temperature
  Serial.print(F("Metric:"));
  Serial.print(metric);
  Serial.print(F(" temperature: "));
  Serial.println(temperature, 3);

  // Only send data if no error
  if (temperature != -127.00 && temperature != 85.00) {
    deltatemp = temperature - lastTemperature;
    if (abs(deltatemp) < DELTA_TEMP) {
      //debug message
      Serial.println(F("Aproximatelly the same Temperature, skipping send"));
    } else {
      lastTemperature = temperature;
      nosend = CYCLES_PER_UPDATE;
      temperatureError = false;
    }
  } else {
    Serial.println(F("Error reading temperature"));
    temperatureError = true;
    start_sensors();
  }
}

// Reads Battery voltage while averaging the last BATTERY_READS values
void readBattery () {
  
  Serial.println(F("Reading Battery Voltage"));
  //Substract the last reading
  BattarrayTotal = BattarrayTotal - Battarray[Battindex];
  //primes the analog converter
  analogRead(BATTERY_SENSE);
  //Reads battery voltage
  wdsleep(10);
  Battarray[Battindex] = analogRead(BATTERY_SENSE);
  //adds the current reading
  BattarrayTotal = BattarrayTotal + Battarray[Battindex];
  Battindex ++;
  if (Battindex >= BATTERY_READS) {
    Battindex = 0;
  }
  BattValue = BattarrayTotal / BATTERY_READS;
  Serial.print(F("analog "));
  Serial.println(BattValue);

  Vbatt = (float)BattValue * BATT_CALC;
  Batt = (Vbatt - VMIN) * 100.0 / (VDELTA);
  if (Vbatt > VMAX) {
    Batt = 100;
  } else if (Vbatt < VMIN) {
    Batt = 0;
  }

  deltavbatt = Vbatt - (BattValue * BATT_CALC);
  if (abs(deltavbatt) > DELTA_VBATT) {
    //force sending values
    nosend = CYCLES_PER_UPDATE;
  }

  //print battery status
  Serial.print(F("battery: "));
  Serial.print(Batt);
  Serial.print(F(", "));
  Serial.println(Vbatt, 3);
}


// Reads Battery voltage while averaging the last BATTERY_READS values
void readSolar () {
  Serial.println(F("Reading Solar Voltage"));

  analogRead(SOLAR_SENSE);
  //Reads battery voltage
  wdsleep(10);
  BattValue = analogRead(SOLAR_SENSE);
  sleep(10);
  //adds the current reading
  BattValue = (BattValue + analogRead(SOLAR_SENSE)) / 2;

  Serial.print(F("Solar analog "));
  Serial.println(BattValue);

  Sbatt = (float)BattValue * BATT_CALC;

  //print battery status
  Serial.print(F("Solar voltage: "));
  Serial.println(Sbatt, 3);
}

void sendValues () {
  //only sends values if they have changed or if it didn't send for 120 cycles (1 hour)
  if (nosend < CYCLES_PER_UPDATE) {
    nosend++;
    return;
  }
  //reset count;
  nosend = 1;

  //print debug message
  Serial.println(F("Sending Values"));
  //send values
  if (temperatureError == false) {
    Serial.print(F("Temperature "));
    Serial.println(temperature,3);
    resend(msgT.set(temperature, 3),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  }
  Serial.print(F("Battery Voltage "));
  Serial.println(Vbatt,3);
  resend(msgB.set(Vbatt, 3),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  Serial.print(F("Solar Voltage "));
  Serial.println(Sbatt,3);
  resend(msgS.set(Sbatt, 3),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  sendBatteryLevel(Batt, ack);
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
  present(TEMP_ID, S_TEMP, TEMP_ID_INFO);
  present(BVOLT_ID, S_MULTIMETER, BVOLT_ID_INFO);
  present(SVOLT_ID, S_MULTIMETER, SVOLT_ID_INFO);
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
      asm volatile ( "jmp 0");
    }
    messagesFailed++;
  }
}


boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == TRUE) {
      isACKed = FALSE;
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
    Serial.println(F("RCVD: ACK"));
    isACKed = TRUE;
  } else {
    // Message received : Open shutters
    if (message.type == V_UP && message.sensor==ROLLER_ID) {
      Serial.println(F("RCVD: cmd Up")); 
      RollerPosition(100);
    }  
    // Message received : Down shutters
    if (message.type == V_DOWN && message.sensor==ROLLER_ID) {
      Serial.println(F("RCVD: cmd Down")); 
      RollerPosition(0);
    }  
    // Message received : Open shutters
    if (message.type == V_UP && message.sensor==ROLLER_ID) {
      Serial.println(F("RCVD: cmd Stop")); 
      RollerStop();
    }         
    // Message received : Set position of Rollershutter 
    if (message.type == V_PERCENTAGE && message.sensor==ROLLER_ID) {
       if (message.getByte() > 100) { 
         setPosition(100)
       } else {
         setPosition(message.getByte());
       }
       Serial.print(F("RCVD: Set position to "));              
       Serial.print(message.getByte());Serial.println(F("%")); 
    }
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
    _process();
    wdt_reset();
  }
  #else
    sleep(ms);
  #endif
}

