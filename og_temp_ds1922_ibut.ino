/*   OneWire Ibutton data logger reader
     by O. Gruenig
     Based on work by Jeff Miller   http://arduinofun.blogspot.com/
     Released under GPLv3 http://www.gnu.org/copyleft/gpl.html
*/

#include <OneWire.h>
#include <SevenSegmentTM1637.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TimeLib.h>
#include <Time.h>

#define ONE_WIRE_PIN D7
#define LED_PIN D6
#define CLK D2   //Display Pin CLK
#define DIO D3   //Display Pin DIO
#define BUTTON_PIN D5
#define TRY_MAX 10 // maximum attempts to connect to wifi, else sleep

byte attached_devices = 0;  // Number of one wire devices
byte ow_device[8];    // Temperature data logger
byte scratchpad[33]; // DS1923 requires required byte through end of the scratch pad to be written anytime a byte is written to scratchpad

const char* ssid = "yourWifiSSID";
const char* password = "yourPWD";
const char* mqtt_server = "raspberrypi";
//const char* mqtt_server = "broker.mqtt-dashboard.com";

// initialise oneWire Bus
OneWire one_wire_bus(ONE_WIRE_PIN);
SevenSegmentTM1637  display(CLK, DIO);
WiFiClient espClient;
PubSubClient client(espClient);

#include "get_time_rtc.h"
#include "datalogger_register.h"
// Datalogger Time Parameters
unsigned long mission_sample_rate = 1;  //sample rate in secs
byte register_data[5];                      // 1 wire device / ibutton register data
float temperature, temp_min;
unsigned long mission_count = 0;                      // MIP total samples: needed to get last data stored into datalogger.
int mip_status = 0;
int osc_status = 0;                         //EOSC = clock running
int button_state = 0, old_button_state = 0;
int device_type = 0;                            // Type of ibutton, currently only 40h=DS1922L allowed
int tlfs = 0;                               // Temperature resolution high or low
byte datalog_poll_delay = 5;                // Delay command to read temp and or RH
tmElements_t tm_start;
time_t t_start, t1, t_min;                 //time in secs since 1970

void setup_wifi() {
  delay(100);
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int try_no = 0; //count attempts to connect
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    try_no++;
    if (try_no >= TRY_MAX) {
      Serial.println("Can't connect to Wifi.");
      display.print("ERR1");
      delay(1000);
      break;
    }
  }
  randomSeed(micros());
  Serial.println("WiFi connected");
}


void SearchOneWireDevices()
{
  byte addr[8];
  attached_devices = 0;
  while (one_wire_bus.search(addr) != 0) {
    for ( byte i = 0; i < 8; i++) { // Read the device ID
      Serial.println(addr[i], HEX);
      if (addr[0] == 0x41) {  // DS1922 Temperature Datalogger Family Device
        ow_device[i] = addr[i];
      }
    }
    attached_devices += 1; // Keep track of number of attached devices
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      attached_devices = 0;        // Sometimes error generates more then 2 devices causing remove iButton message incorrectly
      one_wire_bus.reset_search(); // reset search device queue
      return;
    }
  }
  one_wire_bus.reset_search(); // reset search device queue
  return;
}

float CalcTemperatureCalData(byte* addr, byte tempRL, byte tempRH)  // ibutton address, temperature raw low byte, temperature raw high byte
{
  // Temperature Correction variables from data sheet
  float Tr1 = 60, Offset = 41, Tr2, Tr3, Tc2, Tc3, Err2, Err3, Err1, A, B, C, Tc, Tcorr;
  float tmpCal[8]; // Store Temperature calibration data
  Tc = float((float)tempRH / 2 + (float)tempRL / 512 - Offset);             // Raw temperature uncorrected
  for ( byte i = 0; i < 8; i++) { // need 8 bytes of calibration data
    tmpCal[i] = GetDataLoggerRegister(ow_device, i + 0x40, 0x02);
  }
  // From data sheet, calculate correction factors
  Tr2 = tmpCal[0] / 2 + tmpCal[1] / 512 - Offset;
  Tr3 = tmpCal[4] / 2 + tmpCal[5] / 512 - Offset;
  Tc2 = tmpCal[2] / 2 + tmpCal[3] / 512 - Offset;
  Tc3 = tmpCal[6] / 2 + tmpCal[7] / 512 - Offset;
  Err2 = Tc2 - Tr2;
  Err3 = Tc3 - Tr3;
  Err1 = Err2;
  B = (Tr2 * Tr2 - Tr1 * Tr1) * (Err3 - Err1) / ((Tr2 * Tr2 - Tr1 * Tr1) * (Tr3 - Tr1) + (Tr3 * Tr3 - Tr1 * Tr1) * (Tr1 - Tr2));
  A = B * (Tr1 - Tr2) / (Tr2 * Tr2 - Tr1 * Tr1);
  C = Err1 - A * Tr1 * Tr1 - B * Tr1;
  Tcorr = Tc - (A * Tc * Tc + B * Tc + C);        // Corrected Temperature
  return Tcorr;  // Temperature in deg C
}

void read_reg_all(void) {
  byte ta1 = 0x00;
  byte ta2 = 0x02;
  while (ta1 <= 0x37) {
    register_data[0] = GetDataLoggerRegister(ow_device, ta1, ta2);
    Serial.print("Address: ");
    Serial.print(ta2, HEX);
    Serial.print(":");
    Serial.print(ta1, HEX);
    Serial.print("\t HEX: ");
    Serial.print(register_data[0], HEX);
    Serial.print("\t BIN: ");
    Serial.print(register_data[0], BIN);
    Serial.println();
    delay(0);
    ta1 = ta1 + 1;
  }
}

void read_reg_selected(void) {
  register_data[0] = GetDataLoggerRegister(ow_device, 0x06, 0x02); //0206h sample rate low byte
  mission_sample_rate = register_data[0];
  register_data[0] = GetDataLoggerRegister(ow_device, 0x07, 0x02); //0207h sample rate high byte
  mission_sample_rate = mission_sample_rate + register_data[0] * 256;
  register_data[0] = GetDataLoggerRegister(ow_device, 0x12, 0x02); //0212h RTC control
  if (register_data[0] & 0x02 != 1) { //if EHSS bit not set, then minutes; else seconds
    mission_sample_rate = mission_sample_rate * 60;
  }
  if (register_data[0] & 0x01 == 1) { //EOSC bit set = clock is running
    osc_status = 1;
  } else {
    osc_status = 0;
  }
  Serial.print("Mission_sample_rate in secs =");
  Serial.println(mission_sample_rate);
  Serial.print("osc status =");
  Serial.println(osc_status);
  register_data[0] = GetDataLoggerRegister(ow_device, 0x13, 0x02); //0213h mission control
  if ((register_data[0] & 0x04) > 0 ) { // TLFS bit
    tlfs = 1; //high resolution temp 16 bit
  } else {
    tlfs = 0; //low resolution temp 8 bit, currently not allowed
  }
  Serial.print("temp logging format TLFS =");
  Serial.println(tlfs);
  register_data[0] = GetDataLoggerRegister(ow_device, 0x15, 0x02); // 0215h mip status
  if ((register_data[0] & 0x02) > 0 ) { // MIP bit
    mip_status = 1;
  } else {
    mip_status = 0;
  }
  Serial.print("MIP status =");
  Serial.println(mip_status);
  for (byte i = 0; i < 3; i++) { // (220h, 221h, 222h) - Mission sample counter
    register_data[i + 1] = GetDataLoggerRegister(ow_device, 0x20 + i, 0x02);
  }
  mission_count = register_data[3];
  for (byte i = 2; i > 0; i--) { // Bit shift 3 bytes of sample count data into one int
    mission_count = mission_count << 8 | register_data[i];
  }
  Serial.print("Mission sample count is ");
  Serial.println( mission_count);
  register_data[0] = GetDataLoggerRegister(ow_device, 0x26, 0x02); // 0226h config code
  device_type = register_data[0]; //currently device type 40h = DS1922L allowed only.
  Serial.print("device type =");
  Serial.println(device_type);
}

void read_temp(void) {
  int ta1_raw = 0x00;      //address for temp data starts at 1000h
  byte ta1_l, ta1_h;
  byte ta2 = 0x10;
  int sample_counter = 1; //start with 1st sample
  float temp = 0;
  unsigned long t_temp_ok = 0; //timestamp of first measurement over 35 degrees
  temp_min = 50;
  char message[50];
  while (sample_counter <= mission_count ) {
    ta1_h = ta1_raw;
    ta1_l = ta1_raw + 1;
    register_data[0] = GetDataLoggerRegister(ow_device, ta1_h, ta2); // lower byte temp
    register_data[1] = GetDataLoggerRegister(ow_device, ta1_l, ta2); // higher byte temp
    temp = CalcTemperatureCalData(ow_device, register_data[1], register_data[0]);
    t1 = t_start + (sample_counter - 1) * mission_sample_rate; // timestamp of sample
    if (temp > 35 && t_temp_ok  < 1 ) { // if temp > 35 degrees and first such value
      t_temp_ok = t1;
      Serial.print( "First measurement > 35 degrees:" );
      Serial.printf( "%02d:%02dh\n", hour(t_temp_ok), minute(t_temp_ok));
    }
    sprintf( message, "%d-%02d-%02d %02d:%02d:%02d No: %d T: %02.2f", year(t1), month(t1), day(t1),
             hour(t1), minute(t1), second(t1), sample_counter, temp );
    Serial.println( message );
    if (hour(t1) >= 0 && hour(t1) < 5 && t_temp_ok > 0 && ((t1 - t_temp_ok) >= 1800 ) && ((t1 - t_start) < 86400) && (temp < temp_min)) {
      //count only values for minimum if:
      // between 0:00h and 5:00h  AND
      // a temperatur > 35 degrees has been found, and it is at least 30min ago AND
      // only first 24h (avoid mutiple days measurement in one mission)
      temp_min = temp;
      t_min = t1;
      Serial.printf("New min temperature found: %02.2f\n", temp );
    }
    sample_counter ++;
    ta1_raw = ta1_raw + 2;
    if (ta1_raw > 254) {
      ta1_raw = 0;
      ta2 = ta2 + 1;
    }
    blink( 1 ); //blink and delay
  }
  if ((temp_min > 40) || (temp_min < 35)) {
    temp_min = 0;
    Serial.println("No min temperature between 35 and 40 C.");
  }
}

time_t read_mission_timestamp( void ) {
  unsigned long timesecs;
  for (byte i = 0; i < 6; i++) { // (0219h ... 021Eh) - Mission Timestamp
    register_data[i] = GetDataLoggerRegister(ow_device, 0x19 + i, 0x02);
  }
  tm_start.Second = BCDToBin( register_data[0] );
  tm_start.Minute = BCDToBin( register_data[1] );
  tm_start.Hour = BCDToBin( register_data[2] );
  tm_start.Day = BCDToBin( register_data[3] );
  tm_start.Month = BCDToBin( register_data[4] );
  tm_start.Year = BCDToBin( register_data[5] ) + 30; //diff to 1970, register 2 digit year
  timesecs = makeTime( tm_start );
  return timesecs;
}

void reconnect() {
  int try_no = 0; //count attempts to connect
  // Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  while (!client.connected() && try_no < TRY_MAX)
  {
    Serial.print(".");
    try_no++;
    // Create a random client ID
    String clientId = "ESP8266-IBUT1";
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      //once connected to MQTT broker, subscribe command if any
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
} //end reconnect()

void blink(int times) {
  int i = 0;
  while ( i < times)
  {
    digitalWrite(LED_PIN, HIGH);   // Turn the external LED on
    delay(10);
    digitalWrite(LED_PIN, LOW);   // Turn the external LED off
    delay(100);
    i++;
  }
}

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  delay(500);
  display.begin();            // initializes the display
  display.setBacklight(50);  // set the brightness to 50 %
  while (ow_device[0] != 0x41) {
    Serial.println("Searching for IButton Data Logger...");
    display.print("IBUT");
    SearchOneWireDevices();  //Search for attached 1-wire devices
    Serial.println(attached_devices);
    blink(1);
    delay(1000);
  }
  display.print("READ");
  Serial.println("IButton Data Logger found:");
  Serial.println("wait 2 sec....");
  delay(2000);

  blink(3);
  read_reg_all();
  delay(500);

  read_reg_selected();

  if ((mission_count > 0) &&      //only if data available
      (device_type == 0x40) &&    //only for DS1922L implemented
      (tlfs == 1 ))  {             //only if high resolution temperature 16bit
    t_start = read_mission_timestamp();
    Serial.print("Mission timestamp: ");
    Serial.println(t_start);
    read_temp();
    if (temp_min > 35 ) {

      //display min time
      char msg[4];
      sprintf(msg, "%02d%02d", hour(t_min), minute(t_min));
      display.setColonOn( 1 );
      display.print(msg);
      delay(5000);

      //display min temperature
      int min_part1, min_part2;
      char tempmin_str[4];
      min_part1 = temp_min;
      min_part2 = (float)((float)temp_min - min_part1 + 0.005) * 100; //round to 2 decimals
      sprintf(tempmin_str, "%02d%02d", min_part1, min_part2);
      display.print(tempmin_str);
      delay(5000);
      display.setColonOn( 0 );

      if (mip_status == 1) {      //only connect if mip==1
        setup_wifi();
        //if (espClient.status == WL_CONNECTED) { //if wifi connected, try to send temp_min via MQTT; if not, skip sending
        client.setServer(mqtt_server, 1883);
        if (!client.connected()) {
          reconnect();
        }
      }
      if ((client.connected()) && mip_status == 1 && mission_count > 0) { //only if Wifi and mission in progress
        sprintf( msg, "%02.2f", temp_min );
        client.publish("IbuttonBG/TempMin", msg);
        sprintf( msg, "%d-%02d-%02d %02d:%02d:%02d", year(t_min), month(t_min), day(t_min),
                 hour(t_min), minute(t_min), second(t_min));
        client.publish("IbuttonBG/TimeMin", msg);
        sprintf( msg, "%02.2f°C %02d:%02dh", temp_min, hour(t_min), minute(t_min));
        client.publish("IbuttonBG/state", msg);
        display.print("SENT");
        blink(5);

        display.setColonOn( 0 );
      }
    } // if temp_min != 0
  }
  delay(1000);
  display.print("MIP ");
  delay(1000);
  if (mip_status == 1)
  {
    Serial.println("MIP Status is ON");
    display.print("ON  ");

  } else {
    Serial.println("MIP Status is OFF");
    display.print("OFF ");
  }
}


void loop(void) {
  button_state = digitalRead(BUTTON_PIN); //read push button
  if (button_state != old_button_state) {
    delay(200);
    old_button_state = button_state;
    if (button_state == HIGH) {
      if (mip_status == 1) {  // Stop mission
        display.print( "STOP");
        Serial.print("Attempting to stop mission.");
        blink(5);
        one_wire_bus.reset();
        one_wire_bus.select(ow_device);
        one_wire_bus.write(0x33);  // Stop mission
        // Send 64 bit password + 1 FFh byte
        for (byte i = 0; i < 9; i++) {
          one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password + 1 Dummy byte
        }
        blink(1);
      } else  {               //Start mission
        display.print( "STAR");
        Serial.print("Attempting to start mission.");
        blink(5);
        if (osc_status == 1) { //RTC must be running; otherwise would need to set time first
          one_wire_bus.reset();
          one_wire_bus.select(ow_device);
          one_wire_bus.write(0x96);  // Memory Clear
          // Send 64 bit password + 1 FFh byte
          for (byte i = 0; i < 9; i++) {
            one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password + 1 Dummy byte
          }
          blink(1);
          //read MEMCLR status
          register_data[0] = GetDataLoggerRegister(ow_device, 0x15, 0x02); // 0215h mission status
          if ((register_data[0] & 0x08) > 0 ) { // MEMCLER bit
            Serial.print("Memory cleared");
            display.print("MEM CLEARED");
          } else {
            Serial.print("Memory not cleared");
            display.print("ERR MEM CLEAR");
          }
          blink(1);
          display.print("    ");
          SetDataLoggerRegister(ow_device, 0x06, 0x02, 0x0A); // 0206h set sample rate 10min (low byte)
          SetDataLoggerRegister(ow_device, 0x07, 0x02, 0x00); // 0207h set sample rate 10min (high byte)
          SetDataLoggerRegister(ow_device, 0x12, 0x02, 0x01); // 0212h set sample rate to minutes (not seconds)
          SetDataLoggerRegister(ow_device, 0x13, 0x02, 0xC5); // 0213h set SUTA=0, RO=0, TLFS=high, ETL=1 (11000101)
          Serial.print("Registers set");
          blink(1);
          one_wire_bus.reset();
          one_wire_bus.select(ow_device);
          one_wire_bus.write(0xCC);  // Start mission
          // Send 64 bit password + 1 FFh byte
          for (byte i = 0; i < 9; i++) {
            one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password + 1 Dummy byte
          }
          blink(2);
        } else {
          Serial.println("RTC is off, cannot start");
          display.print("RTC OFF");
        }


      }  //toggle
      //read MIP status
      register_data[0] = GetDataLoggerRegister(ow_device, 0x15, 0x02); // 0215h mip status
      if ((register_data[0] & 0x02) > 0 ) { // MIP bit
        mip_status = 1;
      } else {
        mip_status = 0;
      }
      Serial.print("MIP status =");
      Serial.println(mip_status);

    }
  }
  if (mip_status == 1)
  {
    digitalWrite(LED_PIN, HIGH); // set the LED on

  } else {
    digitalWrite(LED_PIN, LOW); // set the LED off
  }
  delay(200);
}
