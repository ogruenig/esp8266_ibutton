# esp8266_ibutton
OneWire Ibutton data logger reader for ESP8266

This program can be used to read temperatures from a DS1922L data logger "Thermocron" ibutton using an ESP8266 micro controller (I use a Wemos D1 Mini Pro).
The program initially asks to connect an Ibutton, showing "IBUT" on the 7 segment display.
When an Ibutton is connected, it reads in all the temperatures, flashing the LED.
It then calculates the minimum temperature during the previous night (between 0 and 5 o'clock), e.g. to calculate minimum body temperature; you may modify the rule as you require.
It shows the time and minimum temperature on a 7 segment display.
If a mission is in progress, it then tries to connect to local WIFI (modify your connection) and to a local MQTT server, and sends the minimum temperature via MQTT to be logged (I use this to store it in a FHEM home automation system and plot nice graphs).
The it shows wheter a mission is in progress on the 7 segment display (on/off) and also the LED lights up if a mission is in progress, and is off if the mission is stopped.
You may now press the button to switch the mission on or off; the LED will turn on/off if the mission status is changed.
