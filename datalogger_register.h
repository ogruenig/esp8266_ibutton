/*
     One Wire iButton Data Logger Programmer

     iButton Data Logger Routines

     GetDataLoggerRegister(one wire address, TA1, TA2)
     - Get ibutton data logger register value

     SetDataLoggerRegister(one wire address, TA1, TA2, data)
     - Write to register on DS1921G and DS1923 ibutton dataloggers. For DS1923, only supporting TA1 < 220h.

     SetDataLoggerTime(timedate structure, one wire address)
     - Set time for DS1921G & DS1923 Data Loggers in BCD format from DS1994 iButton and Arduino clock


     Update: 4/21/10
     
     Author: Jeff Miller   http://arduinofun.blogspot.com/
     
     Copyright 2010 Released under GPLv3 http://www.gnu.org/copyleft/gpl.html
   
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
    
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details: <http://www.gnu.org/licenses/>
     
*/


// ----------------------------------------------------------------------------------------------------------------------
//
// Get Datalogger Control Register Value (address, TA1, TA2)
//
// ---------------------------------------------------------------------------------------------------------------------- 

byte GetDataLoggerRegister(byte* addr, byte TA1, byte TA2)
{
      byte data;
      
      one_wire_bus.reset();
      one_wire_bus.select(addr);        // Button address

      if (addr[0] == 0x41) {
            one_wire_bus.write(0x69);         // Read memory DS1923
        }
      else {
            one_wire_bus.write(0xF0);         // Read memory DS1920G
      }

// Control Register memory address i.e. 020Eh TA1=0x0E, TA2=0x02 
      one_wire_bus.write(TA1);  //TA1 
      one_wire_bus.write(TA2);  //TA2 

      if (addr[0] == 0x41) { //DS1923
          for( byte i = 0; i < 8; i++) {
              one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password 
              }
      } 

// Store current register data into scratchpad[] if TA1<32 (We only care about registers below 220h)
      if (TA1 < 32){
          for (byte i = TA1; i <32; i++) {
            scratchpad[i] = one_wire_bus.read(); // Read scratchpad data to send back
          }
          data = scratchpad[TA1];
      }
      else {
          data = one_wire_bus.read(); // Read Control Register
      }
      
      one_wire_bus.reset();

      return data;
}

// ----------------------------------------------------------------------------------------------------------------------
// SetDataLoggerRegister(one wire address, TA1 byte, TA2 byte, data)
// 
// Write to register on DS1921G and DS1923 datalogger. For DS1923, only supporting TA1 < 220h.
// ----------------------------------------------------------------------------------------------------------------------
void SetDataLoggerRegister(byte* addr, byte TA1, byte TA2, byte data)
{
      byte oldata = GetDataLoggerRegister(addr, TA1, TA2); // variable for call to GetDataLoggerRegister to update scratchpad[0] for writing data back.

     // Write new register data and remaining scratch pad back to memory
      one_wire_bus.select(addr); // Button address
      one_wire_bus.write(0x0F);  // Write Scratchpad
      one_wire_bus.write(TA1);   // TA1 
      one_wire_bus.write(TA2);   // TA2 
      one_wire_bus.write(data);  // Write 1 byte

      // DS1923 Temp Datalogger Family Device
      if (addr[0] == 0x41) { 
          // Need to only write bytes from starting offset TA1 to end of scratch pad (32nd byte)
          for( byte i = TA1 + 1; i < 32; i++) {
                 one_wire_bus.write(scratchpad[i]);  //Write old scratch pad data back 
              }     
          one_wire_bus.reset();
          one_wire_bus.select(addr); // Button address
          one_wire_bus.write(0x99);  // Copy Scratchpad to memory
          // Authorization Code
          one_wire_bus.write(TA1);   //TA1 
          one_wire_bus.write(TA2);   //TA2 
          one_wire_bus.write(0x1F);  //E/S  On DS1923, E/S is always 1Fh

      // Send 64 bit password
          for( byte i = 0; i < 8; i++) {
                 one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password 
              }
          }
        
        else { //DS1920G Family device
            one_wire_bus.reset();
            one_wire_bus.select(addr); // Button address
            one_wire_bus.write(0x55);  // Copy Scratchpad to memory
          // Authorization Code
            one_wire_bus.write(TA1);  //TA1 
            one_wire_bus.write(TA2);  //TA2 
            one_wire_bus.write(TA1);  //E/S  //Last byte address
        }
      
      one_wire_bus.reset();      
}


/*

// SetDataLoggerTime (timedate structure, one wire address)
//
// Set time for DS1921G & DS1923 Data Loggers in BCD format from DS1994
// ----------------------------------------------------------------------------------------------------------------------
void SetDataLoggerTime(timedate *td, byte* addr)
{  
      int dssecond = IntToBCD(td->second);
      int dsminute = IntToBCD(td->minute);
      int dshour = IntToBCD(td->hour);
      int dsday = IntToBCD(td->day);
//      int dsmonth = IntToBCD(td->month) | 0x80; //month plus century bit; | 0x80 required for DS1920G
      int dsmonth = IntToBCD(td->month); // Month plus century bit: 0 for DS1923, 1 for DS1920G
      int dsyear = IntToBCD(td->year - 2000);
      byte oldata;  // Variable for call to GetDataLoggerRegister to update scratchpad[0] for writing data back.

      oldata = GetDataLoggerRegister(addr, 0x00, 0x02); // Read scratchpad so DS1923 can write data back at 0200h
                                
      one_wire_bus.reset();             // Reset one wire bus
      one_wire_bus.select(addr);        // Button address
      one_wire_bus.write(0x0F);         // Write Scratchpad

// Clock memory address starts at 0200h 
      one_wire_bus.write(0x00);         // TA1 
      one_wire_bus.write(0x02);         // TA2 

// Write clock data to scratch pad
      one_wire_bus.write(dssecond);     // 200h
      one_wire_bus.write(dsminute);     // 201h
      one_wire_bus.write(dshour);       // 202h

// Specific code for each datalogger
      
      // DS1920 Temp Datalogger Family Device
        if (addr[0] == 0x21) {
      
            // Write clock data to scratch pad
            one_wire_bus.write(0x01);            // 203h day of week - don't care
            one_wire_bus.write(dsday);           // 204h
            one_wire_bus.write(dsmonth | 0x80);  // 205h month & century; | 0x80 required for 1 for DS1920G
            one_wire_bus.write(dsyear);          // 206h
    
            one_wire_bus.reset();

            one_wire_bus.select(addr); // Button address
            one_wire_bus.write(0x55);  // Copy Scratchpad to memory
            // Authorization Code
            one_wire_bus.write(0x00);  //TA1 
            one_wire_bus.write(0x02);  //TA2 
            one_wire_bus.write(0x06);  //E/S  //Last byte written address 06h
          
           } 
           
           else {
           
             // DS1923 Temp & Humidity Datalogger Family Device
            one_wire_bus.write(dsday);    // 203h
            one_wire_bus.write(dsmonth);  // 204h month & century
            one_wire_bus.write(dsyear);   // 205h
           
            // All 32 bytes of Scratch Pad must be written
            for( byte i = 6; i < 32; i++) {
                 one_wire_bus.write(scratchpad[i]);  //Send bytes to fill remaining Scratch Pad 
                }     
            one_wire_bus.reset();
            one_wire_bus.select(addr); // Button address
            one_wire_bus.write(0x99);  // Copy Scratchpad to memory
            // Authorization Code
            one_wire_bus.write(0x00);  //TA1 
            one_wire_bus.write(0x02);  //TA2 
            one_wire_bus.write(0x1F);  //E/S  On DS1923, E/S is always 1Fh

            // Send 64 bit password
             for( byte i = 0; i < 8; i++) {
                   one_wire_bus.write(0xFF);  //Send (8) 8 byte Dummy Password 
                }
          }
            
      one_wire_bus.reset();       
}
*/
