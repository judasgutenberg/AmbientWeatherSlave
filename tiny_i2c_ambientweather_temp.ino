/* ATTiny85 as an I2C Slave sending back data from AmbientWeather probes
 * Gus Mueller, July 11 2015
 * Data is collected from AmbientWeather F007TH probes (up to 8 can coexist)
 * and transmitted via I2C as 32 byte packets to a Master (such as a weather station)
 
 * ATTiny Slave code partially based on BroHogan's code, see 
 * https://github.com/adafruit/TinyWireM/blob/master/examples/Tiny85_Temp/Tiny85_Temp.pde
 * SETUP for ATTiny:
 * ATtiny Pin 1 = (RESET) N/U                      
 * ATtiny Pin 2 = (D3) SoftSerial TX (not necessary)
 * ATtiny Pin 3 = (D4) Receive Pin from 433MHz receiver board (to receive AmbientWeather data wirelessly, see below for recommended receiver)                     
 * ATtiny Pin 4 = GND
 * ATtiny Pin 5 = I2C SDA on DS1621  & GPIO        
 * ATtiny Pin 6 = (D1) SoftSerial RX (not necessary)
 * ATtiny Pin 7 = I2C SCK on DS1621  & GPIO        
 * ATtiny Pin 8 = VCC (2.7-5.5V)
 
 * NOTE! - It's very important to use pullups on the SDA & SCL lines!
 * Current Rx & Tx buffers set at 32 bytes - see usiTwiSlave.h
 * NOT A PROBLEM because I send data for all possible 8 sensors as a 32 byte packet
 * Credit and thanks to Don Blake for his usiTwiSlave code. 
 * More on TinyWireS usage - see TinyWireS.h
 */
#include <SoftwareSerial.h> //only used for debugging
SoftwareSerial softSerial(1,3); // RX, TX
#include "TinyWireS.h"   // wrapper class for I2C slave routines

//ambientweathercode, based on code by Ron C Lewis, see:
//https://eclecticmusingsofachaoticmind.wordpress.com/2015/01/21/home-automation-temperature-sensors/
//these are great receivers, don't use the non-superheterodyne kind:
//http://www.ebay.com/itm/281471334108?_trksid=p2060353.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT
#define MAX_BYTES 7
#define countof(x) (sizeof(x)/sizeof(x[0]))
// Interface Definitions
int RxPin           = 4;   //The number of signal from the Rx
// Variables for Manchester Receiver Logic:
word    sDelay     = 242;   //30//121//242 //Small Delay about 1/4 of bit duration
word    lDelay     = 484;  //60//242//484//Long Delay about 1/2 of bit duration, 1/4 + 1/2 = 3/4
byte    polarity   = 1;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
byte    tempBit    = 1;    //Reflects the required transition polarity
boolean firstZero  = false;//flags when the first '0' is found.
boolean noErrors   = true; //flags if signal does not follow Manchester conventions
//variables for Header detection
byte    headerBits = 10;   //The number of ones expected to make a valid header
byte    headerHits = 0;    //Counts the number of "1"s to determine a header
//Variables for Byte storage
boolean sync0In=true;      //Expecting sync0 to be inside byte boundaries, set to false for sync0 outside bytes
byte    dataByte   = 0xFF; //Accumulates the bit information
byte    nosBits    = 6;    //Counts to 8 bits within a dataByte
byte    maxBytes   = MAX_BYTES;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error
byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes
//Variables for multiple packets
byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 last data downloads 
byte    nosRepeats = 3;    //Number of times the header/data is fetched at least once or up to 4 times
//Banks for multiple packets if required (at least one will be needed)
byte  manchester[MAX_BYTES];   //Stores banks of manchester pattern decoded on the fly
// Variables to prepare recorded values for Ambient
byte   stnId     = 0;      //Identifies the channel number
int   dataType  = 0;    //Identifies the Ambient Thermo-Hygrometer code
int   Newtemp   = 0;
int   Newhum    = 0;
//end ambientweathercode
unsigned long milliReadings[8];
char dataOut[32]; //defines the data packed transmitted by I2C

byte cursorPos=0;
#define I2C_SLAVE_ADDR  7            // i2c slave address (38)
 
void setup()
{
  softSerial.begin(19200); //just for debugging out through the softserial port.  
 
  TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode
  TinyWireS.onRequest(requestEvent); 
  
  pinMode(RxPin, INPUT);
  //softSerial.print("booted");
}


void loop()
{
  if(millis() % 4000 ==0) //don't want to do this very often. this way we only do it every four seconds
  {
    printDataLocally();
  }
  TinyWireS_stop_check();
  byte byteRcvd = 0;
  tempBit=polarity; //these begin the same for a packet
  noErrors=true;
  firstZero=false;
  headerHits=0;
  nosBits=6;
  nosBytes=0;
  while (noErrors && (nosBytes<maxBytes))
  {
    while(digitalRead(RxPin)!=tempBit)
    {
      //pause here until a transition is found
    }//at Data transition, half way through bit pattern, this should be where RxPin==tempBit
    
    delayMicroseconds(sDelay);//skip ahead to 3/4 of the bit pattern
    // 3/4 the way through, if RxPin has changed it is definitely an error
    
    if (digitalRead(RxPin)!=tempBit)
    {
      noErrors=false;//something has gone wrong, polarity has changed too early, ie always an error
    }//exit and retry
    else
    {
      delayMicroseconds(lDelay);
      //now 1 quarter into the next bit pattern,
      if(digitalRead(RxPin)==tempBit) //if RxPin has not swapped, then bitWaveform is swapping
      {
        //If the header is done, then it means data change is occuring ie 1->0, or 0->1
        //data transition detection must swap, so it loops for the opposite transitions
        tempBit = tempBit^1;
      }//end of detecting no transition at end of bit waveform, ie end of previous bit waveform same as start of next bitwaveform
      
      //Now process the tempBit state and make data definite 0 or 1's, allow possibility of Pos or Neg Polarity 
      byte bitState = tempBit ^ polarity;//if polarity=1, invert the tempBit or if polarity=0, leave it alone.
      if(bitState==1) //1 data could be header or packet
      {
        if(!firstZero)
        {
          headerHits++;
       
        }
        else
        {
          add(bitState);//already seen first zero so add bit in
        }
      }//end of dealing with ones
      else
      {  //bitState==0 could first error, first zero or packet
        // if it is header there must be no "zeroes" or errors
        if(headerHits<headerBits)
        {
          //Still in header checking phase, more header hits required
          noErrors=false;//landing here means header is corrupted, so it is probably an error
        }//end of detecting a "zero" inside a header
        else
        {
          //we have our header, chewed up any excess and here is a zero
          if (!firstZero) //if first zero, it has not been found previously
          {
            firstZero=true;
            add(bitState);//Add first zero to bytes 
         
          } 
          else
          {
            add(bitState);
          } 
        }//end of dealing with a first zero
      }//end of dealing with zero's (in header, first or later zeroes)
    }//end of first error check
  }//end of while noErrors=true and getting packet of bytes
  
}

void printDataLocally() //used for debugging.  not necessary if you want to use the SoftSerial pins for something else
{
  for(int charPos=0; charPos<32; charPos++)
  {
    int val = (int)dataOut[charPos];
    
      if(charPos % 4 == 0)
      {
      softSerial.print((charPos/4)+1);
      softSerial.print(": ");
      //unsigned int timeVal=(unsigned int)val;
      unsigned int timeVal=(byte)(collapseTime(millis() - milliReadings[charPos/4]));
     
      if(timeVal<121)
      {
        softSerial.print(timeVal);
        softSerial.print("s");
      }
      if(timeVal>120  && timeVal<180)
      {
        softSerial.print(timeVal-120);
        softSerial.print("m");
      }
      if(timeVal>179  && timeVal<204)
      {
        softSerial.print(timeVal-180);
        softSerial.print("h");
      }
      if(timeVal>203  && timeVal<255)
      {
        softSerial.print(timeVal-204);
        softSerial.print("d");
      }
      if(timeVal>203  && timeVal<255)
      {
        softSerial.print("inf");
      }
      softSerial.print(" ");
      }
      else if(charPos % 4 == 1)
      {
      softSerial.print(val);
      softSerial.print('.');
       
      }
      else if(charPos % 4 == 3)
      {
      softSerial.print(val);
      softSerial.print("% ");
      }
      else
      {
      softSerial.print(val);
      softSerial.print(' ');
      }
   
  }
  softSerial.println(); 
}

//send a byte to the I2C master.  
//evidently the master calls this 32 times, not just once as I'd orignally thought
void requestEvent()
{
  byte val;
  val=(byte)dataOut[cursorPos];
  if(cursorPos % 4==0)
  {
    //if we're at the first byte of a four byte sensor packet, send the collapseTime rendering of time since last received from sensor
    TinyWireS.send((byte)(collapseTime(millis() - milliReadings[cursorPos/4])));
    //TinyWireS.send(val);
  }
  else
  {
    TinyWireS.send(val);
  }
  cursorPos++;
  if(cursorPos>31)
  {
    cursorPos=0;
  }
}

byte collapseTime(unsigned long millisDelta)
{
  //represent an expanse of time of as long as a month with a single byte, representing times at different granularities depending
  //how long a period it is
  unsigned int secondsDelta=millisDelta/1000;
  if(secondsDelta<120) //granularity for the first two minutes is one second, for a reading as high as 119
  {
    return secondsDelta;
  
  }
  else if(secondsDelta<3600) //granularity for the first hour is one minute, for a reading as high as 179
  {
    return secondsDelta/60 + 120;
  
  }
  else if(secondsDelta<86400) //granularity for the first day is one hour, for a reading as high as 203
  {
    return secondsDelta/3600 + 180;
  
  }
  else if(secondsDelta<2592000) //granularity for the first month is one day, for a reading as high as 233
  {
    return secondsDelta/86400 + 204;
  
  }
  else
  {
    return 255; //essentially, infinity
  }

}

/////ambientweather functions
void add(byte bitData)
{
  dataByte=(dataByte<<1)|bitData;
  nosBits++;
  if (nosBits==8)
  {
    nosBits=0;
    manchester[nosBytes]=dataByte;
    nosBytes++;
    //Serial.print("B");
  }
  if(nosBytes==maxBytes)
  {
    dataByte = 0xFF;
    // Subroutines to extract data from Manchester encoding and error checking
    
    // Identify channels 1 to 8 by looking at 3 bits in byte 3
    int stnId = ((manchester[3]&B01110000)/16)+1;
    
    // Identify sensor by looking for sensorID in byte 1 (F007th Ambient Thermo-Hygrometer = 0x45)
    dataType = manchester[1];  
    
    // Gets raw temperature from bytes 3 and 4 (note this is neither C or F but a value from the sensor) 
    Newtemp = (float((manchester[3]&B00000111)*256)+ manchester[4]);
    
    // Gets humidity data from byte 5
    Newhum =(manchester [5]); 
    
    
    if ( Checksum (countof(manchester)-2, manchester+1) == manchester[MAX_BYTES-1])
    {
      // Checks sensor is a F007th with a valid humidity reading equal or less than 100
      if ((dataType == 0x45) && (Newhum <= 100))
      {
      
  
        //prepare data as a 32 byte array in this format: #numberOfSensor, integer temp, decimal temperature, humidity, ...
        for (int i=0; i<8; i++)
        {
          if( i ==(int)(stnId-1))
          {
            milliReadings[i]=millis();
            dataOut[i*4]=(char)(i+1);
            dataOut[i*4+1]=(char)((Newtemp-400)/10);
            dataOut[i*4+2]=(char)(Newtemp-(Newtemp/10)*10);
            dataOut[i*4+3]=(char)Newhum;
          }
        }
      
      }
    }
  }
}

void eraseManchester()
{
    for( int j=0; j < 4; j++)
    { 
        manchester[j]=j;
    }
}

uint8_t Checksum(int length, uint8_t *buff)
{
  uint8_t mask = 0x7C;
  uint8_t checksum = 0x64;
  uint8_t data;
  int byteCnt;  
  
  for ( byteCnt=0; byteCnt < length; byteCnt++)
  {
    int bitCnt;
    data = buff[byteCnt];
    
    for ( bitCnt= 7; bitCnt >= 0 ; bitCnt-- )
    {
      uint8_t bit;
      
      // Rotate mask right
      bit = mask & 1;
      mask =  (mask >> 1 ) | (mask << 7);
      if ( bit )
      {
        mask ^= 0x18;
      }
      
      // XOR mask into checksum if data bit is 1      
      if ( data & 0x80 )
      {
        checksum ^= mask;
      }
      data <<= 1; 
    }
  }
  return checksum;
}



