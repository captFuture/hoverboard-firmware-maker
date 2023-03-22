#define DEBUG_HOVER   // will serial.print the feedback struct

#include "hoverserial.h"

#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing

#include <SoftwareSerial.h>
SoftwareSerial oSerialHover(9,8); // eHposer Arduino Mini
/* or for ESP32 with its nice 3 hardware serial ports
#define oSerialHover Serial1
#define oSerialBoardB Serial2
*/

SerialFeedback oHoverFeedback;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Arduino example code");
  pinMode(LED_BUILTIN, OUTPUT);

  SetupHoverArduino(oSerialHover,19200);    //  8 Mhz Arduino Mini too slow for 115200 !!!
  //or SetupHoverEsp32(oSerialBoardA, 115200, pin_RX, pin_TX);

}

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;

void loop() 
{
  unsigned long timeNow = millis();

  // Check for new received data
  if (Receive(oSerialHover,oHoverFeedback)) 
  {
    HoverLog(oHoverFeedback);
  }


  // Send commands
  if (iTimeSend > timeNow) 
    return;
    
  iTimeSend = timeNow + TIME_SEND;
  Send(oSerialHover,0, SPEED_MAX_TEST - 2*abs(iTest) );
  //Send(oSerialHover,0, (timeNow % 10000 > 2000) ? SPEED_MAX_TEST : 0 );

  // Calculate test command signal
  iTest += 10;
  if (iTest > iTestMax) iTest = -iTestMax;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}
