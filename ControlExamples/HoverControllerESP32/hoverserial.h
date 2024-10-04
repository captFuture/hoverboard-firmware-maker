//#define FEEDBACK_ROBO   // adds current and odometer to the feedback struct

//#define DEBUG_RX      // for debugging the serial receive data
#define DEBUG_HOVER   // will serial.print the feedback struct

#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   //uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
#ifdef FEEDBACK_ROBO  
   int16_t  iAmpL;    // ROBO 100*A
   int16_t  iAmpR;    // ROBO 100*A
   int16_t  iOdomL;    // ROBO 100*A
   int16_t  iOdomR;    // ROBO 100*A
 #endif
   int16_t  boardTemp;
   uint16_t cmdLed;
   //uint16_t checksum;
} SerialFeedback;


template <typename O,typename I> void SetupHoverEsp32(O& oSerial, I iBaud, I pin_RX, I pin_TX)
{
  // Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  oSerial.begin(iBaud, SERIAL_8N1, pin_RX, pin_TX);  //Serial1 = 0, 4   Serial2 = 16,17;
}

template <typename O,typename I> void SetupHoverArduino(O& oSerial, I iBaud)
{
  oSerial.begin(iBaud);
}


// ########################## SEND ##########################
//void Send(Serial& oSerial, int16_t uSteer, int16_t uSpeed)
template <typename O,typename U> void Send(O& oSerial, U uSteer, U uSpeed)
{
  // Create command
  //Serial.print("uSteer"); Serial.println(uSteer);
  //Serial.print("uSpeed"); Serial.println(uSpeed);
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  //Serial.print("uSpeed: ");Serial.println(uSpeed);
  // Write to Serial
  oSerial.write((uint8_t *) &Command, sizeof(Command)); 
}
template <typename O,typename I> void SendLR(O& oSerial, I iSpeedLeft, I iSpeedRight) // -1000 .. +1000
{
  // speed coeff in config.h must be 1.0 : (DEFAULT_)SPEED_COEFFICIENT   16384
  // steer coeff in config.h must be 0.5 : (DEFAULT_)STEER_COEFFICIENT   8192 
  Send(oSerial,iSpeedRight,iSpeedLeft);
}



void HoverLog(SerialFeedback& Feedback)
{
  #ifdef DEBUG_HOVER
    //Serial.print("start="); Serial.print(Feedback.start,HEX);
    Serial.print("cmd1:");Serial.print(Feedback.cmd1);
    Serial.print("\tcmd2:");Serial.print(Feedback.cmd2);
    Serial.print("\tvR:");Serial.print(Feedback.speedR_meas);
    Serial.print("\tvL:");Serial.print(Feedback.speedL_meas);
    Serial.print("\tV:");Serial.print(Feedback.batVoltage/100.0);
    #ifdef FEEDBACK_ROBO
      Serial.print("\t AL:");Serial.print(Feedback.iAmpL/100.0);
      Serial.print("\t AR:");Serial.print(Feedback.iAmpR/100.0);
      Serial.print("\txL:");Serial.print(Feedback.iOdomL);
      Serial.print("\txR:");Serial.print(Feedback.iOdomR);
    #endif
    Serial.print("\tT:");Serial.println(Feedback.boardTemp/100);
    //Serial.print("\tled:");Serial.println(Feedback.cmdLed);
    //Serial.print("\tcrc="); Serial.println(Feedback.checksum,HEX);
  #endif
  float speedFactor = 2.0 * 3.14 * 0.08255 * 60 /1000;
  float tempDriveSpeed = (float)Feedback.speedL_meas * speedFactor;
  driveSpeed = abs((int)tempDriveSpeed);
}


// ########################## RECEIVE ##########################
#ifdef DEBUG_RX
  unsigned long iLastRx = 0;
#endif

//boolean Receive(Serial& oSerial, SerialFeedback& Feedback)
template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iAvail = oSerial.available() - sizeof(Feedback);
  int8_t iFirst = 1;
  while (iAvail >= 3+iFirst )
  {
    byte c = oSerial.read();  // Read the incoming byte
    iAvail--;

    #ifdef DEBUG_RX
      if (millis() > iLastRx + 50)  Serial.println();
      Serial.print((c < 16) ? " 0" : " ");
      Serial.print(c,HEX); 
      iLastRx = millis();
    #endif
    
    if (iFirst) // test first START byte
    {
      if (c == (byte)START_FRAME) //if (c == 0xCD)
      {
        iFirst = 0;
      }
    }
    else  // test second START byte
    {
      if (c == START_FRAME >>8 ) //if (c == 0xAB)
      {
        SerialFeedback tmpFeedback;
        byte* p = (byte *)&tmpFeedback;
        for (int i = sizeof(SerialFeedback); i>0; i--)
          *p++    = oSerial.read();

        #ifdef DEBUG_RX
          Serial.print(" -> ");
          HoverLog(tmpFeedback);
        #endif

        uint16_t chkRead = oSerial.read();
        chkRead |= (uint16_t)oSerial.read() << 8;

        uint16_t checksum;
        checksum = (uint16_t)(START_FRAME ^ tmpFeedback.cmd1 ^ tmpFeedback.cmd2 ^ tmpFeedback.speedR_meas ^ tmpFeedback.speedL_meas
  //                      ^ tmpFeedback.iAmpL ^ tmpFeedback.iAmpR // ROBO
  //                      ^ tmpFeedback.iOdomL ^ tmpFeedback.iOdomR // ROBO
                            ^ tmpFeedback.batVoltage ^ tmpFeedback.boardTemp ^ tmpFeedback.cmdLed);

        if (checksum == chkRead)  //tmpFeedback.checksum
        {
            memcpy(&Feedback, &tmpFeedback, sizeof(SerialFeedback));
            #ifdef DEBUG_RX
              //Serial.println(" :-)))))))))))))))))))))))))))");
            #endif
            
            return true;
        }
        #ifdef DEBUG_RX
          Serial.print(chkRead, HEX);
          Serial.print(" != ");
          Serial.print(checksum,HEX);
          Serial.println(" :-(");
        #endif
        return false;       
      }
      if (c != (byte)START_FRAME) //if (c != 0xCD)
        iFirst = 1;
    }
  }
  return false;
}