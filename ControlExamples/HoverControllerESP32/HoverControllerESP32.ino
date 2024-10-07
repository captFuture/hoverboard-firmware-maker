#include <Bluepad32.h>

struct Config {
  char cartname[64] = "MakerCart";
  char ssid[64] = "TarantlVR";
  char passwd[64] = "somepassword";
  int speed_max = 100;
  int speed_min = -100;
  int steer_max = 100;
  int steer_min = -100;
  unsigned int accel_min = 200;
  unsigned int decel_min = 200;
  int boost_max = 100;
};

Config config;

const char* ssid = config.ssid;
const char* password = config.passwd;

#define TaskStackSize   5120
#define PIN_SDA 21
#define PIN_SCL 22

static unsigned int controller_type;

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval

#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
HardwareSerial &HoverSerial = Serial2;

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
boolean motorOn = false;
boolean triggerstate = false;
boolean switchState = false;
unsigned long timeNow = 0;
boolean switchMotors = true;
boolean switchDirections = true;

/* Telemetry */
int16_t driveSpeed = 0;
//int16_t sentSpeed = 0;
int16_t speedR = 0;
int16_t speedL = 0;
int16_t batVoltage = 0;
int16_t boardTemp = 0;

boolean triggerReleased = true;
int16_t configNum = 0;

int16_t leftRightCalibration = 0;
int16_t forwardReverseCalibration = 0;
int16_t thresholdMovement = 100;
int16_t forwardReverseValueR = 0;
int16_t forwardReverseInputR = 0;
int16_t forwardReverseValueL = 0;
int16_t forwardReverseInputL = 0;
int16_t OLDleftRightValue = 0;
int16_t OLDforwardReverseValue = 0;
unsigned int accel = config.accel_min; // Acceleration time [ms]
unsigned int decel = config.accel_min; // Acceleration time [ms]
int16_t safetyCool = 10;

int16_t myDrive = 0;
int16_t oldmyDrive = 0;

#include "hoverserial.h"
#define HoverSerial Serial2
SerialFeedback oHoverFeedback;

int requested_state;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  //odrive.SetVelocity(0, 0);
  //odrive.SetVelocity(1, 0);

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}



void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, sl: 0x%02x, tl: 0x%02x, sr: 0x%02x, tr: 0x%02x\n",
    ctl->index(),       // Controller Index
    ctl->dpad(),        // D-pad
    ctl->buttons(),     // bitmask of pressed buttons
    ctl->axisX(),       // (-511 - 512) left X Axis
    ctl->axisY(),       // (-511 - 512) left Y axis
    ctl->axisRX(),      // (-511 - 512) right X axis
    ctl->axisRY(),      // (-511 - 512) right Y axis
    ctl->brake(),       // (0 - 1023): brake button
    ctl->throttle(),    // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->l1(),  // bitmask of pressed "misc" buttons
    ctl->l2(),  // bitmask of pressed "misc" buttons, 
    ctl->r1(),  // bitmask of pressed "misc" buttons
    ctl->r2()  // bitmask of pressed "misc" buttons
  );



  int16_t steerInput = ctl->axisX();
  int16_t steerValue = map(steerInput, 511, -512, config.steer_min-(config.boost_max*configNum), config.steer_max+(config.boost_max*configNum));

  int16_t forwardInput = ctl->throttle();
  int16_t backwardInput = ctl->brake();

  int16_t forwardValue = map(forwardInput, 0, 1024, 0, config.speed_max+(config.boost_max*configNum));
  int16_t backwardValue = map(backwardInput, 0, 1024, 0, config.speed_min-(config.boost_max*configNum));

  int16_t driveValue = backwardValue + forwardValue;
  if(steerValue < 0){
    forwardReverseValueL = driveValue;
    forwardReverseValueR = driveValue + abs(steerValue);
  }else if(steerValue > 0){
    forwardReverseValueL = driveValue + abs(steerValue);
    forwardReverseValueR = driveValue;
  }else{

  }


/*
if(switchDirections){
  forwardReverseValueL = map(forwardReverseInputL, 512, -512, config.speedl_min-(config.boost_max*configNum), config.speedl_max+(config.boost_max*configNum));
  forwardReverseValueR = map(forwardReverseInputR, 512, -512, config.speedr_min-(config.boost_max*configNum), config.speedr_max+(config.boost_max*configNum));
}else{
  forwardReverseValueL = map(forwardReverseInputL, -512, 512, config.speedl_min-(config.boost_max*configNum), config.speedl_max+(config.boost_max*configNum));
  forwardReverseValueR = map(forwardReverseInputR, -512, 512, config.speedr_min-(config.boost_max*configNum), config.speedr_max+(config.boost_max*configNum));
}*/


  /*Serial.print("l: ");
  Serial.print(JoyAxisY);
  Serial.print(" | r: ");
  Serial.print(JoyAxisRY);
  Serial.println("");
  Serial.print("lm: ");
  Serial.print(mapJoyAxisY);
  Serial.print(" | rm: ");
  Serial.print(mapJoyAxisRY);
  Serial.println();*/

}

void processGamepad(ControllerPtr ctl) {
  if (ctl->x()) {
    motorOn = !motorOn;
    delay(1000);
  }
  if (ctl->a()) {
    configNum = configNum+1;
    if(configNum > 4){
      configNum = 0;
       delay(1000);
    }
    delay(200);     
  }
  if (ctl->b()) {

  }
  if (ctl->y()) {

  }
  if (ctl->l1()) {
    Serial.printf("change motors");
    switchMotors = !switchMotors;
    delay(200);
  }
  if (ctl->r1()) {
    Serial.printf("reverse morors");
    switchDirections = !switchDirections;
    delay(200);
  }

  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  SetupHoverArduino(HoverSerial,HOVER_SERIAL_BAUD);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  
  BP32.forgetBluetoothKeys();
  
  BP32.enableVirtualDevice(false);
  Serial.println("Setup Done");
}

unsigned long iTimeSend = 0;

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  if (Receive(HoverSerial,oHoverFeedback)) 
  {
    HoverLog(oHoverFeedback);
  }
  if (millis() > TIME_SEND + iTimeSend ){
    iTimeSend = millis();

    if(motorOn == true){
      int16_t leftwheel = forwardReverseValueL;
      int16_t rightwheel = forwardReverseValueR;

      /*if(leftRightInput < 0){
        leftwheel= myDrive ;
        rightwheel= myDrive + abs(leftRightValue);
      }else if(leftRightInput > 0){
        leftwheel= myDrive + abs(leftRightValue);
        rightwheel= myDrive;
      }else{
        leftwheel= myDrive ;
        rightwheel= myDrive;
      }*/

    if(switchMotors == true){
      Send(HoverSerial, rightwheel, leftwheel );
    }else{
      Send(HoverSerial, leftwheel, rightwheel );
    }

      //Send(HoverSerial, leftwheel, rightwheel );
      //Serial.print("Sending: "); Serial.print(myDrive); Serial.print(" "); Serial.print(myDrive); Serial.println(" "); 
    }else{
      myDrive = 0;
      Send(HoverSerial, 0, 0);
    }
  }
}
