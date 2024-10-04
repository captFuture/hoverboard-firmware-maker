#include <Bluepad32.h>
//#include <M5Stack.h>

#define RX_PIN 16
#define TX_PIN 17

int speedFW = 50;
int speedRW = -50;
float ma = speedFW * 0.01;

int requested_state;

HardwareSerial odrive_serial(2);
#include <ODriveArduino.h>
// Printing with stream operator
template<class T> inline Print& operator<<(Print& obj, T arg) {
  obj.print(arg);
  return obj;
}
template<> inline Print& operator<<(Print& obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// ODrive object
ODriveArduino odrive(odrive_serial);



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

  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);

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
    ctl->l2(),  // bitmask of pressed "misc" buttons
    ctl->r1(),  // bitmask of pressed "misc" buttons
    ctl->r2()  // bitmask of pressed "misc" buttons
  );
  int JoyAxisY = ctl->axisY();
  int JoyAxisRY = ctl->axisRY();
  int mapJoyAxisY;
  int mapJoyAxisRY;

  mapJoyAxisY = map(JoyAxisY, -512, 512, speedFW, speedRW);
  mapJoyAxisRY = map(JoyAxisRY, -512, 512, speedFW, speedRW);

  Serial.print("l: ");
  Serial.print(JoyAxisY);
  Serial.print(" | r: ");
  Serial.print(JoyAxisRY);
  Serial.println("");
  Serial.print("lm: ");
  Serial.print(mapJoyAxisY);
  Serial.print(" | rm: ");
  Serial.print(mapJoyAxisRY);
  Serial.println();

  float lmf = mapJoyAxisY * 0.01;
  float rmf = mapJoyAxisRY * 0.01;
  Serial.print("lmf: ");
  Serial.print(lmf);
  Serial.print(" | rmf: ");
  Serial.print(rmf);
  Serial.println();

  if (lmf > 0.1 && lmf < -0, 1) { odrive.SetVelocity(0, lmf); } 
  if (rmf > 0.1 && rmf < -0, 1) { odrive.SetVelocity(1, -rmf); }
}

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  if (ctl->x()) {
    speedFW = 600;
    speedRW = -600;
  }
  if (ctl->a()) {
    speedFW = 100;
    speedRW = -100;
  }
  if (ctl->b()) {
    speedFW = 200;
    speedRW = -200;
  }
  if (ctl->y()) {
    speedFW = 400;
    speedRW = -400;
  }
  if (ctl->l1()) {
    Serial.printf("enabling Motors");
    for (int axis = 0; axis < 2; ++axis) {
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      odrive.run_state(axis, requested_state, false);
    }
  }
  if (ctl->r1()) {
    Serial.printf("disabling Motors");
    for (int axis = 0; axis < 2; ++axis) {
      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      odrive.run_state(axis, requested_state, false);
    }
  }
  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
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

void testMotors() {
  Serial.println("Executing test move");

  Serial.println("Ramping up");
  for (float ph = 0.0f; ph < ma; ph += 0.01f) {
    odrive.SetVelocity(0, ph);
    odrive.SetVelocity(1, -ph);
    Serial.print(" ");
    Serial.print(ph);
    delay(10);
  }
  delay(1000);
  Serial.println();
  Serial.println("Ramping down");
  for (float ph = ma; ph > 0.0f; ph -= 0.01f) {
    odrive.SetVelocity(0, ph);
    odrive.SetVelocity(1, -ph);
    Serial.print(" ");
    Serial.print(ph);
    delay(10);
  }

  //M5.Lcd.clearDisplay();
  //M5.Lcd.setCursor(10, 10);
  //M5.Lcd.printf("DONE");
  Serial.println("DONE");
}

// Arduino setup function. Runs in CPU 1
void setup() {
  //M5.begin();
  // ODrive uses 115200 baud
  odrive_serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(115200);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  //BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 50.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
  }
  for (int axis = 0; axis < 2; ++axis) {
    requested_state = ODriveArduino::AXIS_STATE_IDLE;
    odrive.run_state(axis, requested_state, false);
  }

  Serial.println("Ready!");
  //Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  //M5.update();
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  //     vTaskDelay(1);
  /*
  if (M5.BtnA.pressedFor(1000)) {
    M5.Lcd.clearDisplay();
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("enabling Motors");
    for (int axis = 0; axis < 2; ++axis) {
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      odrive.run_state(axis, requested_state, false);
    }
  }
  if (M5.BtnB.wasReleased()) {
    M5.Lcd.clearDisplay();
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("testing Motors");
    testMotors();
  }
  if (M5.BtnC.pressedFor(1000)) {
    M5.Lcd.clearDisplay();
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("disabling Motors");
    for (int axis = 0; axis < 2; ++axis) {
      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      odrive.run_state(axis, requested_state, false);
    }
  }
*/

  if (Serial.available()) {
    char c = Serial.read();
    // Sinusoidal test move
    if (c == 's') {
      testMotors();
    }
  }
}
