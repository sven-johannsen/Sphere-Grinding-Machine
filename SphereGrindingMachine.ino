/*
  SphereGrindingMachine.ino

  The sketch was developed for a unique sphere grinding machine.
  It provides a menu structure on an OLED display and enables
  the user to control the grinding machine via buttons.
  It includes functions for initializing the grinding machine
  as well as creating and storing grinding programs.
  
  Author: Sven Johannsen
*/

#include <EEPROM.h>

// Wire library for connection to OLED display via I2C
#include <Wire.h>

// Adafruit library for specific display type
#include <Adafruit_SSD1306.h>

// Adafruit graphics library
#include <Adafruit_GFX.h>

// conversion from steps to micrometer: 10.987 (rounded)
// number of steps per revolution: 5493.426 (rounded)
#define STEPS_PER_MICROMETER 11

// maximum speed of stepper motors, applied during initialization and running to start position
#define MAX_STEPS_PER_SECOND 1020

//main menu is level 1
#define NUMBER_OF_MENU_LEVELS 3

// maximum number of menu items per level
#define MAX_ITEMS_LEVEL_1 4
#define MAX_ITEMS_LEVEL_2 59
#define MAX_ITEMS_LEVEL_3 49

// maximum characters per line on display
#define MAX_CHAR_PER_LINE 19

// each program consists of program segments (A, B, C, D, E) for each DC motor (DC 1, DC 2, DC 3, DC 4)
// the segments contain a duty cycle value and a corresponding timeslot value
// e.g. DC 1 Duty A = 50%, DC 1 Time A = 10 seconds
#define NUMBER_OF_DC_MOTORS 4
#define NUMBER_OF_PROGRAMS 8
#define NUMBER_OF_PROGRAM_SEGMENTS 5

// each stepper driver is controlled by an individual step pin and a shared direction pin
#define DRIVER1_STEP 20
#define DRIVER2_STEP 21
#define DRIVER3_STEP 22
#define DRIVER123_DIR 13

// each DC driver is controlled by two PWM pins and four logic pins => one PWM pin and two logic pins for each DC motor (A and B).
#define DRIVER4_PWMA 12
#define DRIVER4_PWMB 7
#define DRIVER4_AIN1 10
#define DRIVER4_AIN2 11
#define DRIVER4_BIN1 9
#define DRIVER4_BIN2 8
#define DRIVER5_PWMA 6
#define DRIVER5_PWMB 1
#define DRIVER5_AIN1 4
#define DRIVER5_AIN2 5
#define DRIVER5_BIN1 3
#define DRIVER5_BIN2 2

#define BUTTON1 16
#define BUTTON2 14
#define BUTTON3 15
#define BUTTON4 17

#define BUTTON_CHANGECOUNT 5

// OLED display width (in pixels)
#define SCREEN_WIDTH 128

// OLED display height (in pixels). In fact it is 64. However, 32 is used for better readability.
#define SCREEN_HEIGHT 32

// pin for OLED reset (-1 if sharing Teensy reset pin)
#define OLED_RESET -1

// OLED display adress
#define SCREEN_ADDRESS 0x3C


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

IntervalTimer ButtonTimerInterrupt;
IntervalTimer StepperTimerInterrupt;

// the display menu consists of lines (menu items) each showing an entry name (itemName) and - if available - a corresponding value (itemPtr)
typedef struct menuItem {
  char itemName[MAX_CHAR_PER_LINE + 1];   // e.g. "Start Position"
  void *itemPtr;                          // points to the corresponding variable (e.g. &eepromData.stepperStartPosMicrometer)
  bool isReadOnly;                        // if true, the corresponding value (if there is one) can't be selected
  bool isCallingFunction;                 // if true, "EXEC" is shown instead of a value. The function is called by selecting "EXEC" and pressing the enter button.
  bool isMinutes;                         // if true, the value will be increased(decreased) by increments of 60 instead of 1
  bool isSeconds;
} menuItem;

// each dimension of the array represents a menu level
// to get an overview of the menu structure refer to setupMenu()
menuItem menu[MAX_ITEMS_LEVEL_1 + 1][MAX_ITEMS_LEVEL_2 + 1][MAX_ITEMS_LEVEL_3 + 1] = {0};

// cursor in main menu is menuCursor[1]
// used in handleButtonPressEvent(), updateInitActualPosition(), setDisplayFrameBoundaries() and displayMenu()
int menuCursor[NUMBER_OF_MENU_LEVELS + 1] = {0, 1, 1, 1};

// used in handleButtonPressEvent() and displayMenu()
int level = 1;

// one frame consist of four lines on the OLED display
// used in handleButtonPressEvent(), setDisplayFrameBoundaries() and displayMenu()
int lowerFrameBound[3] = {1, 1, 1};
int upperFrameBound[3] = {4, 4, 4};

// true if a button press is detected
// used in interruptButtonPressDetection(), handleButtonPressEvent() and handleButtonLongPressEvent()
volatile bool button1PressEvent = false;
volatile bool button2PressEvent = false;
volatile bool button3PressEvent = false;
volatile bool button4PressEvent = false;
volatile bool button2LongPressEvent = false;
volatile bool button3LongPressEvent = false;

// used in interruptButtonPressDetection()
volatile bool button1State = LOW;
volatile bool button2State = LOW;
volatile bool button3State = LOW;
volatile bool button4State = LOW;
volatile bool button1LastReading = HIGH;  // starting in released state
volatile bool button2LastReading = HIGH;
volatile bool button3LastReading = HIGH;
volatile bool button4LastReading = HIGH;
volatile int button1SameReadingCount = 0; // how many times consecutively the same reading
volatile int button2SameReadingCount = 0;
volatile int button3SameReadingCount = 0;
volatile int button4SameReadingCount = 0;

// if menu.itemPtr points to one of these variables, another press of the enter button starts the related function
// used in handleButtonPressEvent() and setupMenu()
bool startProgramFlag = 0;
bool stopProgramFlag = 0;
bool startInitializationFlag = 0;
bool stopInitializationFlag = 0;
bool eepromReadFlag = 0;
bool eepromUpdateFlag = 0;

// three modes of stepper movement: Initialization, Running to Start Position and Program Run
bool isProgramRunning = false;
bool isInitializationRunning = false;
bool isRunningToStartPos = false;

bool isValueSelected = false; // used in handleButtonPressEvent(), updateInitActualPosition() and displayMenu()

// timing of the grinding process
unsigned long startTime = 0;  // used in calculateRemainingTime(), startProgram(), setDcTargets()
int elapsedTime = 0;          // used in calculateRemainingTime(), stopProgram()
int remainingTime = 0;        // used in calculateRemainingTime(), setupMenu(), stopProgramIfTimeIsUp()
int totalDuration = 0;        // used in calculateRemainingTime(), setupMenu()

// structure for saving/loading data to/from eeprom
// values in micrometer are for the user, values in steps are for the stepper driver
typedef struct savedData {
  int stepperMicrometerPerHour;
  int stepperStartPosMicrometer;
  int stepperEndPosMicrometer;
  int timeAfterReachingEndPos;
  
  // after loading/before saving, the values are copied to/from corresponding variables (refer to line 185 ff.)
  int8_t dcDutyCycle[NUMBER_OF_DC_MOTORS + 1][NUMBER_OF_PROGRAMS + 1][NUMBER_OF_PROGRAM_SEGMENTS + 1];
  uint16_t dcTimeslot[NUMBER_OF_DC_MOTORS + 1][NUMBER_OF_PROGRAMS + 1][NUMBER_OF_PROGRAM_SEGMENTS + 1];
  int program;
  int stepper1CurrentPosSteps;
  int stepper2CurrentPosSteps;
  int stepper3CurrentPosSteps;
} savedData;

savedData eepromData = {0};

int8_t dcDutyCycle[NUMBER_OF_DC_MOTORS + 1][NUMBER_OF_PROGRAMS + 1][NUMBER_OF_PROGRAM_SEGMENTS + 1];
uint16_t dcTimeslot[NUMBER_OF_DC_MOTORS + 1][NUMBER_OF_PROGRAMS + 1][NUMBER_OF_PROGRAM_SEGMENTS + 1];
int program = 1;
volatile int stepper1CurrentPosSteps = 0;
volatile int stepper2CurrentPosSteps = 0;
volatile int stepper3CurrentPosSteps = 0;
volatile int stepper1TargetPosSteps = 0;
volatile int stepper2TargetPosSteps = 0;
volatile int stepper3TargetPosSteps = 0;
volatile bool interruptState = 0;
volatile bool isStepperBlockedCw = false;   // clockwise
volatile bool isStepperBlockedCcw = false;  // counter-clockwise
int stepperCurrentPosMicrometer = 0;        // used in setupMenu() and updateMenu() only


// "init" variables are used for initialization only
// used in setStepperTargets(), startInitialization() and stopInitialization()
int initStepperCurrentPosMicrometer = 0;

// can be changed and set as current position of stepper motors
// used in setupMenu(), updateInitActualPosition() and stopInitialization()
int initSetActualPosMicrometer = 0;

// used in setupMenu(), setStepperTargets(), startInitialization() and stopInitialization()
int initStepperTargetPosMicrometer = 0; 
int initStepper1TargetPosMicrometer = 0;
int initStepper2TargetPosMicrometer = 0;
int initStepper3TargetPosMicrometer = 0;

int dcCurrentCycle[NUMBER_OF_DC_MOTORS + 1] = {0};  // used in runDcMotors() and stopProgram()
int dcTargetCycle[NUMBER_OF_DC_MOTORS + 1] = {0};   // used in setDcTargets() and runDcMotors()

void setup() {
  ButtonTimerInterrupt.priority(1);
  ButtonTimerInterrupt.begin(interruptButtonPressDetection, 100);
  
  StepperTimerInterrupt.priority(2);
  StepperTimerInterrupt.begin(interruptRunStepperMotors, 1000000);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  
  pinMode(DRIVER1_STEP, OUTPUT);
  pinMode(DRIVER2_STEP, OUTPUT);
  pinMode(DRIVER3_STEP, OUTPUT);
  pinMode(DRIVER123_DIR, OUTPUT);
  
  pinMode(DRIVER4_PWMA, OUTPUT);
  pinMode(DRIVER4_AIN1, OUTPUT);
  pinMode(DRIVER4_AIN2, OUTPUT);
  pinMode(DRIVER4_PWMB, OUTPUT);
  pinMode(DRIVER4_BIN1, OUTPUT);
  pinMode(DRIVER4_BIN2, OUTPUT);
  pinMode(DRIVER5_PWMA, OUTPUT);
  pinMode(DRIVER5_AIN1, OUTPUT);
  pinMode(DRIVER5_AIN2, OUTPUT);
  pinMode(DRIVER5_PWMB, OUTPUT);
  pinMode(DRIVER5_BIN1, OUTPUT);
  pinMode(DRIVER5_BIN2, OUTPUT);

  setupDisplay();
  eepromRead();
  ensureValuesWithinLimits();
  setupMenu();
}

void loop() {
  // interruptButtonPressDetection() detects inputs
  handleButtonLongPressEvent();
  handleButtonPressEvent();
  ensureValuesWithinLimits();

  // three modes of stepper movement: Initialization (Init), Running to Start Position and Program Run
  updateInitActualPosition();
  stopRunToStartPosIfReached();
  setStepperTargets();
  // interruptRunStepperMotors() processes the stepper target positions and moves the stepper motors
  
  setDcTargets();
  runDcMotors();
  
  calculateRemainingTime();
  stopProgramIfTimeIsUp();
    
  updateMenu();
  setDisplayFrameBoundaries();
  displayMenu();
  
  delay(50);
}

void setupDisplay() {
  Serial.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED allocation failed"));
    for (;;);
  }
}

void displayMessage(const char* line1, const char* line2, const char* line3, const char* line4) {
  // the display consists of 4 lines
    
  char str1[MAX_CHAR_PER_LINE + 1];
  char str2[MAX_CHAR_PER_LINE + 1];
  char str3[MAX_CHAR_PER_LINE + 1];
  char str4[MAX_CHAR_PER_LINE + 1];
  
  strncpy(str1, line1, MAX_CHAR_PER_LINE);
  strncpy(str2, line2, MAX_CHAR_PER_LINE);
  strncpy(str3, line3, MAX_CHAR_PER_LINE);
  strncpy(str4, line4, MAX_CHAR_PER_LINE);
  str1[MAX_CHAR_PER_LINE] = '\0';
  str2[MAX_CHAR_PER_LINE] = '\0';
  str3[MAX_CHAR_PER_LINE] = '\0';
  str4[MAX_CHAR_PER_LINE] = '\0';
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  drawCenterString(str1);
  drawCenterString(str2);
  drawCenterString(str3);
  drawCenterString(str4);
  display.display();
  delay(2000);
}

void drawCenterString(const char* str) {
  // draw centered string in current line

  int16_t x;                // current cursor x-coordinate
  int16_t y;                // current cursor y-coordinate
  int16_t stringX;          // boundary x-coordinate, returned by display.getTextBounds()
  int16_t stringY;          // boundary y-coordinate, returned by display.getTextBounds()
  uint16_t stringWidth;     // boundary width, returned by display.getTextBounds()
  uint16_t stringHeight;    // boundary height, returned by display.getTextBounds()
  
  x = 0;
  y = display.getCursorY();                                                            // get current line 
  display.getTextBounds(str, x, y, &stringX, &stringY, &stringWidth, &stringHeight);   // calc width of string
  display.setCursor((SCREEN_WIDTH - stringWidth) / 2, y); 
  display.println(str);
}

void interruptButtonPressDetection() {
  // provides debouncing
  // if button # is pressed, button#PressEvent is set true
  // long press detection (button#LongPressEvent) for button 2 (Up) and button 3 (Down)

  // button 1
  volatile bool button1CurrentReading = digitalRead(BUTTON1);
  if (button1CurrentReading == button1LastReading) {
    button1SameReadingCount++;
  } else {
    button1SameReadingCount = 0;
  }
  button1LastReading = button1CurrentReading;
  if (button1SameReadingCount >= BUTTON_CHANGECOUNT) {
    if (button1CurrentReading == HIGH) {
      button1State = LOW;
    } else {
      if (button1State == LOW) {
        button1PressEvent = true;
      }
      button1State = HIGH;
    }
  }

  // button 2
  volatile bool button2CurrentReading = digitalRead(BUTTON2);
  if (button2CurrentReading == button2LastReading) {
    button2SameReadingCount++;
  } else {
    button2SameReadingCount = 0;
  }
  button2LastReading = button2CurrentReading;

  if (button2SameReadingCount >= BUTTON_CHANGECOUNT && button2SameReadingCount <= 5000) {
    if (button2CurrentReading == HIGH) {
      button2State = LOW;
    } else {
      if (button2State == LOW) {
        button2PressEvent = true;
      }
      button2State = HIGH;
    }
  } else {
    if (button2SameReadingCount > 5000 && button2SameReadingCount <= 80000 && button2CurrentReading == LOW) {
      button2PressEvent = true;
    } else {
      if (button2SameReadingCount > 80000 && button2CurrentReading == LOW) {
        button2LongPressEvent = true;
      } else {
        button2LongPressEvent = false;
      }
    }
  }

  // button 3
  volatile bool button3CurrentReading = digitalRead(BUTTON3);
  if (button3CurrentReading == button3LastReading) {
    button3SameReadingCount++;
  } else {
    button3SameReadingCount = 0;
  }
  button3LastReading = button3CurrentReading;

  if (button3SameReadingCount >= BUTTON_CHANGECOUNT && button3SameReadingCount <= 5000) {
    if (button3CurrentReading == HIGH) {
      button3State = LOW;
    } else {
      if (button3State == LOW) {
        button3PressEvent = true;
      }
      button3State = HIGH;
    }
  } else {
    if (button3SameReadingCount > 5000 && button3SameReadingCount <= 80000 && button3CurrentReading == LOW) {
      button3PressEvent = true;
    } else {
      if (button3SameReadingCount > 80000 && button3CurrentReading == LOW) {
        button3LongPressEvent = true;
      } else {
        button3LongPressEvent = false;
      }
    }
  }

  // button 4
  volatile bool button4CurrentReading = digitalRead(BUTTON4);
  if (button4CurrentReading == button4LastReading) {
    button4SameReadingCount++;
  } else {
    button4SameReadingCount = 0;
  }
  button4LastReading = button4CurrentReading;

  if (button4SameReadingCount >= BUTTON_CHANGECOUNT) {
    if (button4CurrentReading == HIGH) {
      button4State = LOW;
    } else {
      if (button4State == LOW) {
        button4PressEvent = true;
      }
      button4State = HIGH;
    }
  }
}

void handleButtonLongPressEvent() {
  // only for button 2 (Up) and button 3 (Down)
  
  if (button2LongPressEvent == true) {
    for (int i = 0; i <= 5; i++) {
      button2PressEvent = true;
      handleButtonPressEvent();
    }
  }
  if (button3LongPressEvent == true) {
    for (int i = 0; i <= 5; i++) {
      button3PressEvent = true;
      handleButtonPressEvent();
    }
  }
}

void handleButtonPressEvent() {
  // checks if button#PressEvent is set true by interruptButtonPressDetection()

  // BUTTON 1 and BUTTON 4
  // quick start/stop of the program
  if (button1PressEvent == true && button4PressEvent == true) {
    if (isProgramRunning == true) {
      stopProgram();
    } else {
      startProgram();
    }
  }

  // button 1 (Enter)
  // if "EXEC" is selected by the user in the display menu, another button 1 press starts the corresponding function
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &eepromReadFlag) {
    eepromRead();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &eepromUpdateFlag) {
    eepromUpdate();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &startProgramFlag) {
    startProgram();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &stopProgramFlag) {
    stopProgram();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &startInitializationFlag) {
    startInitialization();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &stopInitializationFlag) {
    stopInitialization();
    button4PressEvent = true;
  }
  if (button1PressEvent == true && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &isRunningToStartPos) {
    runToStartPos();
    button4PressEvent = true;
  }

  // enter submenu or select value of corresponding menu item (if available)
  if (button1PressEvent == 1) {
    if ((level <= 1 && menu[menuCursor[1]][1][0].itemName != '\0') ||
        (level <= 2 && menu[menuCursor[1]][menuCursor[2]][1].itemName[0] != '\0')) {
      level++;
    } else {
      if ((level == 1 && menu[menuCursor[1]][0][0].isReadOnly  == false) ||
          (level == 2 && menu[menuCursor[1]][menuCursor[2]][0].isReadOnly == false) ||
          (level == 3 && menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].isReadOnly == false)) {
        isValueSelected = true;
      }
    }
    button1PressEvent = 0;
  }

  // button 2 (Up)
  // move cursor up or increase selected value
  if (button2PressEvent == 1) {
    if (isValueSelected == false) {
      if (menuCursor[level] >= 2) {
        menuCursor[level]--;
      }
    } else {
      switch (level) {
        case 1:
          *(int*)menu[menuCursor[1]][0][0].itemPtr += 1;
          break;
        case 2:
          if (menu[menuCursor[1]][menuCursor[2]][0].isMinutes == true) {
            *(int*)menu[menuCursor[1]][menuCursor[2]][0].itemPtr += 60;   // increase by 60 seconds
          } else {
            *(int*)menu[menuCursor[1]][menuCursor[2]][0].itemPtr += 1;
          }
          break;
        case 3:
          if (menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].isMinutes == true) {
            *(int*)menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].itemPtr += 60;
          } else {
            *(int*)menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].itemPtr += 1;
          }
          break;
      }
    }
    button2PressEvent = 0;
  }

  // button 3 (Down)
  // move cursor down or decrease selected value
  if (button3PressEvent == 1) {
    if (isValueSelected == false) {
      if (level == 1 && menuCursor[1] <= (MAX_ITEMS_LEVEL_1 - 1) && menu[menuCursor[1] + 1][0][0].itemName[0] != '\0') {
        menuCursor[level]++;
      }
      if (level == 2 && menuCursor[2] <= (MAX_ITEMS_LEVEL_2 - 1) && menu[menuCursor[1]][menuCursor[2] + 1][0].itemName[0] != '\0') {
        menuCursor[level]++;
      }
      if (level == 3 && menuCursor[3] <= (MAX_ITEMS_LEVEL_3 - 1) && menu[menuCursor[1]][menuCursor[2]][menuCursor[3] + 1].itemName[0] != '\0') {
        menuCursor[level]++;
      }
    } else {
      switch (level) {
        case 1:
          *(int*)menu[menuCursor[1]][0][0].itemPtr -= 1;
          break;
        case 2:
          if (menu[menuCursor[1]][menuCursor[2]][0].isMinutes == true) {
            *(int*)menu[menuCursor[1]][menuCursor[2]][0].itemPtr -= 60;   // decrease by 60 seconds
          } else {
            *(int*)menu[menuCursor[1]][menuCursor[2]][0].itemPtr -= 1;
          }
          break;
        case 3:
          if (menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].isMinutes == true) {
            *(int*)menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].itemPtr -= 60;
          } else {
            *(int*)menu[menuCursor[1]][menuCursor[2]][menuCursor[3]].itemPtr -= 1;
          }
          break;
      }
    }
    button3PressEvent = 0;
  }

  // button 4 (Back)
  // leave submenu or deselect value
  if (button4PressEvent == 1) {
    if (isValueSelected == false) {
      menuCursor[level] = 1;
      lowerFrameBound[level] = 1;
      upperFrameBound[level] = 4;
      if (level >= 2) {
        level--;
      } else {
        level = 1;
      }
    }
    isValueSelected = false;
    button4PressEvent = 0;
  }
}

void ensureValuesWithinLimits() {
  if (isInitializationRunning == true) {
    program = 0;
  } else {
    if (program < 1) {
      program = 1;
    }
  }
  if (program > NUMBER_OF_PROGRAMS) {
    program = NUMBER_OF_PROGRAMS;
  }
  for (int i = 1; i <= NUMBER_OF_DC_MOTORS; i++) {
    for (int k = 1; k <= NUMBER_OF_PROGRAMS; k++) {
      for (int m = 1; m <= NUMBER_OF_PROGRAM_SEGMENTS; m++) {
        if (dcDutyCycle[i][k][m] > 100) {
          dcDutyCycle[i][k][m] = 100;
        }
        if (dcDutyCycle[i][k][m] < -100) {
          dcDutyCycle[i][k][m] = -100;
        }
        if (dcTimeslot[i][k][m] < 0) {
          dcTimeslot[i][k][m] = 0;
        }
        if (dcTimeslot[i][k][m] > 599) {
          dcTimeslot[i][k][m] = 599;
        }
      }
    }
  }
  if (eepromData.stepperMicrometerPerHour < 1) {
    eepromData.stepperMicrometerPerHour = 1;
  }
  if (eepromData.timeAfterReachingEndPos > 32766) {
    eepromData.timeAfterReachingEndPos = 32766;
  }
  if (eepromData.timeAfterReachingEndPos < 0) {
    eepromData.timeAfterReachingEndPos = 0;
  }
  if (eepromData.stepperStartPosMicrometer < 0) {
    eepromData.stepperStartPosMicrometer = 0;
  }
  if (eepromData.stepperEndPosMicrometer > eepromData.stepperStartPosMicrometer) {
    eepromData.stepperEndPosMicrometer = eepromData.stepperStartPosMicrometer;
  }
  if (eepromData.stepperEndPosMicrometer < 0) {
    eepromData.stepperEndPosMicrometer = 0;
  }
}

void  updateInitActualPosition() {
  // updates initSetActualPosMicrometer to the current position if not in initialization mode
  // shows error message if trying to set actual position before starting initialization mode
  
  if (isInitializationRunning == false) {
    initSetActualPosMicrometer = stepper1CurrentPosSteps / STEPS_PER_MICROMETER;
    if ((button2PressEvent == true || button3PressEvent == true) && isValueSelected == true && menu[menuCursor[1]][menuCursor[2]][0].itemPtr == &initSetActualPosMicrometer) {
      button2PressEvent = false;
      button3PressEvent = false;
      displayMessage("", "Initialization", "not started yet!", "");
    }
  }
}

void stopRunToStartPosIfReached() {
  if (isRunningToStartPos == true &&
  stepper1CurrentPosSteps == stepper1TargetPosSteps &&
  stepper2CurrentPosSteps == stepper2TargetPosSteps &&
  stepper3CurrentPosSteps == stepper3TargetPosSteps) {
    isRunningToStartPos = false;
    displayMessage("", "Starting point", "reached", "");
  } 
}

void setStepperTargets() {
  // sets the target position for each stepper motor

  // initialization
  if (isInitializationRunning == true) {
    StepperTimerInterrupt.update(1000000 / 2 / MAX_STEPS_PER_SECOND);
    while (initStepperCurrentPosMicrometer < initStepperTargetPosMicrometer) {
      initStepper1TargetPosMicrometer++;
      initStepper2TargetPosMicrometer++;
      initStepper3TargetPosMicrometer++;
      initStepperCurrentPosMicrometer++;
    }
    while (initStepperCurrentPosMicrometer > initStepperTargetPosMicrometer) {
      initStepper1TargetPosMicrometer--;
      initStepper2TargetPosMicrometer--;
      initStepper3TargetPosMicrometer--;
      initStepperCurrentPosMicrometer--;
    }
    stepper1TargetPosSteps = initStepper1TargetPosMicrometer * STEPS_PER_MICROMETER;
    stepper2TargetPosSteps = initStepper2TargetPosMicrometer * STEPS_PER_MICROMETER;
    stepper3TargetPosSteps = initStepper3TargetPosMicrometer * STEPS_PER_MICROMETER;
  }

  // running to start position
  if (isRunningToStartPos == true) {
    StepperTimerInterrupt.update(1000000 / 2 / MAX_STEPS_PER_SECOND);
    stepper1TargetPosSteps = eepromData.stepperStartPosMicrometer * STEPS_PER_MICROMETER;
    stepper2TargetPosSteps = eepromData.stepperStartPosMicrometer * STEPS_PER_MICROMETER;
    stepper3TargetPosSteps = eepromData.stepperStartPosMicrometer * STEPS_PER_MICROMETER;
  }

  // program run
  if (isProgramRunning == true) {
    StepperTimerInterrupt.update(1000000 / 2 * 60 * 60 / eepromData.stepperMicrometerPerHour / STEPS_PER_MICROMETER);
    stepper1TargetPosSteps = eepromData.stepperEndPosMicrometer * STEPS_PER_MICROMETER;
    stepper2TargetPosSteps = eepromData.stepperEndPosMicrometer * STEPS_PER_MICROMETER;
    stepper3TargetPosSteps = eepromData.stepperEndPosMicrometer * STEPS_PER_MICROMETER;
  }
}

void interruptRunStepperMotors() {
  // move the corresponding stepper motor, if there is a difference between target and current position
  // each stepper motor is controlled seperately to provide leveling during initialization
  // direction block is necessary, because all stepper drivers share one direction pin
  
  interruptState = !interruptState; // for a single step of a motor, the corresponding step pin has to be set HIGH and thereafter LOW again
  isStepperBlockedCcw = false; 
  if (isProgramRunning == true || isRunningToStartPos == true || isInitializationRunning == true) {
    if (isStepperBlockedCw == false) {
      if (stepper1CurrentPosSteps < stepper1TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, HIGH);
        digitalWrite(DRIVER1_STEP, interruptState);
        stepper1CurrentPosSteps += interruptState; // increases if DRIVER1_STEP pin is set to 1 respectively HIGH
        isStepperBlockedCcw = true;  // assures that the stepper motors don't move in different directions at the same time
      }
      if (stepper2CurrentPosSteps < stepper2TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, HIGH);
        digitalWrite(DRIVER2_STEP, interruptState);
        stepper2CurrentPosSteps += interruptState;
        isStepperBlockedCcw = true;
      }
      if (stepper3CurrentPosSteps < stepper3TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, HIGH);
        digitalWrite(DRIVER3_STEP, interruptState);
        stepper3CurrentPosSteps += interruptState;
        isStepperBlockedCcw = true;
      }
    }

    isStepperBlockedCw = false;
    if (isStepperBlockedCcw == false) {
      if (stepper1CurrentPosSteps > stepper1TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, LOW);
        digitalWrite(DRIVER1_STEP, interruptState);
        stepper1CurrentPosSteps -= interruptState;
        isStepperBlockedCw = true;
      }
      if (stepper2CurrentPosSteps > stepper2TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, LOW);
        digitalWrite(DRIVER2_STEP, interruptState);
        stepper2CurrentPosSteps -= interruptState;
        isStepperBlockedCw = true;
      }
      if (stepper3CurrentPosSteps > stepper3TargetPosSteps) {
        digitalWrite(DRIVER123_DIR, LOW);
        digitalWrite(DRIVER3_STEP, interruptState);
        stepper3CurrentPosSteps -= interruptState;
        isStepperBlockedCw = true;
      }
    }
  }
}

void setDcTargets() {
  // sets the target duty cycles for each DC motor during the related timeslots
  // after reaching the end of the last segment, it starts again with the first segment
  
  int totalTimeslot = 0;
  int dcTimer[NUMBER_OF_DC_MOTORS + 1] = {0};
  if (isProgramRunning == true && isInitializationRunning == false) {
    for (int i = 1; i <= NUMBER_OF_DC_MOTORS; i++) {
      for (int k = 1; k <= NUMBER_OF_PROGRAM_SEGMENTS; k++) {
        totalTimeslot += dcTimeslot[i][program][k];
      }
      dcTimer[i] = ((millis() - startTime) / 1000) % totalTimeslot;
      totalTimeslot = 0;
    }
    for (int i = 1; i <= NUMBER_OF_DC_MOTORS; i++) {
      if (dcTimer[i] <= dcTimeslot[i][program][1]) {
        dcTargetCycle[i] = dcDutyCycle[i][program][1]; // segment A
      }
      if (dcTimer[i] > dcTimeslot[i][program][1] &&
          dcTimer[i] <= (dcTimeslot[i][program][1] + dcTimeslot[i][program][2])) {
        dcTargetCycle[i] = dcDutyCycle[i][program][2]; // segment B
      }
      if (dcTimer[i] > (dcTimeslot[i][program][1] + dcTimeslot[i][program][2]) &&
          dcTimer[i] <= (dcTimeslot[i][program][1] + dcTimeslot[i][program][2] + dcTimeslot[i][program][3])) {
        dcTargetCycle[i] = dcDutyCycle[i][program][3]; // segment C
      }
      if (dcTimer[i] > (dcTimeslot[i][program][1] + dcTimeslot[i][program][2] + dcTimeslot[i][program][3]) &&
          dcTimer[i] <= (dcTimeslot[i][program][1] + dcTimeslot[i][program][2] + dcTimeslot[i][program][3] + dcTimeslot[i][program][4])) {
        dcTargetCycle[i] = dcDutyCycle[i][program][4]; // segment D
      }
      if (dcTimer[i] > (dcTimeslot[i][program][1] + dcTimeslot[i][program][2] + dcTimeslot[i][program][3] + dcTimeslot[i][program][4])) {
        dcTargetCycle[i] = dcDutyCycle[i][program][5]; // segment E
      }
    }
  }
}

void runDcMotors() {
  if (isProgramRunning == true) {
    if (dcCurrentCycle[1] >= 0) {
      digitalWrite(DRIVER5_AIN1, HIGH);
      digitalWrite(DRIVER5_AIN2, LOW);
    } else {
      digitalWrite(DRIVER5_AIN1, LOW);
      digitalWrite(DRIVER5_AIN2, HIGH);
    }
    if (dcCurrentCycle[2] >= 0) {
      digitalWrite(DRIVER5_BIN1, HIGH);
      digitalWrite(DRIVER5_BIN2, LOW);
    } else {
      digitalWrite(DRIVER5_BIN1, LOW);
      digitalWrite(DRIVER5_BIN2, HIGH);
    }
    if (dcCurrentCycle[3] >= 0) {
      digitalWrite(DRIVER4_AIN1, HIGH);
      digitalWrite(DRIVER4_AIN2, LOW);
    } else {
      digitalWrite(DRIVER4_AIN1, LOW);
      digitalWrite(DRIVER4_AIN2, HIGH);
    }
    if (dcCurrentCycle[4] >= 0) {
      digitalWrite(DRIVER4_BIN1, HIGH);
      digitalWrite(DRIVER4_BIN2, LOW);
    } else {
      digitalWrite(DRIVER4_BIN1, LOW);
      digitalWrite(DRIVER4_BIN2, HIGH);
    }
    if (dcCurrentCycle[1] < dcTargetCycle[1]) {
      dcCurrentCycle[1]++;
      analogWrite(DRIVER5_PWMA, abs(dcCurrentCycle[1]) * 255 / 100);
    }
    if (dcCurrentCycle[1] > dcTargetCycle[1]) {
      dcCurrentCycle[1]--;
      analogWrite(DRIVER5_PWMA, abs(dcCurrentCycle[1]) * 255 / 100);
    }
    if (dcCurrentCycle[2] < dcTargetCycle[2]) {
      dcCurrentCycle[2]++;
      analogWrite(DRIVER5_PWMB, abs(dcCurrentCycle[2]) * 255 / 100);
    }
    if (dcCurrentCycle[2] > dcTargetCycle[2]) {
      dcCurrentCycle[2]--;
      analogWrite(DRIVER5_PWMB, abs(dcCurrentCycle[2]) * 255 / 100);
    }
    if (dcCurrentCycle[3] < dcTargetCycle[3]) {
      dcCurrentCycle[3]++;
      analogWrite(DRIVER4_PWMA, abs(dcCurrentCycle[3]) * 255 / 100);
    }
    if (dcCurrentCycle[3] > dcTargetCycle[3]) {
      dcCurrentCycle[3]--;
      analogWrite(DRIVER4_PWMA, abs(dcCurrentCycle[3]) * 255 / 100);
    }
    if (dcCurrentCycle[4] < dcTargetCycle[4]) {
      dcCurrentCycle[4]++;
      analogWrite(DRIVER4_PWMB, abs(dcCurrentCycle[4]) * 255 / 100);
    }
    if (dcCurrentCycle[4] > dcTargetCycle[4]) {
      dcCurrentCycle[4]--;
      analogWrite(DRIVER4_PWMB, abs(dcCurrentCycle[4]) * 255 / 100);
    }
  }
}

void calculateRemainingTime() {
  int distance = eepromData.stepperStartPosMicrometer - eepromData.stepperEndPosMicrometer;
  totalDuration = (distance * 3600 / eepromData.stepperMicrometerPerHour) + eepromData.timeAfterReachingEndPos;
  if (isProgramRunning == true) {
    elapsedTime = (millis() - startTime) / 1000;
  }
  remainingTime = totalDuration - elapsedTime;
}

void stopProgramIfTimeIsUp() {
  if (isProgramRunning == true && remainingTime <= 0) {
    stopProgram();
  }
}

void  updateMenu() {
  // update Actual Position, DC duty cycle and DC timeslot in the "Start/Stop Grinding" menu
  
  stepperCurrentPosMicrometer = stepper1CurrentPosSteps / STEPS_PER_MICROMETER;
  
  menu[3][15][0].itemPtr = &dcDutyCycle[1][program][1];
  menu[3][16][0].itemPtr = &dcTimeslot[1][program][1];
  menu[3][17][0].itemPtr = &dcDutyCycle[1][program][2];
  menu[3][18][0].itemPtr = &dcTimeslot[1][program][2];
  menu[3][19][0].itemPtr = &dcDutyCycle[1][program][3];
  menu[3][20][0].itemPtr = &dcTimeslot[1][program][3];
  menu[3][21][0].itemPtr = &dcDutyCycle[1][program][4];
  menu[3][22][0].itemPtr = &dcTimeslot[1][program][4];
  menu[3][23][0].itemPtr = &dcDutyCycle[1][program][5];
  menu[3][24][0].itemPtr = &dcTimeslot[1][program][5];
  
  menu[3][26][0].itemPtr = &dcDutyCycle[2][program][1];
  menu[3][27][0].itemPtr = &dcTimeslot[2][program][1];
  menu[3][28][0].itemPtr = &dcDutyCycle[2][program][2];
  menu[3][29][0].itemPtr = &dcTimeslot[2][program][2];
  menu[3][30][0].itemPtr = &dcDutyCycle[2][program][3];
  menu[3][31][0].itemPtr = &dcTimeslot[2][program][3];
  menu[3][32][0].itemPtr = &dcDutyCycle[2][program][4];
  menu[3][33][0].itemPtr = &dcTimeslot[2][program][4];
  menu[3][34][0].itemPtr = &dcDutyCycle[2][program][5];
  menu[3][35][0].itemPtr = &dcTimeslot[2][program][5];
  
  menu[3][37][0].itemPtr = &dcDutyCycle[3][program][1];
  menu[3][38][0].itemPtr = &dcTimeslot[3][program][1];
  menu[3][39][0].itemPtr = &dcDutyCycle[3][program][2];
  menu[3][40][0].itemPtr = &dcTimeslot[3][program][2];
  menu[3][41][0].itemPtr = &dcDutyCycle[3][program][3];
  menu[3][42][0].itemPtr = &dcTimeslot[3][program][3];
  menu[3][43][0].itemPtr = &dcDutyCycle[3][program][4];
  menu[3][44][0].itemPtr = &dcTimeslot[3][program][4];
  menu[3][45][0].itemPtr = &dcDutyCycle[3][program][5];
  menu[3][46][0].itemPtr = &dcTimeslot[3][program][5];
  
  menu[3][48][0].itemPtr = &dcDutyCycle[4][program][1];
  menu[3][49][0].itemPtr = &dcTimeslot[4][program][1];
  menu[3][50][0].itemPtr = &dcDutyCycle[4][program][2];
  menu[3][51][0].itemPtr = &dcTimeslot[4][program][2];
  menu[3][52][0].itemPtr = &dcDutyCycle[4][program][3];
  menu[3][53][0].itemPtr = &dcTimeslot[4][program][3];
  menu[3][54][0].itemPtr = &dcDutyCycle[4][program][4];
  menu[3][55][0].itemPtr = &dcTimeslot[4][program][4];
  menu[3][56][0].itemPtr = &dcDutyCycle[4][program][5];
  menu[3][57][0].itemPtr = &dcTimeslot[4][program][5];
}

void setDisplayFrameBoundaries() {
  // one frame consists of four lines on the display
  
  if (menuCursor[level] < lowerFrameBound[level]) {
    lowerFrameBound[level] = (menuCursor[level]);
    upperFrameBound[level] = (menuCursor[level] + 3);
  }
  if (menuCursor[level] > upperFrameBound[level]) {
    lowerFrameBound[level] = (menuCursor[level]) - 3;
    upperFrameBound[level] = (menuCursor[level]);
  }
}

void displayMenu() {
  int x = menuCursor[1];
  int y = menuCursor[2];
  int z = menuCursor[3];
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  for (int i = lowerFrameBound[level]; i <= upperFrameBound[level]; i++) {
    if (level == 1) {
      x = i;
      y = 0;
      z = 0;
    }
    if (level == 2) {
      y = i;
      z = 0;
    }
    if (level == 3) {
      z = i;
    }
    if (menu[x][y][z].itemName != 0) {
      if (menuCursor[level] == i) {
        display.print("> "); // selected item
      } else {
        display.print("  "); // non-selected item
      }
      display.print(menu[x][y][z].itemName);
      if (menu[x][y][z].itemPtr != 0) {
        if (isValueSelected == true && menuCursor[level] == i && menu[x][y][z].isReadOnly == false) {
          display.print(":>"); // selected value
        } else {
          display.print(": "); // non-selected value
        }
        if (menu[x][y][z].isCallingFunction == true) {
          display.println("EXEC");
        } else {
          if (menu[x][y][z].isMinutes == true) {
            int hours = *(int*)menu[x][y][z].itemPtr / 3600;
            int minutes = *(int*)menu[x][y][z].itemPtr / 60;
            display.print(hours);
            display.print(":");
            display.print((minutes - hours * 60 - ((minutes - hours * 60) % 10)) / 10);
            display.print((minutes - hours * 60) % 10);
            display.println(" h");
          } else {
            if (menu[x][y][z].isSeconds == true) {
              int minutes = *(int*)menu[x][y][z].itemPtr / 60;
              int seconds = *(int*)menu[x][y][z].itemPtr;
              display.print(minutes);
              display.print(":");
              display.print((seconds - minutes * 60 - ((seconds - minutes * 60) % 10)) / 10);
              display.print((seconds - minutes * 60) % 10);
              display.println(" m");
            } else {
              display.println(*(int*)menu[x][y][z].itemPtr);
            }
          }
        }
      } else {
        display.println();
      }
    }
  }
  display.display();
}

void runToStartPos() {
  if (isInitializationRunning == true) {
    displayMessage("not possible", "due to running", "initialization", "");
  } else {
    if (isProgramRunning == true) {
      displayMessage("", "not possible due", "to running program", "");
    } else {
      isRunningToStartPos = true;
      displayMessage("", "Running to", "starting point", "");
    }
  }
}

void startInitialization() {
  if (isProgramRunning == true) {
    displayMessage("", "Not possible due", "to running program!", "");
  } else {
    if (isRunningToStartPos == true) {
      displayMessage("", "Still running to", "starting point!", "");
    } else {
      if (isInitializationRunning == true) {
        displayMessage("", "Initialization", "already running!", "");
      } else {
        stepper1CurrentPosSteps = 0;
        stepper2CurrentPosSteps = 0;
        stepper3CurrentPosSteps = 0;
        stepper1TargetPosSteps = 0;
        stepper2TargetPosSteps = 0;
        stepper3TargetPosSteps = 0;
        initStepperCurrentPosMicrometer = 0;
        initStepperTargetPosMicrometer = 0;
        initStepper1TargetPosMicrometer = 0;
        initStepper2TargetPosMicrometer = 0;
        initStepper3TargetPosMicrometer = 0;
        isInitializationRunning = true;
        program = 0;
        displayMessage("", "Starting", "Initialization", "");
      }
    }
  }
}

void stopInitialization() {
  if (isInitializationRunning == false) {
    displayMessage("", "Initialization", "not started yet!", "");
  } else {
    program = 1;
    isInitializationRunning = false;
    initStepperCurrentPosMicrometer = 0;
    initStepperTargetPosMicrometer = 0;
    initStepper1TargetPosMicrometer = 0;
    initStepper2TargetPosMicrometer = 0;
    initStepper3TargetPosMicrometer = 0;
    stepper1CurrentPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    stepper2CurrentPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    stepper3CurrentPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    stepper1TargetPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    stepper2TargetPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    stepper3TargetPosSteps = initSetActualPosMicrometer * STEPS_PER_MICROMETER;
    displayMessage("", "Initialization", "finished", "");
  }
}

void startProgram() {
  if (isInitializationRunning == true) {
    displayMessage("Not possible", "due to running", "initialization!", "");
  } else {
    if (isProgramRunning == true) {
      displayMessage("", "Program", "already running!", "");
    } else {
      if (isRunningToStartPos == true) {
        displayMessage("", "Still running to", "starting point!", "");
      } else {
        isProgramRunning = true;
        startTime = millis();
        displayMessage("", "Starting", "Program", "");
      }
    }
  }
}

void stopProgram() {
  if (isProgramRunning == false) {
    displayMessage("", "Program not", "started yet", "");
  } else {
    dcCurrentCycle[1] = 0;
    dcCurrentCycle[2] = 0;
    dcCurrentCycle[3] = 0;
    dcCurrentCycle[4] = 0;
    digitalWrite(DRIVER4_AIN1, LOW);
    digitalWrite(DRIVER4_AIN2, LOW);
    digitalWrite(DRIVER4_BIN1, LOW);
    digitalWrite(DRIVER4_BIN2, LOW);
    digitalWrite(DRIVER5_AIN1, LOW);
    digitalWrite(DRIVER5_AIN2, LOW);
    digitalWrite(DRIVER5_BIN1, LOW);
    digitalWrite(DRIVER5_BIN2, LOW);
    elapsedTime = 0;
    isProgramRunning = false;
    isRunningToStartPos = false;
    displayMessage("", "Program", "stopped", "");
  }
}

void eepromRead() {
  while (eeprom_is_ready() != 1) {
    displayMessage("", "Waiting for", "EEPROM", "");
  }
  EEPROM.get(1, eepromData);
  program = eepromData.program;
  stepper1CurrentPosSteps = eepromData.stepper1CurrentPosSteps;
  stepper2CurrentPosSteps = eepromData.stepper2CurrentPosSteps;
  stepper3CurrentPosSteps = eepromData.stepper3CurrentPosSteps;
  for (int i = 0; i <= NUMBER_OF_DC_MOTORS; i++) {
    for (int k = 0; k <= NUMBER_OF_PROGRAMS; k++) {
      for (int m = 0; m <= NUMBER_OF_PROGRAM_SEGMENTS; m++) {
        dcDutyCycle[i][k][m] = eepromData.dcDutyCycle[i][k][m];
        dcTimeslot[i][k][m] = eepromData.dcTimeslot[i][k][m];
      }
    }
  }
  ensureValuesWithinLimits();
  displayMessage("", "Loading", "successful", "");
}

void eepromUpdate() {
  while (eeprom_is_ready() != 1) {
    displayMessage("", "Waiting for", "EEPROM", "");
  }
  ensureValuesWithinLimits();
  eepromData.program = program;
  eepromData.stepper1CurrentPosSteps = stepper1CurrentPosSteps;
  eepromData.stepper2CurrentPosSteps = stepper2CurrentPosSteps;
  eepromData.stepper3CurrentPosSteps = stepper3CurrentPosSteps;  
  for (int i = 0; i <= NUMBER_OF_DC_MOTORS; i++) {
    for (int k = 0; k <= NUMBER_OF_PROGRAMS; k++) {
      for (int m = 0; m <= NUMBER_OF_PROGRAM_SEGMENTS; m++) {
        eepromData.dcDutyCycle[i][k][m] = dcDutyCycle[i][k][m];
        eepromData.dcTimeslot[i][k][m] = dcTimeslot[i][k][m];
      }
    }
  }

  EEPROM.put(1, eepromData);
  displayMessage("", "Saving", "successful", "");
}

void  setupMenu() {
  // fill the menu array
  
  // main menu
  strncpy(menu[1][0][0].itemName, "Save/Load Data", MAX_CHAR_PER_LINE);
  strncpy(menu[2][0][0].itemName, "Initialization", MAX_CHAR_PER_LINE);
  strncpy(menu[3][0][0].itemName, "Start/Stop Grinding", MAX_CHAR_PER_LINE);
  strncpy(menu[4][0][0].itemName, "DC Motor Programs", MAX_CHAR_PER_LINE);
  for (int i = 1; i <= 4; i++) {
    menu[i][0][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  }

  // submenu "Save/Load Data"
  strncpy(menu[1][1][0].itemName, "LOAD Data", MAX_CHAR_PER_LINE);
  strncpy(menu[1][2][0].itemName, "SAVE Data", MAX_CHAR_PER_LINE);
  menu[1][1][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  menu[1][2][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  menu[1][1][0].itemPtr = &eepromReadFlag;
  menu[1][2][0].itemPtr = &eepromUpdateFlag;
  menu[1][1][0].isCallingFunction = true;
  menu[1][2][0].isCallingFunction = true;

  // submenu "Initialization"
  strncpy(menu[2][1][0].itemName, "START Init", MAX_CHAR_PER_LINE);
  strncpy(menu[2][2][0].itemName, "FINISH Init", MAX_CHAR_PER_LINE);
  strncpy(menu[2][3][0].itemName, "Set Act Pos", MAX_CHAR_PER_LINE);
  strncpy(menu[2][4][0].itemName, "Move all", MAX_CHAR_PER_LINE);
  strncpy(menu[2][5][0].itemName, "Move Step 1", MAX_CHAR_PER_LINE);
  strncpy(menu[2][6][0].itemName, "Move Step 2", MAX_CHAR_PER_LINE);
  strncpy(menu[2][7][0].itemName, "Move Step 3", MAX_CHAR_PER_LINE);
  for (int i = 1; i <= 7; i++) {
    menu[2][i][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  }
  menu[2][1][0].itemPtr = &startInitializationFlag;
  menu[2][2][0].itemPtr = &stopInitializationFlag;
  menu[2][3][0].itemPtr = &initSetActualPosMicrometer;
  menu[2][4][0].itemPtr = &initStepperTargetPosMicrometer;
  menu[2][5][0].itemPtr = &initStepper1TargetPosMicrometer;
  menu[2][6][0].itemPtr = &initStepper2TargetPosMicrometer;
  menu[2][7][0].itemPtr = &initStepper3TargetPosMicrometer;
  menu[2][1][0].isCallingFunction = true;
  menu[2][2][0].isCallingFunction = true;

  // submenu "Start/Stop Grinding"
  strncpy(menu[3][1][0].itemName, "DC Motor Program", MAX_CHAR_PER_LINE);
  strncpy(menu[3][2][0].itemName, "MOVE to Start", MAX_CHAR_PER_LINE);
  strncpy(menu[3][3][0].itemName, "START Program", MAX_CHAR_PER_LINE);
  strncpy(menu[3][4][0].itemName, "STOP Program ", MAX_CHAR_PER_LINE);
  strncpy(menu[3][5][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][6][0].itemName, "Micrometer/h", MAX_CHAR_PER_LINE);
  strncpy(menu[3][7][0].itemName, "Start Pos", MAX_CHAR_PER_LINE);
  strncpy(menu[3][8][0].itemName, "Tgt Pos", MAX_CHAR_PER_LINE);
  strncpy(menu[3][9][0].itemName, "Time @ Tgt", MAX_CHAR_PER_LINE);
  strncpy(menu[3][10][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][11][0].itemName, "Ttl Time", MAX_CHAR_PER_LINE);
  strncpy(menu[3][12][0].itemName, "Rmg Time", MAX_CHAR_PER_LINE);
  strncpy(menu[3][13][0].itemName, "Act Pos", MAX_CHAR_PER_LINE);
  strncpy(menu[3][14][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][15][0].itemName, "DC 1 Duty A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][16][0].itemName, "DC 1 Time A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][17][0].itemName, "DC 1 Duty B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][18][0].itemName, "DC 1 Time B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][19][0].itemName, "DC 1 Duty C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][20][0].itemName, "DC 1 Time C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][21][0].itemName, "DC 1 Duty D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][22][0].itemName, "DC 1 Time D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][23][0].itemName, "DC 1 Duty E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][24][0].itemName, "DC 1 Time E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][25][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][26][0].itemName, "DC 2 Duty A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][27][0].itemName, "DC 2 Time A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][28][0].itemName, "DC 2 Duty B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][29][0].itemName, "DC 2 Time B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][30][0].itemName, "DC 2 Duty C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][31][0].itemName, "DC 2 Time C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][32][0].itemName, "DC 2 Duty D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][33][0].itemName, "DC 2 Time D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][34][0].itemName, "DC 2 Duty E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][35][0].itemName, "DC 2 Time E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][36][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][37][0].itemName, "DC 3 Duty A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][38][0].itemName, "DC 3 Time A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][39][0].itemName, "DC 3 Duty B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][40][0].itemName, "DC 3 Time B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][41][0].itemName, "DC 3 Duty C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][42][0].itemName, "DC 3 Time C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][43][0].itemName, "DC 3 Duty D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][44][0].itemName, "DC 3 Time D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][45][0].itemName, "DC 3 Duty E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][46][0].itemName, "DC 3 Time E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][47][0].itemName, "-------------------", MAX_CHAR_PER_LINE);
  strncpy(menu[3][48][0].itemName, "DC 4 Duty A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][49][0].itemName, "DC 4 Time A", MAX_CHAR_PER_LINE);
  strncpy(menu[3][50][0].itemName, "DC 4 Duty B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][51][0].itemName, "DC 4 Time B", MAX_CHAR_PER_LINE);
  strncpy(menu[3][52][0].itemName, "DC 4 Duty C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][53][0].itemName, "DC 4 Time C", MAX_CHAR_PER_LINE);
  strncpy(menu[3][54][0].itemName, "DC 4 Duty D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][55][0].itemName, "DC 4 Time D", MAX_CHAR_PER_LINE);
  strncpy(menu[3][56][0].itemName, "DC 4 Duty E", MAX_CHAR_PER_LINE);
  strncpy(menu[3][57][0].itemName, "DC 4 Time E", MAX_CHAR_PER_LINE);
  for (int i = 1; i <= 57; i++) {
    menu[3][i][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  }
  menu[3][15][0].itemPtr = &dcDutyCycle[1][program][1];
  menu[3][16][0].itemPtr = &dcTimeslot[1][program][1];
  menu[3][17][0].itemPtr = &dcDutyCycle[1][program][2];
  menu[3][18][0].itemPtr = &dcTimeslot[1][program][2];
  menu[3][19][0].itemPtr = &dcDutyCycle[1][program][3];
  menu[3][20][0].itemPtr = &dcTimeslot[1][program][3];
  menu[3][21][0].itemPtr = &dcDutyCycle[1][program][4];
  menu[3][22][0].itemPtr = &dcTimeslot[1][program][4];
  menu[3][23][0].itemPtr = &dcDutyCycle[1][program][5];
  menu[3][24][0].itemPtr = &dcTimeslot[1][program][5];
  menu[3][26][0].itemPtr = &dcDutyCycle[2][program][1];
  menu[3][27][0].itemPtr = &dcTimeslot[2][program][1];
  menu[3][28][0].itemPtr = &dcDutyCycle[2][program][2];
  menu[3][29][0].itemPtr = &dcTimeslot[2][program][2];
  menu[3][30][0].itemPtr = &dcDutyCycle[2][program][3];
  menu[3][31][0].itemPtr = &dcTimeslot[2][program][3];
  menu[3][32][0].itemPtr = &dcDutyCycle[2][program][4];
  menu[3][33][0].itemPtr = &dcTimeslot[2][program][4];
  menu[3][34][0].itemPtr = &dcDutyCycle[2][program][5];
  menu[3][35][0].itemPtr = &dcTimeslot[2][program][5];
  menu[3][37][0].itemPtr = &dcDutyCycle[3][program][1];
  menu[3][38][0].itemPtr = &dcTimeslot[3][program][1];
  menu[3][39][0].itemPtr = &dcDutyCycle[3][program][2];
  menu[3][40][0].itemPtr = &dcTimeslot[3][program][2];
  menu[3][41][0].itemPtr = &dcDutyCycle[3][program][3];
  menu[3][42][0].itemPtr = &dcTimeslot[3][program][3];
  menu[3][43][0].itemPtr = &dcDutyCycle[3][program][4];
  menu[3][44][0].itemPtr = &dcTimeslot[3][program][4];
  menu[3][45][0].itemPtr = &dcDutyCycle[3][program][5];
  menu[3][46][0].itemPtr = &dcTimeslot[3][program][5];
  menu[3][48][0].itemPtr = &dcDutyCycle[4][program][1];
  menu[3][49][0].itemPtr = &dcTimeslot[4][program][1];
  menu[3][50][0].itemPtr = &dcDutyCycle[4][program][2];
  menu[3][51][0].itemPtr = &dcTimeslot[4][program][2];
  menu[3][52][0].itemPtr = &dcDutyCycle[4][program][3];
  menu[3][53][0].itemPtr = &dcTimeslot[4][program][3];
  menu[3][54][0].itemPtr = &dcDutyCycle[4][program][4];
  menu[3][55][0].itemPtr = &dcTimeslot[4][program][4];
  menu[3][56][0].itemPtr = &dcDutyCycle[4][program][5];
  menu[3][57][0].itemPtr = &dcTimeslot[4][program][5];
  menu[3][1][0].itemPtr = &program;
  menu[3][2][0].itemPtr = &isRunningToStartPos;
  menu[3][3][0].itemPtr = &startProgramFlag;
  menu[3][4][0].itemPtr = &stopProgramFlag;
  menu[3][6][0].itemPtr = &eepromData.stepperMicrometerPerHour;
  menu[3][7][0].itemPtr = &eepromData.stepperStartPosMicrometer;
  menu[3][8][0].itemPtr = &eepromData.stepperEndPosMicrometer;
  menu[3][9][0].itemPtr = &eepromData.timeAfterReachingEndPos;
  menu[3][11][0].itemPtr = &totalDuration;
  menu[3][12][0].itemPtr = &remainingTime;
  menu[3][13][0].itemPtr = &stepperCurrentPosMicrometer;
  menu[3][5][0].isReadOnly = true;
  menu[3][10][0].isReadOnly = true;
  menu[3][11][0].isReadOnly = true;
  menu[3][12][0].isReadOnly = true;
  menu[3][13][0].isReadOnly = true;
  menu[3][14][0].isReadOnly = true;
  menu[3][9][0].isMinutes = true;
  menu[3][11][0].isMinutes = true;
  menu[3][12][0].isMinutes = true;
  menu[3][16][0].isSeconds = true;
  menu[3][18][0].isSeconds = true;
  menu[3][20][0].isSeconds = true;
  menu[3][22][0].isSeconds = true;
  menu[3][24][0].isSeconds = true;
  menu[3][27][0].isSeconds = true;
  menu[3][29][0].isSeconds = true;
  menu[3][31][0].isSeconds = true;
  menu[3][33][0].isSeconds = true;
  menu[3][35][0].isSeconds = true;
  menu[3][38][0].isSeconds = true;
  menu[3][40][0].isSeconds = true;
  menu[3][42][0].isSeconds = true;
  menu[3][44][0].isSeconds = true;
  menu[3][46][0].isSeconds = true;
  menu[3][49][0].isSeconds = true;
  menu[3][51][0].isSeconds = true;
  menu[3][53][0].isSeconds = true;
  menu[3][55][0].isSeconds = true;
  menu[3][57][0].isSeconds = true;
  menu[3][2][0].isCallingFunction = true;
  menu[3][3][0].isCallingFunction = true;
  menu[3][4][0].isCallingFunction = true;

  // submenu "DC Motor Programs"
  strncpy(menu[4][1][0].itemName, "Program 1", MAX_CHAR_PER_LINE);
  strncpy(menu[4][2][0].itemName, "Program 2", MAX_CHAR_PER_LINE);
  strncpy(menu[4][3][0].itemName, "Program 3", MAX_CHAR_PER_LINE);
  strncpy(menu[4][4][0].itemName, "Program 4", MAX_CHAR_PER_LINE);
  strncpy(menu[4][5][0].itemName, "Program 5", MAX_CHAR_PER_LINE);
  strncpy(menu[4][6][0].itemName, "Program 6", MAX_CHAR_PER_LINE);
  strncpy(menu[4][7][0].itemName, "Program 7", MAX_CHAR_PER_LINE);
  strncpy(menu[4][8][0].itemName, "Program 8", MAX_CHAR_PER_LINE);
  for (int i = 1; i <= 8; i++) {
    menu[4][i][0].itemName[MAX_CHAR_PER_LINE] = '\0';
  }

  // sub-submenu Program 1-8 in DC Motor Programs
  for (int i = 1; i < NUMBER_OF_PROGRAMS + 1; i++) {
    strncpy(menu[4][i][1].itemName, "DC 1 Duty A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][2].itemName, "DC 1 Time A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][3].itemName, "DC 1 Duty B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][4].itemName, "DC 1 Time B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][5].itemName, "DC 1 Duty C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][6].itemName, "DC 1 Time C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][7].itemName, "DC 1 Duty D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][8].itemName, "DC 1 Time D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][9].itemName, "DC 1 Duty E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][10].itemName, "DC 1 Time E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][11].itemName, "-------------------", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][12].itemName, "DC 2 Duty A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][13].itemName, "DC 2 Time A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][14].itemName, "DC 2 Duty B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][15].itemName, "DC 2 Time B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][16].itemName, "DC 2 Duty C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][17].itemName, "DC 2 Time C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][18].itemName, "DC 2 Duty D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][19].itemName, "DC 2 Time D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][20].itemName, "DC 2 Duty E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][21].itemName, "DC 2 Time E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][22].itemName, "-------------------", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][23].itemName, "DC 3 Duty A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][24].itemName, "DC 3 Time A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][25].itemName, "DC 3 Duty B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][26].itemName, "DC 3 Time B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][27].itemName, "DC 3 Duty C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][28].itemName, "DC 3 Time C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][29].itemName, "DC 3 Duty D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][30].itemName, "DC 3 Time D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][31].itemName, "DC 3 Duty E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][32].itemName, "DC 3 Time E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][33].itemName, "-------------------", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][34].itemName, "DC 4 Duty A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][35].itemName, "DC 4 Time A", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][36].itemName, "DC 4 Duty B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][37].itemName, "DC 4 Time B", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][38].itemName, "DC 4 Duty C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][39].itemName, "DC 4 Time C", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][40].itemName, "DC 4 Duty D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][41].itemName, "DC 4 Time D", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][42].itemName, "DC 4 Duty E", MAX_CHAR_PER_LINE);
    strncpy(menu[4][i][43].itemName, "DC 4 Time E", MAX_CHAR_PER_LINE);
    for (int k = 1; k <= 43; k++) {
      menu[4][i][k].itemName[MAX_CHAR_PER_LINE] = '\0';
    }
    menu[4][i][1].itemPtr = &dcDutyCycle[1][i][1];
    menu[4][i][2].itemPtr = &dcTimeslot[1][i][1];
    menu[4][i][3].itemPtr = &dcDutyCycle[1][i][2];
    menu[4][i][4].itemPtr = &dcTimeslot[1][i][2];
    menu[4][i][5].itemPtr = &dcDutyCycle[1][i][3];
    menu[4][i][6].itemPtr = &dcTimeslot[1][i][3];
    menu[4][i][7].itemPtr = &dcDutyCycle[1][i][4];
    menu[4][i][8].itemPtr = &dcTimeslot[1][i][4];
    menu[4][i][9].itemPtr = &dcDutyCycle[1][i][5];
    menu[4][i][10].itemPtr = &dcTimeslot[1][i][5];
    menu[4][i][12].itemPtr = &dcDutyCycle[2][i][1];
    menu[4][i][13].itemPtr = &dcTimeslot[2][i][1];
    menu[4][i][14].itemPtr = &dcDutyCycle[2][i][2];
    menu[4][i][15].itemPtr = &dcTimeslot[2][i][2];
    menu[4][i][16].itemPtr = &dcDutyCycle[2][i][3];
    menu[4][i][17].itemPtr = &dcTimeslot[2][i][3];
    menu[4][i][18].itemPtr = &dcDutyCycle[2][i][4];
    menu[4][i][19].itemPtr = &dcTimeslot[2][i][4];
    menu[4][i][20].itemPtr = &dcDutyCycle[2][i][5];
    menu[4][i][21].itemPtr = &dcTimeslot[2][i][5];
    menu[4][i][23].itemPtr = &dcDutyCycle[3][i][1];
    menu[4][i][24].itemPtr = &dcTimeslot[3][i][1];
    menu[4][i][25].itemPtr = &dcDutyCycle[3][i][2];
    menu[4][i][26].itemPtr = &dcTimeslot[3][i][2];
    menu[4][i][27].itemPtr = &dcDutyCycle[3][i][3];
    menu[4][i][28].itemPtr = &dcTimeslot[3][i][3];
    menu[4][i][29].itemPtr = &dcDutyCycle[3][i][4];;
    menu[4][i][30].itemPtr = &dcTimeslot[3][i][4];
    menu[4][i][31].itemPtr = &dcDutyCycle[3][i][5];
    menu[4][i][32].itemPtr = &dcTimeslot[3][i][5];
    menu[4][i][34].itemPtr = &dcDutyCycle[4][i][1];
    menu[4][i][35].itemPtr = &dcTimeslot[4][i][1];
    menu[4][i][36].itemPtr = &dcDutyCycle[4][i][2];
    menu[4][i][37].itemPtr = &dcTimeslot[4][i][2];
    menu[4][i][38].itemPtr = &dcDutyCycle[4][i][3];
    menu[4][i][39].itemPtr = &dcTimeslot[4][i][3];
    menu[4][i][40].itemPtr = &dcDutyCycle[4][i][4];
    menu[4][i][41].itemPtr = &dcTimeslot[4][i][4];
    menu[4][i][42].itemPtr = &dcDutyCycle[4][i][5];
    menu[4][i][43].itemPtr = &dcTimeslot[4][i][5];
    menu[4][i][2].isSeconds = true;
    menu[4][i][4].isSeconds = true;
    menu[4][i][6].isSeconds = true;
    menu[4][i][8].isSeconds = true;
    menu[4][i][10].isSeconds = true;
    menu[4][i][13].isSeconds = true;
    menu[4][i][15].isSeconds = true;
    menu[4][i][17].isSeconds = true;
    menu[4][i][19].isSeconds = true;
    menu[4][i][21].isSeconds = true;
    menu[4][i][24].isSeconds = true;
    menu[4][i][26].isSeconds = true;
    menu[4][i][28].isSeconds = true;
    menu[4][i][30].isSeconds = true;
    menu[4][i][32].isSeconds = true;
    menu[4][i][35].isSeconds = true;
    menu[4][i][37].isSeconds = true;
    menu[4][i][39].isSeconds = true;
    menu[4][i][41].isSeconds = true;
    menu[4][i][43].isSeconds = true;
  }
}
