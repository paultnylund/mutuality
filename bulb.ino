#include <ZX_Sensor.h>
#include <CapacitiveSensor.h>
#include <Perceptron.h>
#include <AccelStepper.h>
#include "EasyDelta.h"

const int PAUSE_TIME = 1000; // measured in ms
int count = 0;

const int TOUCH_SEND = 8;
const int TOUCH_RECIEVE = 3; // capacitive touch sensor pins
CapacitiveSensor capSense = CapacitiveSensor(TOUCH_SEND,TOUCH_RECIEVE); // init capacitive touch

const int ZX_ADDR = 0x10; // ZX Sensor I2C address
const int BOOK_LIFTED = 0; // Group to watch for interrupt (PIN 2)
ZX_Sensor zx_sensor = ZX_Sensor(ZX_ADDR);
volatile GestureType bookStatus;
volatile bool zx_interrupt_flag, isTouched;

perceptron sense(2); // number of inputs: touched + 1 bias
float guess = 0.0;

Motor motorA(0,1,d,0);
Motor motorB(4,5,XMAX,(YMAX - d));
Motor motorC(6,7,0,YMAX);

AccelStepper stepperA(AccelStepper::DRIVER, 1, 0);
AccelStepper stepperB(AccelStepper::DRIVER, 5, 4);
AccelStepper stepperC(AccelStepper::DRIVER, 7, 6);

void setup() {
  
  bookStatus = NO_GESTURE;

  Serial.begin(9600);
  Serial.println();

  if ( zx_sensor.init(GESTURE_INTERRUPTS) ) {
    Serial.println("ZX Sensor initialization complete");
  } else {
    Serial.println("Something went wrong during ZX Sensor init!");
  }

  capSense.set_CS_AutocaL_Millis(0xFFFFFFFF); // disable autocalibrate of CapacitiveSensor func.
  
  sense.randomize(); // weight init
  Serial.print("Randomized Weights: ");
  Serial.println(*sense.weights);
  Serial.println();
  
  zx_interrupt_flag = false;
  zx_sensor.clearInterrupt();
  attachInterrupt(BOOK_LIFTED, interruptZXRoutine, RISING);
  Serial.println("Configured ZX Interrupts");
  Serial.println();

  stepperA.setMaxSpeed(3000.0);
  stepperA.setAcceleration(1000.0);
  stepperB.setMaxSpeed(3000.0);
  stepperB.setAcceleration(1000.0);
  stepperC.setMaxSpeed(3000.0);
  stepperC.setAcceleration(1000.0);
  Serial.println("Configured Stepper Motors");
  Serial.println();

  Serial.println("----------");
  Serial.println("~ Ready! ~");
  Serial.println("----------");
  Serial.println();
  delay(10);
}

void(* resetFunc) (void) = 0;

void loop() {

  if (count == 0) {

    if ( zx_interrupt_flag ) {

      Serial.print("Book Moved / ");

      zx_interrupt_flag = false; // clear interrupt flag
      zx_sensor.clearInterrupt(); // clear interrupt
      Serial.println("Cleared ZX interrupt");
      
      bookStatus = zx_sensor.readGesture(); // read last bookStatus
  
      if (bookStatus == UP_SWIPE) {
  
        Serial.println("^ Object Lifted ^");
        Serial.println();
  
        makeAGuess();
      }
  
      count++;
    }
    
  } else if (count > 0) {
  
      makeAGuess();
      return;
      
  } else {

    Serial.println("ERROR! - Count variable is less than 0 for some reason.");
    Serial.println();
  }
}

void makeAGuess() {

  guess = sense.feedForward(); // make a guess

  Serial.print("Guessing: ");
  Serial.println(guess);
  Serial.println();

  if (guess != 1) { // check guess

    engage();
    waitForTouch();
    return;
    
  } else if (guess == 1) {

    goHome();
    Serial.println("Staying Away");
    wait(5);
    resetFunc();
  }
}

void interruptZXRoutine() {
  zx_interrupt_flag = true;
}

void touchSensed() {
  sense.inputs[0] = 2; // store in perceptron inputs
  Serial.println("Touch Sensed!\t ...Teaching.");
  Serial.println();
  return;
}

void waitForTouch() {

  Serial.println();
  Serial.println("Awaiting Touch...");
  Serial.println();

  while (true) {

    for (int i = 0; i <= PAUSE_TIME; i++) {

      long touchLvl = capSense.capacitiveSensor(30);

      if ( touchLvl >= 110 ) {

        Serial.println(touchLvl);
  
        touchSensed();
        goHome();
        return;
      }

      delay(1);
    }

    sense.inputs[0] = -1; // store in perceptron inputs
    Serial.println("No Touches.\t ...Teaching.");
    Serial.println();
    return;
  }
}

void engage() {

  Serial.println("Engaging");
  
  moveToPos(1800, 1000, 1000);

  return;
}

void goHome() {

  Serial.println("Homing");

  sense.train(-guess, guess);
  
  moveToPos(500, 500, 400);
  
  return;
}

void moveToPos(int x, int y, int z) {

  Serial.print("Moving to Position:\t( ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.print(" )");
  Serial.println();

  motorA.Update(x,y,z);
  motorB.Update(x,y,z);
  motorC.Update(x,y,z);

//  stepperA.moveTo(motorA.rotations);
//  stepperB.moveTo(motorB.rotations);
//  stepperC.moveTo(motorC.rotations);

  stepperA.moveTo(2000);
  stepperB.moveTo(2000);
  stepperC.moveTo(2000);

  stepperA.run();
  stepperB.run();
  stepperC.run();

  wait(3);
  
  return;
}

void wait(int n) {
  for (int i = 0; i < n; i++) {
    Serial.print(". ");
    delay(n * 500);
  }
  Serial.println();
  return;
}

