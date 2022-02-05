#include <FastAccelStepper.h>

#define dirPinStepper1 2
#define stepPinStepper1 4

#define dirPinStepper2 32
#define stepPinStepper2 33

#define dirPinStepper3 16
#define stepPinStepper3 17

#define dirPinStepper4 21
#define stepPinStepper4 22

#define dirPinStepper5 25
#define stepPinStepper5 26

#define dirPinStepper6 12
#define stepPinStepper6 13

#define PI 3.141592653

int raw_steps_per_rev[] = {
  1600, // Joint 1
  1600,
  1600,
  3200,
  3200,
  3200 //  Joint 6
};

float reduction_ratios[] = {
  25.0, // Joint 1
  27.0,
  15.0,
  11.846153,
  11.0,
  19.0 //  Joint 6
};

float computed_multiplier[6];

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[6];

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 18, 19);

  for (int i = 0; i < 6; i++) {
    computed_multiplier[i] = raw_steps_per_rev[i] * reduction_ratios[i] / (2 * PI);
    Serial.printf("Computed multiplier: %f \n", computed_multiplier[i]);
  }

  engine.init();
  steppers[0] = engine.stepperConnectToPin(stepPinStepper1);
  steppers[0]->setDirectionPin(dirPinStepper1);
  steppers[0]->setSpeedInUs(500);  // the parameter is us/step !!!
  steppers[0]->setAcceleration(3000);

  steppers[1] = engine.stepperConnectToPin(stepPinStepper2);
  steppers[1]->setDirectionPin(dirPinStepper2);
  steppers[1]->setSpeedInUs(500);  // the parameter is us/step !!!
  steppers[1]->setAcceleration(3000);

  steppers[2] = engine.stepperConnectToPin(stepPinStepper3);
  steppers[2]->setDirectionPin(dirPinStepper3, false);
  steppers[2]->setSpeedInUs(500);  // the parameter is us/step !!!
  steppers[2]->setAcceleration(3000);

  steppers[3] = engine.stepperConnectToPin(stepPinStepper4);
  steppers[3]->setDirectionPin(dirPinStepper4);
  steppers[3]->setSpeedInUs(500);  // the parameter is us/step !!!
  steppers[3]->setAcceleration(3000);

  steppers[4] = engine.stepperConnectToPin(stepPinStepper5);
  steppers[4]->setDirectionPin(dirPinStepper5, false);
  steppers[4]->setSpeedInUs(500);  // the parameter is us/step !!!
  steppers[4]->setAcceleration(3000);

  steppers[5] = engine.stepperConnectToPin(stepPinStepper6 );
  steppers[5]->setDirectionPin(dirPinStepper6);
  steppers[5]->setSpeedInUs(100);  // the parameter is us/step !!!
  steppers[5]->setAcceleration(5000);
}

void loop() {
  if (Serial2.available()) {
    String inputString = Serial2.readStringUntil('\n');
    const char *str = inputString.c_str();

    float joints[6];
    if (sscanf(str, "SET: %f, %f, %f, %f, %f, %f", &joints[0], &joints[1], &joints[2], &joints[3], &joints[4], &joints[5]) == 6) {
      Serial.printf("Received %f, %f, %f, %f, %f, %f \n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

      for (int i = 0; i < 6; i++) {
        steppers[i]->moveTo(joints[i]*computed_multiplier[i]);
      }
    }
  }

  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    const char *str = inputString.c_str();

    float joints[6];
    if (sscanf(str, "SET: %f, %f, %f, %f, %f, %f", &joints[0], &joints[1], &joints[2], &joints[3], &joints[4], &joints[5]) == 6) {
      Serial.printf("Received %f, %f, %f, %f, %f, %f \n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

      for (int i = 0; i < 6; i++) {
        steppers[i]->moveTo(joints[i]*computed_multiplier[i]);
      }
    }
  }
  /*if (Serial.available() > 0) {
    String myString = Serial.readStringUntil('\n');
    String motor = myString.substring(0, 1);
    int number = myString.substring(1).toInt();

    if (motor.equals("a")) {
      stepper1->moveTo(number);
    } else if (motor.equals("b")) {
      stepper2->moveTo(number);
    } else if (motor.equals("c")) {
      stepper3->moveTo(number);
    } else if (motor.equals("d")) {
      stepper4->moveTo(number);
    } else if (motor.equals("e")) {
      stepper5->moveTo(number);
    } else if (motor.equals("f")) {
      stepper6->moveTo(number);
    }

    Serial.println("Moving motor " + motor +  " to: " + String(number));
    }*/
}
