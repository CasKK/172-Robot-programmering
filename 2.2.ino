#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4OLED display;


#define numSensors 5                    //defines how many sensors is active.
uint16_t lineSensorValues[numSensors];  //Int-array with the line sensor readings.

float speed = 150.0;  //Standard speed. It is a float so that it can be devided.

unsigned long startTime;
int waitFactor = 4;

//Linesensor stuff
uint16_t lineSensorMax[5] = { 0, 0, 0, 0, 0 };                 //Int-array with the max value each sensor has read, during the calibrationLS funktion. The values start at zero, to be up-adjusted in the calibration function.
uint16_t lineSensorMin[5] = { 2000, 2000, 2000, 2000, 2000 };  //Int-array with the min value each sensor has read, during the calibrationLS funktion. The values start at max, to be down-adjusted in the calibration function.
uint16_t threshold[5];                                         //Int-array with the thesholds for when a sensor's values qualifies as on the line or not.

//Movement ratios
float sharpTurnInside = -0.2;  //Inside wheel speed factor on sharp turns.
float sharpTurnOutside = 0.5;  //Outside wheel speed factor on sharp turns.
float softTurnInside = 0.5;    //Inside wheel speed factor on soft turns.
float softTurnOutside = 0.75;  //Outside wheel speed factor on soft turns.
float turnInside = 0.3;        //Inside wheel speed factor on normal turns.
float turnOutside = 0.75;      //Outside wheel speed factor on normal turns.

bool driveMode = 0;  //Drivemode. If drivemode is 1, a while-loop with the drive algorithm will run. If 0, the program will exit the while-loop.
bool lastTurn = 0;   //This will keed track of the last sharp turn, incase the line should enter a blindspot between the sensors.

void setup() {
  lineSensors.initFiveSensors();  //Initialize five line sensors.
  Serial.begin(9600);             //Begin serial communication.
}

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  //Reads line sensors.
  //printReadingsToSerial(); //Prints readings to serial.
}

int sensorOnLine() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  uint8_t numberToSwitch = 0;
  for (int i = 0; i < numSensors; i++) {
    if (lineSensorValues[i] > threshold[i]) {
      numberToSwitch |= 1 << i;
    }
  }
  return (numberToSwitch);
}

//Print and display functions. ---------------
void printReadingsToSerial() {  //The funktion that prints readings to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          lineSensorValues[0],  //left sensor
          lineSensorValues[1],  //middle left
          lineSensorValues[2],  //middle
          lineSensorValues[3],  //middle right
          lineSensorValues[4]   //right sensor
  );
  Serial.print(buffer);
}
void printMaxToSerial() {  //The funktion that prints the max values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d",
          lineSensorMax[0],  //left sensor
          lineSensorMax[1],  //middle left
          lineSensorMax[2],  //middle
          lineSensorMax[3],  //middle right
          lineSensorMax[4]   //right sensor
  );
  Serial.print(buffer);
}
void printMinToSerial() {  //The funktion that prints the min values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          lineSensorMin[0],  //left sensor
          lineSensorMin[1],  //middle left
          lineSensorMin[2],  //middle
          lineSensorMin[3],  //middle right
          lineSensorMin[4]   //right sensor
  );
  Serial.print(buffer);
}
void printThresholdToSerial() {  //The funktion that prints the theshold values to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          threshold[0],  //left sensor
          threshold[1],  //middle left
          threshold[2],  //middle
          threshold[3],  //middle right
          threshold[4]   //right sensor
  );
  Serial.print(buffer);
}


//Movement funktions. Self explanetory. -----------------------------------------------------------------------
void spin() {
  display.print("spin");
  motors.setSpeeds(speed * 0.5, -speed * 0.5);
}
//Not used rn -->
void forward() {
  //display.print("WUUUU!!!");
  display.print("fwd");
  motors.setSpeeds(speed, speed);
}
void sharpLeft() {
  display.print("<----!!!");
  motors.setSpeeds(speed * sharpTurnInside, speed * sharpTurnOutside);
}
void sharpRight() {
  display.print("!!!---->");
  motors.setSpeeds(speed * sharpTurnOutside, speed * sharpTurnInside);
}
void turnLeft() {
  display.print("<---");
  motors.setSpeeds(speed * turnInside, speed * turnOutside);
}
void turnRight() {
  display.print("--->");
  motors.setSpeeds(speed * turnOutside, speed * turnInside);
}
void softLeft() {
  display.print("<--");
  motors.setSpeeds(speed * softTurnInside, speed * softTurnOutside);
}
void softRight() {
  display.print("-->");
  motors.setSpeeds(speed * softTurnOutside, speed * softTurnInside);
}
//---------------------------------------

void calibrateLS() {  //Will calibrate the line sensors for the line follower mode.
  display.clear();
  display.print("delay");
  delay(1000);
  display.clear();

  spin();  //Sets the robot in a spin, so the sensors will pass over the line and the floor.
  bool cal = 1;
  for (uint16_t i = 0; i < 3000; i++) {  //This loop runs 5000 times. Each sensor is read once per loop, and the min/max values are updated if needed.
    readLineSensors();
    for (uint8_t j = 0; j < numSensors; j++) {       //For loop that iterates through each line sensor.
      if (lineSensorValues[j] > lineSensorMax[j]) {  //Checks if the newly read line sensor value is higher than the previous max.
        lineSensorMax[j] = lineSensorValues[j];      //Sets max line sensor value to the newly read line sensor value.
      }
      if (lineSensorValues[j] < lineSensorMin[j]) {  //Checks if the newly read line sensor value is lower than the previous min.
        lineSensorMin[j] = lineSensorValues[j];      //Sets min line sensor value to the newly read line sensor value.
      }
    }
    Serial.print("Max:     ");
    printMaxToSerial();  //Prints provisional max values
    Serial.print("Readings:");
    printReadingsToSerial();  //Prints readings
    Serial.print("Min:     ");
    printMinToSerial();  //Prints provisional min values

    if (buttonC.getSingleDebouncedRelease()) {  //This button will set cal to 0, and break the loop.
      cal = 0;
      buttonC.waitForRelease();
      break;
    }
  }
  motors.setSpeeds(0, 0);                                                           //Stops the spin. The final Max and min values have been found at this point.
  if (cal) {                                                                        //If cal is 0, the thesholds will not be updated. This means the calibration can be cancelled without erasing the old calibration.
    for (int j = 0; j < numSensors; j++) {                                          //Iterates through each line sensor
      threshold[j] = (lineSensorMax[j] - lineSensorMin[j]) / 2 + lineSensorMin[j];  //Calculates the thesholds based on max and min.
    }
  }
  Serial.print("Thres:  ");
  printThresholdToSerial();  //Prints theshold.
  cal = 0;
}

void driveToLine(){
  forward();
  while(sensorOnLine() == 0){
    
  }
}


void driveLine() {
  display.clear();
  while (driveMode) {
    switch (sensorOnLine()) {
      case 0:  //Normal mode 1 (turn into the line)
        motors.setSpeeds(speed * 0.5, speed);
        Serial.print("case 0");
        break;
      case 1:  //Normal mode 2 (turn away from the line)
        motors.setSpeeds(speed, speed * 0.5);
        Serial.print("case 1");
        startTime = millis();
        break;
      case 2:  //Error mode. Will turn away from the line until sensor 0 looses the line on the OUTSIDE.
        while (sensorOnLine() != 1) {
          motors.setSpeeds(speed, speed * 0.5);
          Serial.print("case 2 part1");
          delay(50);
        }
        while (sensorOnLine() == 1) {
          motors.setSpeeds(speed, speed * 0.5);
          Serial.print("case 2 part2");
          delay(50);
        }
        break;
      default:  //Any other state should mean it is on the inside of a turn,
        sharpRight();
        startTime = millis();
        Serial.print("default");
    }
    if (millis() - startTime > speed * waitFactor) { //To turn sharp when on outside of a turn.
      sharpLeft();
    }
    if (buttonC.getSingleDebouncedRelease()) {  //This button will set drivemode to 0, and the loop will therefore not continue.
      driveMode = 0;                            //yes
      motors.setSpeeds(0, 0);                   //To stop whatever movement is going on
      buttonC.waitForRelease();
    }
  }
}

void loop() {
  if (buttonA.getSingleDebouncedRelease()) {
    buttonA.waitForRelease();
    calibrateLS();
  }

  if (buttonB.getSingleDebouncedRelease()) {
    driveMode = 1;
    buttonB.waitForRelease();
    driveLine();
  }

  if (buttonC.getSingleDebouncedRelease()) {
    buttonC.waitForRelease();
    Serial.println(sensorOnLine());
  }
  display.gotoXY(0,0);
  display.print("Loop");
  display.print("     ");
}
