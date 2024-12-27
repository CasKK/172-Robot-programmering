#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

//Linesensor values
#define numSensors 5                    //defines how many sensors is active.
uint16_t lineSensorValues[numSensors];  //Int-array with the line sensor readings.
uint16_t threshold[5];                  //Int-array with the thesholds for when a sensor's values qualifies as on the line or not.

//Movement values
float speed = 100.0;           //Standard speed. It is a float so that it can be devided.
float spinFactor = 0.6;        //Speed factor for spin
float sharpTurnInside = -0.7;  //Inside wheel speed factor on sharp turns.
float sharpTurnOutside = 0.7;  //Outside wheel speed factor on sharp turns.
float softTurnInside = 0.5;    //Inside wheel speed factor on soft turns.
float softTurnOutside = 1;     //Outside wheel speed factor on soft turns.

//misc values
unsigned long startTime;  //Used to mesure the time before it initiates a sharp turn on the outside of a corner.
uint8_t waitFactor = 4;   //Used to calculate the time before it initiates a sharp turn on the outside of a corner.
bool driveMode = 0;       //If driveMode is 1, a while-loop with the drive algorithm will run. If 0, the while-loop will not continue .
bool direction = 0;       //Used to decide if the line should be followed clockwise or counterclockwise.


void setup() {
  lineSensors.initFiveSensors();  //Initialize five line sensors.
  Serial.begin(9600);             //Begin serial communication.
}

void printArrayToSerial(int arr[]) {  //Funktion that prints array to serial.
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d \n",
          arr[0],  //left
          arr[1],  //middle left
          arr[2],  //middle
          arr[3],  //middle right
          arr[4]   //right
  );
  Serial.print(buffer);
}

void calibrateLS() {  //Will calibrate the line sensors(make theshold).
  display.clear();
  display.print("delay");  //To allow the user to remove finger from button.
  delay(1000);
  display.clear();
  uint16_t lineSensorMax[5] = { 0, 0, 0, 0, 0 };                 //Int-array with the max value each sensor has read during the calibrationLS funktion. The values start at zero, to be up-adjusted in the calibration function.
  uint16_t lineSensorMin[5] = { 2000, 2000, 2000, 2000, 2000 };  //Int-array with the min value each sensor has read during the calibrationLS funktion. The values start at max, to be down-adjusted in the calibration function.
  spin();  //Sets the robot in a spin, so the sensors will pass over the line and the floor.
  bool cal = 1;
  for (uint16_t i = 0; i < 3000; i++) {                   //This loop runs 3000 times. Each sensor is read once per loop, and the min/max values are updated if needed.
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  //Reads line sensors.
    for (uint8_t j = 0; j < numSensors; j++) {            //For loop that iterates through each line sensor.
      if (lineSensorValues[j] > lineSensorMax[j]) {       //Checks if the newly read line sensor value is higher than the previous max.
        lineSensorMax[j] = lineSensorValues[j];           //Sets max line sensor value to the newly read line sensor value.
      }
      if (lineSensorValues[j] < lineSensorMin[j]) {  //Checks if the newly read line sensor value is lower than the previous min.
        lineSensorMin[j] = lineSensorValues[j];      //Sets min line sensor value to the newly read line sensor value.
      }
    }
    /*
    Serial.print("Max:      ");
    printArrayToSerial(lineSensorMax);  //Prints provisional max values
    Serial.print("Readings: ");
    printArrayToSerial(lineSensorValues);  //Prints readings
    Serial.print("Min:      ");
    printArrayToSerial(lineSensorMin);  //Prints provisional min values
    */
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
  /*
  Serial.print("Thres:    ");
  printArrayToSerial(threshold);  //Prints theshold.
  */
  cal = 0;
}

int sensorOnLine() {                                    //Checks which sensors are on the line. Returns unique int for each combination.
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  //Reads line sensor values into array "lineSensorValues".
  uint8_t x = 0;                                        //Initializes int to be manipulated later.
  if (direction == 0) {                                 //Checks if direction is clockwise or counterclockwise.
    for (int i = 0; i < numSensors; i++) {              //If direction is 0, the bot will drive counterclockwise, and the unique int will be made with it's bits from right to left:
      if (lineSensorValues[i] > threshold[i]) {         //For each line sensor, if line sensor value is more than the corresponding theshold:
        x |= 1 << i;                                    //Set bit in (x) to 1 if corresponding bit in (x) or corresponding bit in (1 << i) is 1. Same as (x = x | 1 << i).
      }                                                 //The (1 << i) make int with bits 00000001 and then moves the bits i places to the left.
    }
  } else {                                       //If direction is 1, the bot will drive clockwise.
    for (int i = 0; i < numSensors; i++) {       //While still reading sensors in the same order as before,
      if (lineSensorValues[i] > threshold[i]) {  //the unique int will be made from left to right starting from bit 5 (bit numSensors):
        x |= 1 << (numSensors - i - 1);          //This results in the order of the sensors being reversed in the bits, and the same switch case can therefore be used for both directions.
      }
    }          //This function essentially makes an integer 00000000, where the last 5 bits corresponds to a sensor on the line or not.
  }            //That bit combination makes a unique integer to be used in the switch.
  return (x);  //If only sensor 0 is on the line, it would look like this: 00000001. If only sensor 2: 00000100. Only sensor 4: 00010000.
}

//Movement funktions.
void spin() {  //Will spin the robot.
  display.gotoXY(0, 0);
  display.print("calibration");
  motors.setSpeeds(speed * spinFactor, -speed * spinFactor);
}
void forward() {  //Drives forward.
  display.gotoXY(0, 0);
  display.print("fwd");
  motors.setSpeeds(speed, speed);
}
void softTurn(bool x) {  //Soft turn towards input direction.
  display.gotoXY(0, 0);
  display.print("Soft");
  display.gotoXY(0, 1);
  if (x == 0) {
    motors.setSpeeds(speed * softTurnInside, speed * softTurnOutside);
    display.print("<--");
  } else {
    motors.setSpeeds(speed * softTurnOutside, speed * softTurnInside);
    display.print("-->");
  }
  display.print("       ");
  display.gotoXY(0, 0);
  display.print("       ");
}
void sharpTurn(bool x) {  //Sharp turn towards input direction.
  display.gotoXY(0, 0);
  display.print("Sharp");
  display.gotoXY(0, 1);
  if (x == 0) {
    motors.setSpeeds(speed * sharpTurnInside, speed * sharpTurnOutside);
    display.print("<----!!!");
  } else {
    motors.setSpeeds(speed * sharpTurnOutside, speed * sharpTurnInside);
    display.print("!!!---->");
  }
  display.print("       ");
  display.gotoXY(0, 0);
  display.print("       ");
}
//---------------------------------------


void driveToLine() {  //This function will lead the Zumo to the line.
  display.clear();
  driveMode = 1;
  forward();                              //Starts off by moving forward.
  while (!sensorOnLine() && driveMode) {  //while not sensorOnLine. Same as while sensorOnLine == 0.
    delay(20);
    if (buttonC.getSingleDebouncedRelease()) {  //This button will set drivemode to 0, and the loop will therefore not continue.
      driveMode = 0;                            //break; could also be used. The while could then be while(true) instead of while(driveMode).
      buttonC.waitForRelease();
    }
  }
  motors.setSpeeds(0, 0);                 //when sensorOnLine is no longer 0, motors will stop.
  if (sensorOnLine() > 8 && driveMode) {  //If a sensor on the right side is the first one to see the line, it will set the direction to clockwise. Will not run if driveMode is 0(which means it was cancelled).
    direction = 1;                        //Standard direction = 0 is counterclockwise, and it will bounce off the left most sensor.
  } else if (driveMode) {                 //Direction = 1 is clockwise, and it will bounce off the right most sensor.
    direction = 0;
  }
  driveMode = 0;
}

void driveLine() {  //This function will lead the Zumo once the line is found.
  display.clear();
  driveMode = 1;
  while (driveMode) {
    switch (sensorOnLine()) {
      case 0 & 16:            //Normal mode 1. This runs of no sensor is on the line
        softTurn(direction);  //Turn into the line.
        Serial.println("case 0");
        break;
      case 1 & 17:             //Normal mode 2. This runs if only sensor 0 is on the line.
        softTurn(!direction);  //Turn away from the line.
        Serial.println("case 1");
        startTime = millis();
        break;
      case 2:  //Will turn away from the line sharper than the normal case 1. This runs if only sensor 1 (the second sensor from a side) is on the line.
        while (sensorOnLine() != 1) {
          sharpTurn(!direction);  //Turn away from the line.
          Serial.println("case 2");
          delay(50);
          if (buttonC.getSingleDebouncedRelease()) {  //This button will set drivemode to 0, and the loop will therefore not continue.
            driveMode = 0;                            //break; could also be used. The while could then be while(true) instead of while(driveMode).
            motors.setSpeeds(0, 0);                   //To stop whatever movement is going on
            buttonC.waitForRelease();
            break;
          }
        }
        break;
      default:  //Any other state should mean it is on the inside of a turn,

        sharpTurn(!direction);  //Turn away from the line.
        startTime = millis();
        Serial.println("default");
    }
    if (millis() - startTime > speed * waitFactor) {  //To turn sharp when on outside of a turn. It mesures the time since it saw a line last.
      sharpTurn(direction);                           //Turns sharp if the time is exceeded.
      Serial.println("Outside");
    }
    if (buttonC.getSingleDebouncedRelease()) {  //This button will set drivemode to 0, and the loop will therefore not continue.
      driveMode = 0;                            //break; could also be used. The while could then be while(true) instead of while(driveMode).
      motors.setSpeeds(0, 0);                   //To stop whatever movement is going on
      buttonC.waitForRelease();
    }
  }
}

void loop() {
  if (buttonA.getSingleDebouncedRelease()) {  //Button A press will calibrate line sensors(make theshold).
    buttonA.waitForRelease();
    calibrateLS();  //Build-in delay to remove finger from button.
  }
  if (buttonB.getSingleDebouncedRelease()) {  //Button B press will start the drive mode.
    buttonB.waitForRelease();
    display.gotoXY(0, 0);
    display.print("Delay");
    delay(1000);  //Allow for finger removal.
    driveToLine();
    driveLine();
  }
  display.gotoXY(0, 0);
  display.print("Loop");
  display.print("     ");
}
