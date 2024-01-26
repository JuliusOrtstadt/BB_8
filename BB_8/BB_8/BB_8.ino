/*
******************************************
*     Informatique pour la robotique 
*         Maze solving robot
*   By HENDRIKSE Jeremy & ORTSTADT Julius
*               Robo3
******************************************
*/

#include "Robot.hpp";
#include "Functions.hpp";

// Pins for the different components on the robot.
#define LIGHT_SENSOR_PIN A0

#define PROX_SENSOR_L_PIN A1
#define PROX_SENSOR_R_PIN A2
#define PROX_SENSOR_FL_PIN A3
#define PROX_SENSOR_FR_PIN A4
#define PROX_SENSOR_RL_PIN A5
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN 6
#define PROX_SENSOR_DR_PIN 9

#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5

// Variables that will hold the value of the proximity sensors.
int sensorL;
int sensorR;
int sensorFL;
int sensorFR;
int sensorDL;
int sensorDR;

// Variables containing the thresholdsfor the distance sensors.
const int thresholdFront = 500;
const int thresholdDiagonal = 600;
const int thresholdSide = 400;

// Variables for the speed of the robot.
const int speed = 255;
const int turnSpeed = speed - 100;

// Variables for tile detection.
color last_color_detected = color::UNDEFINED;
GroundType actual_ground;
int tile_timer_threshold = 200; // Value may need to be adjusted depending on the computer.
unsigned long int timer_black_color = ULONG_MAX;
unsigned long int timer_red_color = ULONG_MAX;
int redcpt;
int blackcpt;

// Randomly generated list to simulate a random direction decision.
int randPath[100] = {0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
int turncpt = 0;

// Variables to measure the time needed by the robot.
unsigned long timeToBlack; // To get to the black tile.
unsigned long lapTime;     // To complete one lap around the maze.
unsigned long startMillis;
bool timeblccpt = false; // If the robot has measured the time needed to get to the black tile.
bool timeredcpt = false; // If the robot has measured the time needed to go from the red tile to the red tile after completing one lap around the maze.
bool testLap = true; // If the robot is measuring the maze.

// Variable and Enum so the robot can find its way back to start after random exploration.
unsigned long lapStartTime;
enum RobotState{FOLLOW_WALL, GO_STRAIGHT};
RobotState currentState = FOLLOW_WALL;

void hardware_setup() { // HoRoSim Function.
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

void setup() {
  // Start communication between HoRoSim window and Arduino.
  Serial.begin(4800);

  // Set pin mode for motors
  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);

  startMillis = millis(); // Initialize the start time.
  lapStartTime = millis(); // Initialize the start time for the backToStart function.
}

void loop() {  
  // Find the black tile in the maze and get back to start as fast as possible
  findBlackTile();
}

void findBlackTile(){
  // The robot does a test lap.
  if (testLap == true){
    mapTime();
  }
  else if (testLap == false) {
    if ((timeToBlack != 0) && (redcpt != 2)){ // Black tile has been found along the perimeter wall.
      if (lapTime-timeToBlack < timeToBlack){ // Time by following the right wall is shorter so we do this and then turn around.
        tileDetection();
        if (blackcpt == 0){
          followWallRight();
        }
        else if (blackcpt == 1){ // Black tile found / Turn around.
          followWallLeft();
        }
      }
      else { // The other path is shorter.
        tileDetection();
        if (blackcpt == 0){ // We follow the left wall until we find the black tile.
          followWallLeft(); 
        }
        else if (blackcpt == 1){ // We turn around.
          followWallRight();
        }
      }
    }
    
    else if (blackcpt != 1){ // The black tile has not been found along the perimeter wall. We randomly explore the maze.
      tileDetection();
      if (turncpt >= 100){ // Reset the counter if it exceeds the randPath list bounds.
        turncpt = 0;
      }
      exploreMazeRandom(); 
      redcpt = 1; // Red tile counter forced to one so that the robot can go back to start in the end.
    }

    // The robot is back at the start and stops.
    else if (redcpt == 2){
      completeStop();
    }
    
    // The black tile has been found (not along the perimeter wall). The robot looks for this wall and finds its way back to start.
    else { 
      tileDetection();
      backToStart();
    }
  }
}

void backToStart(){ // Function which allows the robot to get back to start if random exploration occured.
  unsigned long currentTime = millis();

  switch (currentState){
    case FOLLOW_WALL:
      followWallLeft();
      if (currentTime - lapStartTime > lapTime + 5000){ // The robot followed the left wall longer than it takes to do a lap (+ tolerance) -> change wall and start again
        currentState = GO_STRAIGHT; 
        lapStartTime = currentTime;
      }
      break;
    case GO_STRAIGHT:
      detection();
      if (sensorFL > thresholdFront && sensorFR > thresholdFront){
        forward(speed);
        if (currentTime - lapStartTime > 5000){ // Searched for a new wall for 5 seconds / start following it again
        currentState = FOLLOW_WALL;
        lapStartTime = currentTime;
        }
      }
      else { // Found a new wall sooner / start following it again
        currentState = FOLLOW_WALL;
        lapStartTime = currentTime;
      }
      break;
  }
}

void mapTime(){ // Function to measure the time needed by the robot to complete one lap around the robot.
  tileDetection();
  followWallLeft();
  
  // Time to black tile, if black tile is spotted along the perimeter wall
  if (blackcpt == 1 && timeblccpt == false){
    timeToBlack = millis() - startMillis;
    Serial.print("Black tile time: ");
    Serial.print(timeToBlack/1000);
    Serial.println(" s");
    timeblccpt = true;
  }
  // Robot is back at start after finishing one lap around the maze 
  if (redcpt == 2 && timeredcpt == false){
    lapTime = millis() - startMillis;
    Serial.print("Lap time: ");
    Serial.print(lapTime/1000);  
    Serial.println(" s");
    timeredcpt = true;
    testLap = false;
    redcpt = 1; // Reset these values since this was only a test lap and not part of the main goal of the robot.
    blackcpt = 0;
  }
}

void exploreMazeRandom(){ // Function used to explore the maze randomly
  detection();
  
  if ((sensorFL < thresholdFront) || (sensorFR < thresholdFront)){
    // Something is detected somewhere in the path the robot.
    // We choose a "random" direction for the robot to follow.
    if (randPath[turncpt] == 1){ 
      if (sensorL > thresholdSide){ // If there is no wall on this side we can turn.
        turnLeft(turnSpeed);
        turncpt++;
      }
      else { // There is a wall on this side, we can't turn. We ignore the random decision and turn in the other direction.
        turnRight(turnSpeed);
        turncpt++;
      }
    }
    // Same as before except in the other direction.
    else if (randPath[turncpt] == 0) {
      if (sensorR > thresholdSide){
        turnRight(turnSpeed);
        turncpt++;
      }
      else {
        turnLeft(turnSpeed);
        turncpt++;
      }
    }
  }

  else if ((sensorFL > thresholdFront) && (sensorFR > thresholdFront)){ 
    // Nothing is detected in front of the robot -> robot moves forward
    forward(speed);
  }

  else if ((sensorFL < thresholdFront) && (sensorL < thresholdSide)){
    // Something is detected to the left of the robot, we need to turn right
    turnRight(turnSpeed);
    turncpt++;
  }

  else if ((sensorFR < thresholdFront) && (sensorR < thresholdSide)){
    // Something is detected to the right of the robot, we need to turn left
    turnLeft(turnSpeed);
    turncpt++;
  }
}

void tileDetection() { // Detect which colored tile the robot is on.
  color color = lightSensor();
  if (color != color::UNDEFINED && color != last_color_detected) {
    last_color_detected = color;
    if (color == color::RED) {
      timer_red_color = millis();
    }
    else if (color == color::WHITE && actual_ground != GroundType::WHITETILE) {
      Serial.println("ON white TILE");
      actual_ground = GroundType::WHITETILE;
    }
    else if (color == color::BLACK) {
      timer_black_color = millis();
    }
  }
  else if (color==color::BLACK && actual_ground != GroundType::BLACKTILE && (millis()-timer_black_color > tile_timer_threshold) ) {
    Serial.println("ON black TILE");
    actual_ground = GroundType::BLACKTILE;
    blackcpt++;
    Serial.print(blackcpt);Serial.print("  ");Serial.println(redcpt);
  }

  else if (color==color::RED && actual_ground != GroundType::REDTILE && (millis()-timer_red_color > tile_timer_threshold) ){
    Serial.println("ON red TILE");
    actual_ground = GroundType::REDTILE;
    redcpt++;
    Serial.print(blackcpt);Serial.print("  ");Serial.println(redcpt);
  }
}

color lightSensor() { // Detect the color of the tile using the light sensor
  const int val = analogRead(LIGHT_SENSOR_PIN);
  if ( 0<val && val<150 ){
    return color::BLACK;
  }
  else if (275<val && val<290) {
    return color::RED;
  }
  else if (500<val) {
    return color::WHITE;
  }
  else {
    return color::UNDEFINED;
  }
}

void printColor(color c) { // Print the color of the tile
  switch (c) {
    case color::BLACK:
      Serial.println("Black");
      break;
    case color::RED:
      Serial.println("Red");
      break;
    case color::WHITE:
      Serial.println("White");
      break;
    case color::UNDEFINED:
      Serial.println("Undefined");
      break;
  }
}

void followWallLeft(){ // Follow the left wall
  detection(); // Read all the proximity sensor values
  if ((sensorFL > thresholdFront) && (sensorFR > thresholdFront)){ 
      // Nothing is detected in front of the robot -> robot moves forward
      forward(speed);
  }
  if ((sensorFL < thresholdFront) || (sensorFR < thresholdFront)){
    // Something is detected somewhere in the path the robot -> turn right
    turnRight(turnSpeed);
  }
  if ((sensorFL > thresholdDiagonal) && (sensorL > thresholdSide) && (sensorDL > thresholdDiagonal)){
    // If we lose sight of the left wall we turn left to find it again
    turnLeft(turnSpeed);
  }
}

void followWallRight(){ // Follow the right wall
  detection(); // Read all the proximity sensor values
  if ((sensorFL > thresholdFront) && (sensorFR > thresholdFront)){ 
    // Nothing is detected in front of the robot -> robot moves forward
    forward(speed);
  }
  if ((sensorFL < thresholdFront) || (sensorFR < thresholdFront)){
    // Something is detected somewhere in the path the robot -> turn left
    turnLeft(turnSpeed);
  }
  if ((sensorFR > thresholdDiagonal) && (sensorR > thresholdSide) && (sensorDR > thresholdDiagonal)){
    // If we lose sight of the right wall we turn right to find it again
    turnRight(turnSpeed);
  }
}

void detection(){ // Read all the proximity sensor values and store them.
  sensorL=analogRead(PROX_SENSOR_L_PIN);
  sensorR=analogRead(PROX_SENSOR_R_PIN);
  sensorFL=analogRead(PROX_SENSOR_FL_PIN);
  sensorFR=analogRead(PROX_SENSOR_FR_PIN);
  sensorDL=analogRead(PROX_SENSOR_DL_PIN);
  sensorDR=analogRead(PROX_SENSOR_DR_PIN);
}

void forward(int speed){ // The robot moves forwards
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);

  analogWrite(MOTOR_R_SPEED, speed);
  analogWrite(MOTOR_L_SPEED, speed);
}

void turnLeft(int speed){ // The robot turns left
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);

  analogWrite(MOTOR_R_SPEED, speed);
  analogWrite(MOTOR_L_SPEED, speed);
}

void turnRight(int speed){ // The robot turns right
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);

  analogWrite(MOTOR_R_SPEED, speed);
  analogWrite(MOTOR_L_SPEED, speed);
}

void completeStop(){ // The robot comes to a complete stop
  analogWrite(MOTOR_R_SPEED, 0);
  analogWrite(MOTOR_L_SPEED, 0);
}