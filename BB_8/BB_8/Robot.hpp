/*
******************************************
*     Informatique pour la robotique 
*         Maze solving robot
*   By HENDRIKSE Jeremy & ORTSTADT Julius
*               Robo3
******************************************
*/

#ifndef ROBOT_HPP;
#define ROBOT_HPP;

/* class: Robot
this class blablabla

*/

class Robot{
  private:
    //Basic robot moving functions
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void completeStop();

    //Methods used to read values
    void lightSensor();
    void detection();

    //utility functions
    void tileDetection();
    void followWallLeft();
  

  public:
    //constructor
    Robot();

    //findBlackCase is the main function off this exercice
    //because if full fills the main objective wich is
    //to find te balck case and go back to the start point
    void findBlackCase() {
      tileDetection();
      followWallLeft();
    };

    //test is a method woch content changes a lot,
    //its only purpose is to test parts of the code witout commenting the main code (findBlackCase)
    //the content is defined in the file "BB_8.ino"
    void test();
};

#endif ROBOT_HPP;