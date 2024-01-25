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

class Robot{
  private:
    //Basic robot moving functions
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void completeStop();

    //Methods used to
    void lightSensor();
    void detection();
    //utility functions
    void tileDetection();
    void followWallLeft();
  

  public:
    Robot(); //constructeur
    void findBlackCase() {
      tileDetection();
      followWallLeft();
    };
    void test();
};



#endif ROBOT_HPP;