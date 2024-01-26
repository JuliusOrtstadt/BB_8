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

#include "Functions.hpp"

/*
 * This class symobolize the robot, it has methods for basic movements
 * but also ones to execute more complex tasks like the one asked in this exercice.
 */
class Robot{
  private:
    //------------------------Basic movement methods-----------------------
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void completeStop();

    //------------------------value reading methods------------------------

    /* Reads the value of the light sensor and returns one of the tree colors that can be found in the labyrinth,
     * if it none of these colors it returns color::UNDEFININD */
    color lightSensor();

    /* Atualise the value of the proximity sensors stored in the associated varaibles */
    void detection();


    //------------More developped methods using the basic ones------------

    /* This method detects on wich tile the robot is.
     * There are white, red and balck tiles, when two last one it actualise a counter.
     * This method should be called as often as possible. */
    void tileDetection();

    /* This method makes the robot follow the left wall using the proximity sensors. 
     * This method should be called as often as possible. */
    void followWallLeft();

    /* This method makes the robot follow the left wall using the proximity sensors. 
     * This method should be called as often as possible. */
    void followWallRight();

    /* This method makes the robot try to find the start again. It works by randomly searching for it.
     * This method should be called as often as possible.*/
    void backToStart();

    /* This method makes the robot try to find the red tile. It works by randomly searching for it.
     * This method should be called as often as possible. */
    void exploreMazeRandom();

    /* This method measure the time needed by the robot to complete one lap around the robot. */
    void mapTime();
  

  public:
    //constructor
    Robot();

    //findBlackTile is the main function off this exercice
    //because if full fills the main objective wich is
    //to find te balck case and go back to the start point
    void findBlackTile();

    //test is a method woch content changes a lot,
    //its only purpose is to test parts of the code witout commenting the main code (findBlackTile)
    //the content is defined in the file "BB_8.ino"
    void test();
};


#endif ROBOT_HPP;