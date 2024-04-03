# <p align="center">Documentation</p>
## <p align="center">BB-8 - The maze solving robot</p>
#### <p align="center">Hendrikse Jeremy & Ortstadt Julius - Robo3</p>

## Setup
This project was made using Arduino IDE for coding, CoppeliaSim Edu for the simulation and HoRoSim for the link between the IDE and the simulation software.
To install HoRoSim download the installer from the software section of this repo.
As the installation folder, select the Arduino sketch folder.
You then need to add the compiler to the PATH variable.
To test if everything is working, open the Arduino IDE, create a new sketch and in the Boards and Ports menu select: "ArduinoUno_HoRoSim". 
Now open CoppeliaSim, load one of the labyrinths, upload the code in the IDE and now everything should work.
If you want to upload another code make sure that you close the HoRoSim window before.

## Note to the user
If the robot is really fast and can't follow walls correctly, the user should change the value of the "speed" variable (File: BB_8.ino / Line 48) to a lower value. 
Note that this value cannot be less than 100 otherwise the "turnSpeed" value will be negative and the code won't work as intended. 

It can also happen that if the robot is really fast at the beginning of the simulation and becomes slower at a certain point, although no variable is changed in the code or a specific event took place, that the timing will not work properly and the robot may not be able to find its way back to start as intended. 
If this is the case, we saw that by simply re-uploading the code, meaning restarting the simulation, took care of the problem. 

Finally, if the robots movements are to rapid and/or sharp, it can lead to the robot either tipping over or leaning a small amount of time. If this happens, it can be that the light sensor detects black and thinks it found the black tile, which was not the case.
If this happens, the robots speed may need to be slowed a bit and the simulation restarted.

## Strategy
The robots goal is to find the black tile in the maze. 
To achieve this we used a multitude of functions enabling the robot to do different things at different stages of its journey.
First, the robot will perform a test lap during which we will measure the time he needs to complete one lap (so from start to start) around the maze. 
Additionaly, if we find the black tile along the perimeter wall (which we follow), we also measure the time the robot needed to get there.
After the test lap we reset the counters and the real exploration begins.
If the robot has a time for the black tile, we determine which way is faster to get there and follow the right or left wall accordingly.
Next, if the robot didn't find the black tile along the perimeter wall, e.g. we don't have a time to the black tile, we start a random exploration.
Basically, the robot will bounce around the maze until he finds the black tile, making semi-random decisions, more on that later.
When he finds the black tile, the robot needs to find its way back. 
For that, the robot will follow a wall to its left for the same amount of time it took him to do one lap around the maze (plus tolerance). 
We chose this amount of time to be sure that he doesn't "exit" the wall earlier in case it is the perimeter wall and it would have led him back to start.
Now after this amount of time, the robot will go forward until he finds a new wall and start the whole process all over again until he is back at start. 
When he arrived back at start either after random exploration or by following the wall, as soon as he detects the red tile the robot stops.

So, in basic terms, this is the strategie our robot follows to find the black tile and get back to where he started.

This is the only simple solutions or maybe the only solution, I will explain why. 
So first, why not try to make a matrix from the labyrinth to navigate in it, well because we don't have enough sensor (a motor encoder could be great or distance sensor with a big range).
Also because, depending on the computer, the simulation doens't run on the same speed and the millis method from arduino is based on real time and not the simulation time. 
This led to another problem, which was detecting the red and black tiles, this also really quite difficult to solve because the timer is based on a timer to make the difference between the black lines and the black tile.
Also we cannot detect in a realiable manner the lines on the ground because often the light sensor detects red when it is between the black line and the white tile.
Last problem, we cannot have acces to all sensors on the robot whith analog entries because there are only 6 on the Arduino and wa have 8 sensors so we had make a choice.


## The code
Let's go over some of our code: 
We have one class Robot that represents our robot and it has multiple methods representing its capabilities.
In the "loop" function of arduino we have "findBlackTile" that fulfills the robots mission OR we have the "test" method that allows us to test parts of the code with ease.

- The "findBlackTile" method. 
This method basically combines all the logic the robot is going to do. 
Meaning, it states the different tasks the robot is doing and calls the corresponding functions needed during this specific task. The flowchart for this method is below.\
![Flowchart of findBlackTile](/Ressources/Flowcharts/findBlackTile_Flowchart.png)


- The "test" method.
This method is there to test different parts of the code with more ease.

- The "exploreMazeRandom" method. As can be guessed by the name, this method will make the robot explore the maze on a random basis. 
If the black tile is not found along the perimeter wall, meaning it is somewhere inside the maze, we need to explore the maze to find it. However, we weren't able to figure out a strategy to do it, so we went with a random approach. We generated a random list of size 100 with 0 and 1. This list indicates how the robot will turn (1 for left and 0 for right if possible). 
We had to do this since HoRoSim doesn't support the Random library for Arduino.
The robot will choose a "random" turn if something is in its path. Otherwise, he either goes forward, right or left, depending on the possibily (if there is a wall to the side or nothing etc...).
The robot does this until he finds the black tile.
Below, the flowchart for this method.\
![Flowchat of exploreMazeRandom](/Ressources/Flowcharts/exploreMazeRandom_Flowchart.png)

- The "backToStart" method. We created this method to treat the task of the robot getting back to the starting point after random exploration (he is lost inside the maze around walls not connected to the perimeter wall). With a timer, we measure how long the robot has been following a specific wall and if it led him back to start. We use an enum type to switch between the states and therefore let the robot either follow the left wall or search for a new one. 
We added some tolerance to the timer to really make sure that he can complete a theoretic lap around the maze. Each lap of the robot will never be exactly the same so we need to take this into consideration.
Below the flowchart for this method.\
![Flowchart of backToStart](/Ressources/Flowcharts/backToStart_Flowchart.png)

- The "followWallLeft" & "followWallRight" functions. 
These are pretty self explaining. They allow the robot to follow the left and right wall respectively. 
Let us analyse "followWallLeft". 
If nothing is detected in the path of the robot we go forward. 
If something is in front of it, go right as we want to follow the left wall. 
If we loose sight of the left wall we turn left to find it again. 
This process is the same for the right wall except that we change the turn direction and use the sensors on the other side. Below the flowchart for "followWallLeft".\
![Flowchart of followWallLeft](/Ressources/Flowcharts/followWallLeft_Flowchart.png)

- The "mapTime" method. 
As the name suggests, this method will perform a test lap which will allow us to measure the time needed for 2 things during the simulation: the time needed to go from start to start by following the left wall and, if found along the perimeter wall, the time needed to get to the black tile. 
We print the result in the HoRoSim window and reset the values after the test lap so that everything works properly after.

- The "detection" method has one goal and that is to read the values of the proximity sensors and save it in the corresponding variables.

- The drive functions ("forward" / "turnLeft" / "turnRight" / "completeStop") do exactly what they suggest. They control the robot at speed given as an argument when we call the method.

- The "tileDetection" method.
This method detects on wich tile the robot is. There are white, red and black tiles, when on the two last ones, it updates the corresponding counters.
This method should be called as often as possible.

- The "lightSensor" method.
This method reads the value of the light sensor and returns a color from the enum color.

A note for all the times that we use the values from the proximity sensors. Since the values are all analog, we defined threshold values for the side, the front and the diagonal in order to make the line following as efficient as possible.


For the rest of the code we have one function "printColor" that prints a color from the enum color because the print function from arduino can't do it.
We have two enum, one "color" which has four values (RED, WHITE, BLACK and UNDEFIND), these are the colors found on the labyrinth.
The other enum is "GroundType" which has five values (BLACKTILE, WHITETILE, REDTILE, LINE and UNDEFINED), these are the different ground types found on the labyrinth.

## Conclusion
To conclude this documentation and therefore this project, we would like to add that it was a valuable experience in learning how to code such a project and think of different strategies that we could use so that the final goal could be achieved. Under normal circumstances, the robot achieves its goal. If the black tile is situated next to a perimeter wall, the process is fairly rapid. However, if the black tile is not along the perimeter, the task could take more time (c.f Video demonstration in "Ressources/Demonstration").
