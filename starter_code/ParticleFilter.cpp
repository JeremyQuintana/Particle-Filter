/*
3.2.4
  Approach
    Whenever the newObservation method is called filter is also called to update the ParticleList
    In the filter method it creates a new ParticleList holding all possible particles after one movement
    If there are no current particles in the particles list it generates possible positions in the maze with all orientations
    If there are particles in particleList if gets the possible particles it takes each particle from ParticleList
    Each particle obtained from particle list is read and compared to the observation
    If the direction changed then all particcles are generated with same coordinates of those in particleList but with a new orientation
    If the direction didnt change then it moves one forward based on the direction of the robot
    The ParticleList is then cleared
    These particles from the new ParticleList are then turned into observations
    Observations are compared to the observation recieved
    If theyre the same they are deep copied into the newly cleared particle list
  Issues
    Random values read back from particles array due to size of array being too small
    predParticleList not having initial values to read and predict robot movement From
    Rotating the robot to a specific orientation given its current orientation e.g. 3 to 0 and 0 to 3
    Creating a new observation to compare to which was solved by using the createGrid code from Timothy Wiley
    How to create reusable code that returns a particle that is one coordinate forward in a direction of the robot
    Turning orientations from observation to a readable format
    Getting maze values surrounding a coordinate to create an observation
    Initial given size of array for particleList was too small to store all particles using my Approach
    Getting initial predicted particles as there is initially no particlea in particleList to derrive predictions from
  Justify Choices
    Seperate and remove cohesion as much as logically possible for code reuse and readability
    Creating a predParticleList enabled me to add particles and compare them at a later time in my code to reduce cohesion
    Turning predicted particles to observations and comparing the observations removed the need to constantly iterate through an observation
    Use only one return statment to increase code quality
    Make lines less than 80 characters long to increase readability
    Handling the predicted particle list at the end of the filter code and imputing them into the particle list enables for easy modifications to code
    With predParticleList I did not need to keep searching through particleList for a given value to delete but instead clear it all and plac enew values
  Analyse Efficiency & Quality
    In terms of storing particles within the particleList, it is reduced due to checking before entering the particles into the List
    Quality in terms of seperating functions enable simple readability although can be improved by seperating the generation of predicted particles
    With each observation it needs to check if the particleList is empty so it can generate predicted particles slows program down with another if statement
    Creating particle for different orientations clockwise and counter clockwise could have been reduced in size but would also decrease efficiency due to if statements
    Does not include multiple return statemetns to imporve quality
    Use of defines to increase code readability
    Store variables in class variables that requires use in multiple classes to prevent variables with the same value in stored in memory at the same time
    Reduced search times as everything is only searched once then stored in variables like observations of particles

3.4.1
  Approach
    Implement rotation to observations maintaining core approach of program
    Made the array dynamic to reduce amount of memory reserved for the program
    Checks with each observation if the observation is of '*' or arrow to run relevant Code
    The reduced cohesion enabled for a simple relatively small if statement and produce relevant predicted particles
    Created a deep copy of the maze to keep in the class until its finished to ensure encapsulation in case other classes alter the maze
    Delete predicted particle list after each new observation to clear space in memory
    Placed moving robot forward to a function to improve code reusability
    Created copy constructors so that I could delete all particles in the predicted particle list and copy particles over to the proper particle list
    Deletes ParticleList and creates a new one with array size of predicted particle list
  Justify
    Constantly reduces array size of predicted particle list with each observation
    Use of comparing observations means that the maze is not continuously searched
    By deep copying the maze it no longer needs to be searching on the heap
    Stores whether maze is of milestone 2 or 3 so that it doesnt need to constantly search through observation on heap
    Deletes observation after filtering new particles to save memory
    Does not check through each particle in the maze after new observation but takes previous particles and takes its possible moves reducing predParticleList
    Delete grids created after comparing observations to free up space for next comparison
    Deletes particleList and creates a new ParticleList with dynamix arraysize of amount in predicted particleList as it is the maximum amount it can store
*/

#include "ParticleFilter.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <string.h>

#define OBSERVATION_SIZE    3
#define POSSIBLE_ROTATIONS  4
#define WALL_REDUCTION      2
#define POSSIBLE_MOVES      3

// Initialise a new particle filter with a given maze of size (x,y)
ParticleFilter::ParticleFilter(char** maze, int rows, int cols) {
  this->rows = rows;
  this->cols = cols;
  particleList = new ParticleList
    ((rows-WALL_REDUCTION)*(cols-WALL_REDUCTION)*POSSIBLE_ROTATIONS);

  // Creates deep copy of maze
  this->maze = createGrid(cols, rows);
  for(int y=0; y<rows; y++){
    for(int x=0; x<cols; x++){
      this->maze[y][x] = maze[y][x];
    }
  }
}

// Clean-up the Particle Filter
ParticleFilter::~ParticleFilter() {
  //As a deep copy was made I must delete it
  delete[] maze;
  maze = NULL;
}

// A new observation of the robot, of size 3x3
void ParticleFilter::newObservation(Grid observation) {
  this->observation = observation;

  milestone = true;
  if(observation[1][1] == '*') milestone = false;
  filter();

  delete[] observation;
}

// Return a DEEP COPY of the ParticleList of all particles representing the current possible locations of the robot
ParticleList* ParticleFilter::getParticles() {
  ParticleList* copiedParticleList = new ParticleList(*particleList);
  return copiedParticleList;
}

/*Filters particleList to reflect new possible particles with new observation
Turn new possible particles into possible observations
Compare observation with possible observations and keep similar ones
Return similar particles*/
void ParticleFilter::filter(){
  // New object is temporary to store all possible particles
  ParticleList* predParticleList = NULL;

  // Checks if the particleList is empty
  if(particleList->getNumberParticles() == 0){
    predParticleList = new ParticleList
      ((rows-WALL_REDUCTION)*(cols-WALL_REDUCTION)*POSSIBLE_ROTATIONS);
    // Goes through each coordinate in maze
    for(int x=0; x<cols; x++){
      for(int y=0; y<rows; y++){
        // If the coordinate in the maze has character '.' then
        if(maze[y][x] == '.'){
          // Creates new particle with all 4 possible orientations
          // Adds particle to the temporary particle list
          for(int o=0; o<POSSIBLE_ROTATIONS; o++){
            ParticlePtr predParticle = new Particle(x, y, o);
            predParticleList->add_back(predParticle);
          }
        }
      }
    }
  }

  // If the particleList contains particles
  else{
    predParticleList = new ParticleList
      (particleList->getNumberParticles()*POSSIBLE_MOVES);
    // Checks if the observation is of milestone2 or milestone 3
    // Generates milestone 3 particles
    if(milestone == false){
      // Goes through each particle in particleList
      for(int i=0; i<particleList->getNumberParticles();i++){
        ParticlePtr prevParticle = particleList->get(i);

        // Adds to the predParticleList a particle where the robot moves one forward
        // Before adding to predParticleList it checks if the paticle is a valid position
        ParticlePtr predParticle = translateRobotForward(particleList->get(i));
        if(maze[predParticle->getY()][predParticle->getX()] == '.'){
          predParticleList->add_back(predParticle);
        }
        else delete predParticle;

        // Changes the orientation one clockwise and one anticlockwise then adds the new particles to the predParticleList
        int orientation = prevParticle->getOrientation();
        orientation++;
        if(orientation == ORIEN_DOWN+1) orientation = ORIEN_LEFT;
        predParticle = new Particle
          (prevParticle->getX(), prevParticle->getY(), orientation);
        predParticleList->add_back(predParticle);
        for(int i=0; i<2 ; i++){
        orientation--;
        if(orientation == ORIEN_LEFT-1) orientation = ORIEN_DOWN;
        }
        predParticle = new Particle
          (prevParticle->getX(), prevParticle->getY(), orientation);
        predParticleList->add_back(predParticle);
      }
    }
    // Generating milestone2 possibleParticles
    else{
      // If it didnt change then it moved forward one step
      if(particleList->get(0)->getOrientation() == translateObsOrient()){
        // For each particle in particleList generates a new particle one step forward
        // Direction of the step is found out by observation orientation
        for(int i=0; i<particleList->getNumberParticles();i++){
          ParticlePtr predParticle =
            translateRobotForward(particleList->get(i));
          if(maze[predParticle->getY()][predParticle->getX()] == '.'){
            predParticleList->add_back(predParticle);
          }
          else delete predParticle;
        }
      }
      else{
        // Generates particles to add to predParticleList with same as particleList coordinates but updated orientation
        for(int i=0; i<particleList->getNumberParticles(); i++){
          ParticlePtr prevParticle = particleList->get(i);
          ParticlePtr predParticle = new Particle
            (prevParticle->getX(), prevParticle->getY(), translateObsOrient());
          predParticleList->add_back(predParticle);
        }
      }
    }
  }

  // Deletes particle list and creates a new one with an array size of amount of particles within predicted particleList
  delete particleList;
  particleList = new ParticleList(predParticleList->getNumberParticles());

  for(int i=0; i<predParticleList->getNumberParticles(); i++){
    // Turns particles in predParticleList to observations
    Grid predictedObservation = getObsFromParticle(predParticleList->get(i));
    // Compares those predicted observations to the current observations
    if(compareObservations(observation, predictedObservation) == true){
      // If it matches then it adds it to the cleared particleList
      ParticlePtr possibleParticle = new Particle(*(predParticleList->get(i)));
      particleList->add_back(possibleParticle);
    }
    // As the predictedObservation is a new Grid it must be deleted after use
    delete predictedObservation;
  }
  // As I copied the values from the pointers in predParticles to particleList I can delete predParticleList
  delete predParticleList;
}

// Returns a grid from passed in particle
Grid ParticleFilter::getObsFromParticle(ParticlePtr particle){
  Grid grid = createGrid(OBSERVATION_SIZE, OBSERVATION_SIZE);

  // From particle coordinates creates grid from maze
  for(int y=0; y<OBSERVATION_SIZE; y++){
    for(int x=0; x<OBSERVATION_SIZE; x++){
      grid[y][x] = maze[particle->getY()-1+y][particle->getX()-1+x];
    }
  }

  // If the current map is of milestone 3
  if(milestone == false){
    // Places '*' to center of grid
    grid[1][1] = '*';

    // Gets orientation of particle and alligns up to maps relative up
    int orien = particle->getOrientation();
    if(orien == ORIEN_LEFT) grid = rotateGridClockwise(grid,1);
    if(orien == ORIEN_UP) grid = rotateGridClockwise(grid,0);
    if(orien == ORIEN_RIGHT) grid = rotateGridClockwise(grid,3);
    if(orien == ORIEN_DOWN) grid = rotateGridClockwise(grid,2);
  }
  // If of milestone 2 places relevant arrow to center of grid
  else grid[1][1] = translateObsDirection(particle->getOrientation());
  return grid;
}

// Creates and returns a grid
// Code from Timothy Wiley used within Unit_Test.cpp
Grid ParticleFilter::createGrid(int x, int y){
  Grid grid = NULL;
  grid = new char*[y];
  for (int i = 0; i != y; ++i) {
    grid[i] = new char[x];
  }

  return grid;
}

// Compare two observations
bool ParticleFilter::compareObservations(Grid obs1, Grid obs2){
  bool same = true;

  // Same becomes false once it finds that any coordinate which is being iterated through is not the same
  for(int y=0; y<OBSERVATION_SIZE; y++){
    for(int x=0; x<OBSERVATION_SIZE; x++){
      if(obs1[y][x] != obs2[y][x]) same = false;
    }
  }
  return same;
}

// Translates orientation from observation to int
int ParticleFilter::translateObsOrient(){
  int orientation = ORIEN_LEFT;
  if(observation[1][1] == '<') orientation = ORIEN_LEFT;
  if(observation[1][1] == '^') orientation = ORIEN_UP;
  if(observation[1][1] == '>') orientation = ORIEN_RIGHT;
  if(observation[1][1] == 'v') orientation = ORIEN_DOWN;

  return orientation;
}

// Translates passed in int to orientation
char ParticleFilter::translateObsDirection(int orientation){
  char direction = '<';
  if(orientation == ORIEN_LEFT) direction = '<';
  if(orientation == ORIEN_UP) direction = '^';
  if(orientation == ORIEN_RIGHT) direction = '>';
  if(orientation == ORIEN_DOWN) direction = 'v';

  return direction;
}

// Returns a ParticlePtr that is one step forward in the direction the initalParticle is facing
ParticlePtr ParticleFilter::translateRobotForward(ParticlePtr initialParticle){
  int movement = 0;
  int orientation = initialParticle->getOrientation();
  // Determines if its a negative or positive movement in the x or y direction
  if(orientation == ORIEN_LEFT || orientation == ORIEN_UP) movement = -1;
  if(orientation == ORIEN_RIGHT || orientation == ORIEN_DOWN) movement = 1;

  int translateX = movement;
  int translateY = movement;
  // Determines if its a movement in the x or y axis
  if(orientation == ORIEN_LEFT || orientation == ORIEN_RIGHT) translateY = 0;
  if(orientation == ORIEN_UP || orientation == ORIEN_DOWN) translateX = 0;

  int x = initialParticle->getX() + translateX;
  int y = initialParticle->getY() + translateY;
  ParticlePtr newParticle = new Particle(x, y, orientation);
  return newParticle;
}

// Rotates past in grid clockwise 90 degrees
// Explanation
// As the loop iterates through initial grid the loop iterates through the rows of rotateGrid
// Hence it places the columns of initial grid to the rows of rotateGrid
Grid ParticleFilter::rotateGridClockwise(Grid grid, int rotation){
  Grid rotateGrid = createGrid(OBSERVATION_SIZE, OBSERVATION_SIZE);
  Grid initialGrid = grid;
  int rotateGridX = 0;
  int rotateGridY = 0;

  // Creates deep copy of grid into rotated so when grid is altered rotateGrid is not
  for(int x=0; x<OBSERVATION_SIZE; x++){
    for(int y=0; y<OBSERVATION_SIZE; y++){
      rotateGrid[y][x] = grid[y][x];
    }
  }
  // For loop to loop until amount of rotations are reached
  for(int i=0; i<rotation; i++){
    // Variables of coordinate to store rotated grid value
    rotateGridX = OBSERVATION_SIZE - 1;
    rotateGridY = 0;

    // For loops goes through each variable in the initial grid
    for(int x=0; x<OBSERVATION_SIZE; x++){
      for(int y=0; y<OBSERVATION_SIZE; y++){
        rotateGrid[rotateGridY][rotateGridX] = initialGrid[y][x];
        rotateGridX--;
      }
      rotateGridY++;
      rotateGridX = OBSERVATION_SIZE - 1;
    }

    // For loops to make a deep copy so that when rotateGrid is altered for rotation initialGrid is not
    for(int x=0; x<OBSERVATION_SIZE; x++){
      for(int y=0; y<OBSERVATION_SIZE; y++){
        initialGrid[y][x] = rotateGrid[y][x];
      }
    }
  }
  // No longer needs the initialGrid
  delete initialGrid;
  return rotateGrid;
}
