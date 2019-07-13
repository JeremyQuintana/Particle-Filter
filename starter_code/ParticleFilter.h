
#ifndef COSC_ASS_ONE_PARTICLE_FILTER
#define COSC_ASS_ONE_PARTICLE_FILTER

#include "Particle.h"
#include "ParticleList.h"
#include "Types.h"

#define MAZE_SIZE          100

class ParticleFilter {
public:

   /*                                           */
   /* DO NOT MOFIFY ANY CODE IN THIS SECTION    */
   /*                                           */


   // Initialise a new particle filter with a given maze of size (x,y)
   ParticleFilter(char** maze, int rows, int cols);

   // Clean-up the Particle Filter
   ~ParticleFilter();

   // A new observation of the robot, of size 3x3
   void newObservation(Grid observation);

   // Return a DEEP COPY of the ParticleList of all particles representing
   //    the current possible locations of the robot
   ParticleList* getParticles();

   /*                                           */
   /* YOU MAY ADD YOUR MODIFICATIONS HERE       */
   void filter();

   //returns grid observation based on the particle passed in
   Grid getObsFromParticle(Particle* particle);

   //compares both observations if theyre the same value and returns boolean
   bool compareObservations(Grid obs1, Grid obs2);

   // Translate the observations arrow to an integer orientation
   int translateObsOrient();

   // Translates an integer to an arrow orientation character
   char translateObsDirection(int orientation);

   // Returns a new particle where the passed in particle is one move forward in a given direction
   ParticlePtr translateRobotForward(ParticlePtr initialParticle);

   // Returns grid of grid passed in rotated clockwise rotation number of times
   Grid rotateGridClockwise(Grid grid, int rotation);

   // Helper method to create 3x3 2d arrays
   Grid createGrid(int x, int y);

   /*                                           */
private:
  Grid          maze;
  int           rows;
  int           cols;
  ParticleList* particleList;
  Grid          observation;
  //true=milestone2, false=milestone3
  bool          milestone;
};

#endif // COSC_ASS_ONE_PARTICLE_FILTER
