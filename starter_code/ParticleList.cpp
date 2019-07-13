
#include "ParticleList.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>


#ifndef DEBUG
#define DEBUG        0
#endif

// Initialise a new particle filter with a given maze of size (x,y)
ParticleList::ParticleList(){
  // Initialise all particles in array to NULL
  for(int i=0; i<arraySize; i++){
    particles[i] = NULL;
  }
  numParticles = 0;
}

//Constructor for dynamic array and memory managemtent
ParticleList::ParticleList(int arraySize){
  this->arraySize = arraySize;

  //Initialise particles as a ParticlePtr of size passed in array
  particles = new ParticlePtr[arraySize];
  //Initialises all particles in array to NULL
  for(int i=0; i<arraySize; i++){
    particles[i] = NULL;
  }
  numParticles = 0;
}

//Copy constructor
ParticleList::ParticleList(ParticleList& previousList){
  this->arraySize = previousList.getArraySize();
  particles = new ParticlePtr[arraySize];
  numParticles = 0;
  for(int i=0; i<previousList.getNumberParticles(); i++){
    ParticlePtr newParticle = new Particle(*(previousList.get(i)));
    add_back(newParticle);
  }
}

// Clean-up the particle list
ParticleList::~ParticleList(){
  delete[] particles;
  particles = NULL;
  numParticles = 0;
}

// Number of particles in the ParticleList
int ParticleList::getNumberParticles(){
  return numParticles;
}

// Get a pointer to the i-th particle in the list
ParticlePtr ParticleList::get(int i){
  return particles[i];
}

// Add a particle (as a pointer) to the list
//    This class now has control over the pointer
//    And should delete the pointer if the particle is removed from the list
void ParticleList::add_back(ParticlePtr particle){
  particles[numParticles] = particle;
  numParticles++;
}

// Remove all particles from the list
void ParticleList::clear(){
  for(int i=0; i<numParticles; i++){
    delete particles[i];
  }
  numParticles = 0;
}

// Returns arraySize
int ParticleList::getArraySize(){
  return arraySize;
}
