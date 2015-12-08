#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include "extra.h"
#include <vector>

#include "particleSystem.h"

class PendulumSystem: public ParticleSystem
{
public:
	PendulumSystem(int numParticles);
	
	vector<Vector3f> evalF(vector<Vector3f> state);

	vector<Vector4f> springsConnected(int particleIndex);
	
	void draw();
	void move();	
	void wireframe();
	void drawI(int i);
};

#endif
