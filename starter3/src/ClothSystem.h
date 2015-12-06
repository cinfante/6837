#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include "extra.h"
#include <vector>

#include "particleSystem.h"

class ClothSystem: public ParticleSystem
{
///ADD MORE FUNCTION AND FIELDS HERE
public:
	ClothSystem(int numParticles);
	vector<Vector3f> evalF(vector<Vector3f> state);
	vector<Vector4f> springsConnected(int particleIndex);
	void draw();
	void move();
	void wireframe();
	void drawI(int i);
	void collision();

private:
	bool moving = false;
	int counter = 0;
	bool drawBool = true;
	vector<bool> part_col;
	vector<Vector3f> col_norm;
};

#endif
