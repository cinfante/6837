
#include "simpleSystem.h"
#include <iostream>

using namespace std;

SimpleSystem::SimpleSystem()
{
	m_vVecState.push_back(Vector3f(1.0f,1.0f,1.0f));
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> SimpleSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;

	//loop through vector of Vector3fs, add -y, x, 0
	
	for (int i = 0; i < state.size(); i++){
		Vector3f currentState = state[i];
		float currentX = currentState[0];
		float currentY = currentState[1];
		
		Vector3f newState = Vector3f(-1.0f*currentY, currentX, 0.0f);
		f.push_back(newState);
	}

	return f;
}

void SimpleSystem::move(){
	
}

void SimpleSystem::wireframe(){
	
}

void SimpleSystem::drawI(int i){
	
}

// render the system (ie draw the particles)
void SimpleSystem::draw()
{
	glutSolidSphere(0.1f,10.0f,10.0f);

		Vector3f pos  = m_vVecState[0];//YOUR PARTICLE POSITION
	  glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
}
