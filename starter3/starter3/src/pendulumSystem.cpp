
#include "pendulumSystem.h"
#include <iostream>

PendulumSystem::PendulumSystem(int numParticles):ParticleSystem(numParticles)
{
	m_numParticles = numParticles;
	
	// fill in code for initializing the state based on the number of particles
	for (int i = 0; i < m_numParticles; i++) {
		
		// for this system, we care about the position and the velocity
		m_vVecState.push_back(Vector3f(-1.0f * i, 0.0f, 0.0f));
		m_vVecState.push_back(Vector3f(0.0f,0.0f,0.0f));
/* 		m_vVecState.push_back(Vector3f(-1.0f,0.0f,0.0f)); //static for simple pendulum //pos p1
		m_vVecState.push_back(Vector3f(0.0f,0.0f,0.0f)); //vel p1
		m_vVecState.push_back(Vector3f(-3.0f,0.0f,0.0f)); //pos p2
		m_vVecState.push_back(Vector3f(-1.0f,0.0f,0.0f)); //vel p2
		m_vVecState.push_back(Vector3f(-3.5,0.0,0.0));
		m_vVecState.push_back(Vector3f(0.0,0.0,0.0));
		m_vVecState.push_back(Vector3f(-4.0,0.0,0.0));
		m_vVecState.push_back(Vector3f(0.0,0.0,0.0)); */
		if (i + 1 < m_numParticles){
			springs.push_back(Vector4f(i+1, i, 0.05f, 1.0f));
		}
	}
	
	//set up springs here
//	vector<Vector4f> springs2;
/* 	springs.push_back(Vector4f(1.0f,0.0f,0.05f,1.0f));
	springs.push_back(Vector4f(2.0f,1.0f,0.05f,1.0f));
	springs.push_back(Vector4f(3.0f,2.0f,0.05f,1.0f)); */
//	springs = springs2;
	
}

void PendulumSystem::move(){
	
}

void PendulumSystem::wireframe(){
	drawBool = not drawBool;
}

void PendulumSystem::drawI(int i){
	springsForChosenParticle = springsConnected(i);
}

vector<Vector4f> PendulumSystem:: springsConnected (int particleIndex){
	vector<Vector4f> connectedSprings;
	for (int i = 0; i < springs.size(); i++){
		if ((int) springs[i][0] == particleIndex){
			connectedSprings.push_back(springs[i]);
		}
		else if ((int) springs[i][1] == particleIndex){
			Vector4f newSpring = Vector4f(springs[i][1], springs[i][0], springs[i][2], springs[i][3]);
			connectedSprings.push_back(newSpring);
		}
	}
	return connectedSprings;
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f> PendulumSystem::evalF(vector<Vector3f> state)
{
	vector<Vector3f> f;
	
	for (int i = 0; i < state.size() - 1; i += 2){
		f.push_back(state[i+1]); // v
		
		//gravity
		float m = 0.05f;
		float g = -9.8f;
		float gravF = m*g;
		Vector3f gravity = Vector3f(0.0f, gravF, 0.0f); //affects y
		
		//viscous drag
		float k = 0.5f; //random coeff for sphere found on wikipedia
		Vector3f  velocity3f = state[i+1];
		Vector3f viscousDrag = velocity3f * -k;
		
		
		Vector3f springForce = Vector3f(0.0f,0.0f,0.0f);
		vector<Vector4f> sprs = springsConnected(i/2);
		cerr << sprs.size() << "\n";
		for (int j = 0; j < sprs.size(); j++){
			
			Vector4f spring = sprs[j];
			spring.print();
			int p1Index = spring[0];
			int p2Index = spring[1];
			float rest = spring[2];
			float stiff = spring[3];
			Vector3f p1 = m_vVecState[2*p1Index];
			Vector3f p2 = m_vVecState[2*p2Index];
			Vector3f d = p1 - p2;
			
			springForce += -stiff*(d.abs() - rest) * (d/d.abs());
			
		}
		
		Vector3f finalForce = (gravity + viscousDrag + springForce)/m;
		f.push_back(finalForce); 
		
	}
	f[1] = Vector3f(0.0,0.0,0.0);
	return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw()
{
	for (int i = 0; i < m_numParticles; i++) {
		
		Vector3f pos = m_vVecState[2*i] ;//  position of particle i. YOUR CODE HERE
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
//	cerr << springs.size() << "\n";

	if (springsForChosenParticle.size() == 0){
		for (int i = 0; i < springs.size(); i++){
			
			//from opengl.org
			
				if (drawBool){
					glLineWidth(2.5);
					glColor3f(1.0,0.0,0.0);
					glBegin(GL_LINES);
					Vector3f point1 = m_vVecState[2*springs[i][0]];
					Vector3f point2 = m_vVecState[2*springs[i][1]];
					glVertex3f(point1[0], point1[1], point1[2]);
					glVertex3f(point2[0], point2[1], point2[2]);
					glEnd();
				}
			}
				
		
	}
	else{
		for (int i = 0; i < springsForChosenParticle.size(); i++){
			glLineWidth(2.5);
					glColor3f(1.0,0.0,0.0);
					glBegin(GL_LINES);
					Vector3f point1 = m_vVecState[2*springsForChosenParticle[i][0]];
					Vector3f point2 = m_vVecState[2*springsForChosenParticle[i][1]];
					glVertex3f(point1[0], point1[1], point1[2]);
					glVertex3f(point2[0], point2[1], point2[2]);
					glEnd();
		}
	}

}
