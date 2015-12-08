#include "ClothSystem.h"
#include <math.h>
#include <iostream>
#include <map>

//TODO: Initialize here
ClothSystem::ClothSystem(int numParticles):ParticleSystem(pow(numParticles, 2))
{
	//set up the springs
	vector<Vector4f> springs2;
	Vector4f horizontal, vertical;
	for (int i = 0; i < numParticles; i++){
		for (int j = 0; j < numParticles; j++){
			//row i, column j
			if (j < numParticles && i < numParticles-1){
				horizontal = Vector4f((numParticles * j) + i, (numParticles * j) + i + 1, 0.5,5.0);
			}
			if (j < numParticles-1){
				vertical = Vector4f((j * numParticles) + i, (j * numParticles) + i + numParticles, 0.5,5.0);
			}
			springs2.push_back(horizontal);
			springs2.push_back(vertical);
		}

	}

	structural = springs2;
	springs = springs2;
	m_numParticles = numParticles;

	//shear springs
	Vector4f forwardSlash, backwardSlash;
	for (int i = 0; i < numParticles; i++){
		for (int j = 0; j < numParticles; j++){
			if (i < numParticles-1 and j < numParticles - 1){
				forwardSlash = Vector4f((numParticles * j) + i, (numParticles * j) + i + numParticles + 1, 0.5*pow(2,0.5), 5.0 );
			}
			if (i - 1 >= 0 and j < numParticles - 1){
				backwardSlash = Vector4f((numParticles * j) + i, (numParticles * j) + i + numParticles - 1, 0.5*pow(2,0.5),5.0);
			}
			shear.push_back(forwardSlash);
			shear.push_back(backwardSlash);
			springs.push_back(forwardSlash);
			springs.push_back(backwardSlash);
		}
	}

	//flexion
	Vector4f doubleHorizontal, doubleVertical;
	for (int i = 0; i < numParticles; i++){
		for (int j = 0; j < numParticles; j++){
			if (i < numParticles - 2){
				doubleHorizontal = Vector4f((numParticles * j) + i, (numParticles * j) + i + 2, 0.5*2, 5.0);
			}
			if (j < numParticles - 2){
				doubleVertical = Vector4f((numParticles * j) + i, (numParticles * j) + i + 2*numParticles, 0.5*2, 5.0);
			}
			flexion.push_back(doubleHorizontal);
			flexion.push_back(doubleVertical);
			springs.push_back(doubleHorizontal);
			springs.push_back(doubleVertical);
		}
	}

	for (int i = 0; i < numParticles; i++){
		for (int j = 0; j < numParticles; j++){

			m_vVecState.push_back(Vector3f(0.0f + j * 0.5f, 0.0f, i * 0.5f));
			m_vVecState.push_back(Vector3f(0.0f, 0.0f, 0.0f));
			part_col.push_back(false);
			col_norm.push_back(Vector3f(0,0,0));

		}
	}
	
}

void ClothSystem::move(){
	moving = not moving;
}

void ClothSystem::wireframe(){
	drawBool = not drawBool;
}

void ClothSystem::drawI(int i){
	springsForChosenParticle = springsConnected(i);
}


vector<Vector4f> ClothSystem:: springsConnected (int particleIndex){
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
vector<Vector3f> ClothSystem::evalF(vector<Vector3f> state)
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
		float k = 2.5f;
		float u_f = 0.3f;
		Vector3f  velocity3f = state[i+1];
		Vector3f viscousDrag = velocity3f * -k;
		Vector3f springForce = Vector3f(0.0f,0.0f,0.0f);
		vector<Vector4f> sprs = springsConnected(i/2);
		for (int j = 0; j < sprs.size(); j++){
			
			Vector4f spring = sprs[j];
			//spring.print();
			int p1Index = spring[0];
			int p2Index = spring[1];
			float rest = spring[2];
			float stiff = spring[3];
			Vector3f p1 = m_vVecState[2*p1Index];
			Vector3f p2 = m_vVecState[2*p2Index];
			Vector3f d = p1 - p2;
			
			
			springForce += -stiff*(d.abs() - rest) * (d/d.abs());
		}
		Vector3f fric = Vector3f(0,0,0);
		if (part_col[i/2]){
			Vector3f n = -col_norm[i/2];
			Vector3f n_force = Vector3f(0,0,0);
			if (n.abs() != 0){
				n_force = Vector3f().dot(gravity,n)/n.abs()*n;
			}
			n_force.negate();
			fric = u_f * n_force;
			Vector3f net = gravity + n_force + springForce;
			if (fric.abs() > net.abs()){
				fric = net;
			}
		}
		
		Vector3f finalForce = (gravity + viscousDrag + springForce - fric)/m;// gravity + viscousDrag + springForce;
		
		f.push_back(finalForce); 
		
	}
	
	//f[0] = Vector3f(0.0,0.0,0.5);
	f[1] = Vector3f(0.0,0.0,0.0);
	//f[2*pow(m_vVecState.size()/2,0.5)-2] = Vector3f(0.0,0.0,0.5);
	//f[2*pow(m_vVecState.size()/2,0.5)-1] = Vector3f(0.0,0.0,0.0);
	if (moving){
		//counter++;
		if (counter < 800){
			//move forward
			counter++;
			f[0] = Vector3f(-0.5,0.0,-0.5);
			//f[2*pow(m_vVecState.size()/2,0.5)-2] = Vector3f(0.0,0.0,1.0);
		}
		else if (counter < 1600){
			f[0] = Vector3f(0.5,0.0,0.5);
			//f[2*pow(m_vVecState.size()/2,0.5)-2] = Vector3f(0.0,0.0,-1.0);
			counter++;
		}
		if (counter >= 1600){
			counter = 0;
		}
	}

	return f;
}

///TODO: render the system (ie draw the particles)
void ClothSystem::draw()
{
	
	//Draw sphere
	//glPushMatrix();
	//glTranslatef(0,0,0);
	//glutSolidSphere(0.075f, 10.0f, 10.0f);
	//glPopMatrix();
//	naiveBVH();
	if (naiveBVH()){
		collision();
	}
	//std::cout << "first" << std::endl;

	for (int i = 0; i < m_vVecState.size()/2.; i++) {
		
		Vector3f pos = m_vVecState[2*i] ;//  position of particle i. YOUR CODE HERE
		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2] );
		glutSolidSphere(0.075f,10.0f,10.0f);
		glPopMatrix();
	}
	

	if (springsForChosenParticle.size() == 0){
		for (int i = 0; i < structural.size(); i++){
			
			//from opengl.org
			
				if (drawBool){
					glLineWidth(2.5);
					glColor3f(1.0,0.0,0.0);
					glBegin(GL_LINES);
					Vector3f point1 = m_vVecState[2*structural[i][0]];
					Vector3f point2 = m_vVecState[2*structural[i][1]];
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


void ClothSystem::collision(){
	Vector3f org = Vector3f(1.0f, -2.0f, 1.0f);
	float r = 1.0f;
	//std::cout << "here" << std::endl;
	
//	int counter = 0;
	for (int i = m_numParticles-1; i >= 0; i--){
		//std::cout << "here" << std::endl;
		for (int j = 0; j < m_numParticles; j++){
//			counter++;
			Vector3f part = m_vVecState[i*m_numParticles+j*2];
			float pos = pow(-org.x()+part.x(),2)+pow(-org.y()+part.y(),2)+pow(-org.z()+part.z(),2);
	//		std::cout << pos << std::endl;
			if (pos < pow(r,2)){
				Vector3f unit = (part-org).normalized();
				Vector3f newPos = r*unit + org;
				m_vVecState[i*m_numParticles+j*2] = newPos;
				part_col[i*m_numParticles+j] = true;
				col_norm[i*m_numParticles+j] = unit;
			}
			else{
				
				part_col[i*m_numParticles+j] = false;
			}

		}
	}
	cerr << counter << "\n";

}

bool ClothSystem::naiveBVH(){

	float staticSphereRadius = 1.0f;
	Vector3f staticSphereCenter = Vector3f(1.0f,-2.0f,1.0f);

	Vector3f sphereCenter;
	Vector3f firstParticlePos = m_vVecState[0];
	Vector3f lastParticlePos = m_vVecState[2 * (pow(m_numParticles,2) - 1)];
	

	
	if (m_numParticles % 2 == 0){
		float centerX = (firstParticlePos.x() + lastParticlePos.x())/2.0f;
		float centerY = (firstParticlePos.y() + lastParticlePos.y())/2.0f;
		float centerZ = (firstParticlePos.z() + lastParticlePos.z())/2.0f;

	//	cerr << "CENTER " << centerX << " " << centerY << " " << centerZ << "\n";		
	//	cerr << "FIRST " << firstParticlePos.y() << " " << lastParticlePos.y() << "\n";

		sphereCenter = Vector3f(centerX, centerY, centerZ);
	}
	else{
		Vector3f sphereCenter = m_vVecState[pow(m_numParticles,2.0f) - 1];
//		cerr << "CENTER " << lastParticlePos.x() << " " << lastParticlePos.y() << " " << lastParticlePos.z() << "\n";
	}	
	float firstPow = pow(lastParticlePos.x() - sphereCenter.x(),2.0f);
	float secondPow = pow(lastParticlePos.y() - sphereCenter.y(),2.0f);
	float thirdPow = pow(lastParticlePos.z() - sphereCenter.z(),2.0f);
	float sphereRadius = sqrt(firstPow + secondPow + thirdPow);
	
	Vector3f distanceVector = sphereCenter - staticSphereCenter;

	float distanceBetweenCenters = distanceVector.abs();
	float radiiSum = staticSphereRadius + sphereRadius;
	
	//cerr << radiiSum << " -------- " << distanceBetweenCenters << "\n";

	if (radiiSum > distanceBetweenCenters){
	
		cerr << "TRUE\n";
		return true;
	}
	//cerr << "FALSE\n";
	return false;

}

//maps the sphere to the indicies of the particles it contains
//sphere is determined by the position of the vector
//i.e. 3rd entry is 3rd bounding sphere going from left to right, top to bottom
void ClothSystem::sphereToParticleIndicies(){

	//under the assumption we will only have 3 levels (including overall sphere as level 1)
	//since our simulation will work max 8x8, 3 levels is 1 level before we're just checking particles

	//this code is highly dependent on 8x8 system
	
	
	
	if (m_numParticles % 2 == 0){
		float counter = 3.0;
		//building the 16 subspheres
		map<float, vector<int>> level3ToIndicies;
		for (int j = 0; j < m_numParticles - 1; j += 2){
			
			for (int i = 0; i < m_numParticles - 1; i += 2){
				vector<int> subsphereIndices;
				subsphereIndices.push_back(j * m_numParticles + i * 2);
				subsphereIndices.push_back((j + 1) * m_numParticles + i * 2);
				subsphereIndices.push_back((j + 1) * m_numParticles + (i + 1) * 2);
				subsphereIndices.push_back(j * m_numParticles + (i + 1) * 2);
				level3ToIndicies[counter] = subsphereIndices;
				counter += 0.1;
			}
			
		}
		
		//mapping 4 medium subspheres to 16 subspheres INDICIES
		map<float, vector<int>> level2ToLevel3;
		float counter2 = 2.0;
		for (int j = 0; j < 3; j+=2){
			for (int i = 0; i < 3; i+=2){
				
				vector<int> level3Indicies;
				level3Indicies.push_back(3.0 + 0.1 * (j * sqrt(level3ToIndicies.size()) + i));
				level3Indicies.push_back(3.0 + 0.1 * (j+1 * sqrt(level3ToIndicies.size()) + i));
				level3Indicies.push_back(3.0 + 0.1 * (j * sqrt(level3ToIndicies.size()) + i+1));
				level3Indicies.push_back(3.0 + 0.1 * (j+1 * sqrt(level3ToIndicies.size()) + i));
				level2ToLevel3[counter2] = level3Indicies;
				counter2 += 0.1;
			}
		}
		
		//mapping 1 large sphere to 4 medium subspheres
		map<float, vector<int>> level1ToLevel2;
		vector<int> level2Indicies;
		level2Indicies.push_back(2.0);
		level2Indicies.push_back(2.1);
		level2Indicies.push_back(2.2);
		level2Indicies.push_back(2.3);
		level1ToLevel2[1.0] = level2Indicies;
		
	}
	else{

	}
}



















