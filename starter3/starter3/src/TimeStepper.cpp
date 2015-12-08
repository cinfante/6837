#include "TimeStepper.h"

///TODO: implement Explicit Euler time integrator here
void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	vector<Vector3f> oldState = particleSystem -> getState();
	vector<Vector3f> newState;
	for (int i = 0; i < oldState.size(); i++){
		Vector3f newVector = oldState[i] + stepSize * particleSystem -> evalF(oldState)[i];
		newState.push_back(newVector);
	}
	particleSystem -> setState(newState);
	
}

///TODO: implement Trapzoidal rule here
void Trapzoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
	vector<Vector3f> oldState = particleSystem -> getState();
	vector<Vector3f> newState;
	
	vector<Vector3f> intermediaryState;
	for (int i = 0; i < oldState.size(); i++){
		Vector3f toAdd = Vector3f(oldState[i] + stepSize * particleSystem -> evalF(oldState)[i]);
		intermediaryState.push_back(toAdd);
	}
	
	for (int i = 0; i < oldState.size(); i++){
		Vector3f f0 = particleSystem -> evalF(oldState)[i];
		
		Vector3f f1 = particleSystem ->evalF(intermediaryState)[i];
		newState.push_back(oldState[i] + (stepSize/2.0f) * (f0 + f1));
	}

	
	particleSystem -> setState(newState);
}
