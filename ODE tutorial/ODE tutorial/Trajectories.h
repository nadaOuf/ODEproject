#ifndef _TRAJECTORIES_H
#define _TRAJECTORIES_H
#include "headers.h"
#include "Spline.h"

class Trajectories
{
public:
	Trajectories();
	~Trajectories();
	
	void generateLegTrajectory(int legIndex, vector<glm::vec3> points);

	float getLegHeight(int legIndex, float stridePhase);
	float getLegPitch(int legIndex, float stridePhase);

	glm::vec3 getPointOnCurve(int legIndex, float stridePhase);

	enum SKELETON {RL, RR, FL, FR};

private:
	vector<Spline*> m_vTrajectories;

	vector<float> m_vPreviousPhase;
};
#endif