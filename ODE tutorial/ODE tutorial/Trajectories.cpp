#include "Trajectories.h"

Trajectories::Trajectories()
{
	for( int i = 0; i < 4; ++i)
	{
		m_vTrajectories.push_back(new Spline());
		m_vPreviousPhase.push_back(0);
	}
}

Trajectories::~Trajectories()
{

}

void Trajectories::generateLegTrajectory(int legIndex, vector<glm::vec3> points)
{
	Spline* temp = new Spline(points);

	m_vTrajectories[legIndex] = temp;
	m_vPreviousPhase[legIndex] = 0;
}

float Trajectories::getLegHeight(int legIndex, float stridePhase)
{
	glm::vec3 point = m_vTrajectories[legIndex]->getPointOnSpline(stridePhase);

	return point.z;
}

float Trajectories::getLegPitch(int legIndex, float stridePhase)
{
	if(m_vPreviousPhase[legIndex] > stridePhase)
		m_vPreviousPhase[legIndex] = stridePhase;
	glm::vec3 point1 = m_vTrajectories[legIndex]->getPointOnSpline(m_vPreviousPhase[legIndex]);
	glm::vec3 point2 = m_vTrajectories[legIndex]->getPointOnSpline(stridePhase);

	glm::vec3 hip = point1 + glm::vec3(0, 0, 0.2);

	glm::vec3 v1 = hip - point1;
	glm::vec3 v2 = hip - point2;

	float dotProduct = glm::dot(v1, v2);
	float v1Length = sqrt(glm::dot(v1, v1));
	float v2Length = sqrt(glm::dot(v2, v2));
	float v1V2 = v1Length * v2Length;
	if(dotProduct == v1V2)
		return 0;

	
	float angle = - acos(dotProduct / (v1Length * v2Length));

	m_vPreviousPhase[legIndex] = stridePhase;

	return angle;
}

glm::vec3 Trajectories::getPointOnCurve(int legIndex, float stridePhase)
{
	return m_vTrajectories[legIndex]->getPointOnSpline(stridePhase);
}