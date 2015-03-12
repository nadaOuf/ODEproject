#ifndef _SPLINE_H
#define _SPLINE_H

#include "headers.h"

class Spline 
{
public:
	Spline();
	Spline(vector<glm::vec3> points);

	glm::vec3 getPointOnSpline(float t);

	glm::vec3 getPoint(int i);
	void addPoint(glm::vec3 point);

	~Spline();

private:
	vector<glm::vec3> m_vInterpolationPoints;
	vector<glm::vec3> m_vControlPoints;
	
	//Phantom Points
	glm::vec3 m_vStartPoint;
	glm::vec3 m_vEndPoint;

	void reSetPoints();
	void calcStartEndPoints();

	void generateSpline();

	glm::vec3 Lerp(glm::vec3 one, glm::vec3 two, float t);
};

#endif