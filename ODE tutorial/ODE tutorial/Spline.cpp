#include "Spline.h"

Spline::Spline() : m_vStartPoint(0, 0, 0), m_vEndPoint(0, 0, 0)
{
}

Spline::Spline(vector<glm::vec3> points)
{
	m_vInterpolationPoints = points;
	generateSpline();
}

Spline::~Spline()
{

}

glm::vec3 Spline::getPointOnSpline(float t)
{
	if(t == 0)
		return m_vInterpolationPoints[0];
	//Get the index of the first point
	int i = floor(t);
	glm::vec3 b0  = m_vControlPoints[4*i];
	glm::vec3 b1 = m_vControlPoints[4*i+1];
	glm::vec3 b2 = m_vControlPoints[4*i+2];
	glm::vec3 b3 = m_vControlPoints[4*i+3];

	float correct_t = t -i;
	glm::vec3 b10 = Lerp(b0, b1, correct_t);
	glm::vec3 b12 = Lerp(b1, b2, correct_t);
	glm::vec3 b23 = Lerp(b2, b3, correct_t);

	glm::vec3 b20 = Lerp(b10, b12, correct_t);
	glm::vec3 b21 = Lerp(b12, b23, correct_t);

	return Lerp(b20, b21, correct_t);
}

void Spline::reSetPoints()
{
	m_vStartPoint = glm::vec3(0, 0, 0);
	m_vEndPoint = glm::vec3(0, 0, 0);
	m_vControlPoints.clear();
}

glm::vec3 Spline::getPoint(int i)
{
	return m_vInterpolationPoints[i];
}

void Spline::addPoint(glm::vec3 point) //used to add points in order
{
	m_vInterpolationPoints.push_back(point);
	generateSpline();
}

glm::vec3 Spline::Lerp(glm::vec3 one, glm::vec3 two, float t)
{
	return one * t + (1-t) * two;
}

void Spline::calcStartEndPoints()
{
	int n = m_vInterpolationPoints.size();
	if( n < 2)
		return;

	//Get the first and second points
	glm::vec3 p0 = m_vInterpolationPoints[0];
	glm::vec3 p1 = m_vInterpolationPoints[1];

	//Calculate the start point
	glm::vec3 v10 = p0 - p1;
	//v10 = glm::normalize(v10);
	m_vStartPoint = p0 + v10 * 10.0f;

	//Get the last two points
	glm::vec3 pn_1 = m_vInterpolationPoints[n - 2];
	glm::vec3 pn = m_vInterpolationPoints[n - 1];

	v10 = pn - pn_1;
	//v10 = glm::normalize(v10);

	m_vEndPoint = pn + v10 * 10.0f;
}

void Spline::generateSpline()
{
	//reSet any saved points
	reSetPoints();

	//Set the start and end points
	calcStartEndPoints();

	glm::vec3 pi(0, 0, 0), pi_1(0, 0, 0), pi_2(0, 0, 0), pi_3(0, 0, 0);
	//Calculate the control points
	//This code only works if the number of points is >= 2
	for( int i = 0; i < m_vInterpolationPoints.size() - 1; ++i)
	{
		pi = m_vInterpolationPoints[i];
		pi_1 = (i == 0 ? m_vStartPoint : m_vInterpolationPoints[i-1]);
		pi_2 = m_vInterpolationPoints[i+1];
		pi_3 = (i == m_vInterpolationPoints.size() - 2 ? m_vEndPoint : m_vInterpolationPoints[i+2]); 

		glm::vec3 controlPoint1(pi), controlPoint2(pi + (1.0f/6.0f) * (pi_2 - pi_1)), controlPoint3(pi_2 - (1.0f/6.0f) * (pi_3 - pi)), controlPoint4(pi_2);

		m_vControlPoints.push_back(controlPoint1);
		m_vControlPoints.push_back(controlPoint2);
		m_vControlPoints.push_back(controlPoint3);
		m_vControlPoints.push_back(controlPoint4);
	}
}