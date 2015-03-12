#include "Controller.h"

Controller::Controller()
{
}

Controller::Controller(float kp, float kv)
{
	m_fKp = kp;
	m_fKv = kv;
}

Controller::~Controller()
{
}

glm::vec3 Controller::calculateControllerOutput(glm::vec3 kpTerm, glm::vec3 kvTerm)
{
	glm::vec3 result = glm::vec3(0,0,0);

	result = m_fKp*kpTerm - m_fKv*kvTerm;

	return result;
}