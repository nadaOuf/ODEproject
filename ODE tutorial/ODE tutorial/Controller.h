#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "headers.h"

class Controller
{
public:
	enum type{P, PD};
	Controller();
	Controller(float kp, float kv=0.0);
	~Controller();

	glm::vec3 calculateControllerOutput(glm::vec3 kpTerm, glm::vec3 kvTerm = glm::vec3(0,0,0));
	

private:
	float m_fKp;
	float m_fKv;

};


#endif
