#include "VirtualForces.h"


VirtualForces::VirtualForces(void)
{
}


VirtualForces::~VirtualForces(void)
{
}

void VirtualForces::GravityCompensation(Link* start, Link* end)
{
	glm::vec3 gravity = glm::vec3(0, 0, start->mass * 9.8f);

	Link* currentLink = start;
	glm::vec3 pGlobal = start->GetCMGlobalPosition();
	glm::vec3 tmp;

	while(currentLink != NULL)
	{
		if(currentLink == NULL)
			throw new runtime_error("GravityCompensation ERROR : chaining is not correct.");

		glm::vec3 tmpV = pGlobal - currentLink->GetJointGlobalPosition();
		glm::vec3 torque = glm::cross(10.0f * tmpV, gravity);

		currentLink->AddTorque(-torque);
		currentLink = currentLink->GetParent();
	}

	/*if(end != NULL)
	{
		glm::vec3 tmpV = pGlobal - currentLink->GetParent()->GetGlobalPosition();
		glm::vec3 torque = glm::cross(tmpV, gravity);
		currentLink->AddTorque(torque);
	}*/
}