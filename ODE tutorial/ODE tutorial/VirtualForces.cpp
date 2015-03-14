#include "VirtualForces.h"


VirtualForces::VirtualForces(void)
{
}


VirtualForces::~VirtualForces(void)
{
}

void VirtualForces::GravityCompensation(Link* start, Link* end)
{
	glm::vec3 gravity = glm::vec3(0, start->mass * 9.8f ,0);

	Link* currentLink = start;
	glm::vec3 pGlobal = start->GetGlobalPosition();
	glm::vec3 tmp;

	if(currentLink != end)
	{
		if(currentLink == NULL)
			throw new runtime_error("GravityCompensation ERROR : chaining is not correct.");

		glm::vec3 tmpV = pGlobal - currentLink->GetParent()->GetGlobalPosition();
		glm::vec3 torque = glm::cross(tmpV, gravity);


		currentLink = currentLink->GetParent();
	}

	if(end != NULL)
	{
		glm::vec3 tmpV = pGlobal - currentLink->GetParent()->GetGlobalPosition();
		glm::vec3 tmpT = glm::cross(tmpV, gravity);
	}
}