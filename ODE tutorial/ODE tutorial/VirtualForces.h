#pragma once

#include "Link.h"
 
class VirtualForces
{
public:
	VirtualForces(void);
	~VirtualForces(void);

	void GravityCompensation(Link* start, Link* end);

	std::vector<glm::vec3> torques;
};

