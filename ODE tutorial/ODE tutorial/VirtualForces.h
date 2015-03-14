#pragma once

#include "Link.h"
 
class VirtualForces
{
public:
	VirtualForces(void);
	~VirtualForces(void);

	void GravityCompensation(Link* start, Link* end);
};

