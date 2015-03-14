#pragma once

#include "Link.h"
#include <exception>

#define MAX_ITER 50
#define EPSILON 0.01

/*
	LOOK -	IKChain contains "Joint" whose angles will 
			be calculated by CCD IK.
*/

class IKChain
{
	public:
		IKChain(Link* begin, Link* end, int depth);
		~IKChain();

		void SolveIK(glm::vec3 goal) const;

	private:
		Link* root_;
		Link* end_;
		int link_;
};




