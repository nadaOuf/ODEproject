#pragma once

#include "Link.h"
#include <exception>

#define MAX_ITER 50
#define EPSILON 0.0001

/*
	LOOK -	IKChain contains "Joint" whose angles will 
			be calculated by CCD IK.
*/

class IKChain
{
	public:
		IKChain(Link* begin, Link* end, int depth);
		~IKChain();

		Link* getRoot()
		{
			return root_;
		};

		glm::vec3 getEndJointPosition_Current()
		{
			return end_->GetJointGlobalPosition_Current();
		};

		glm::vec3 GetPosition_inRootFrame(glm::vec3 position)
		{
			return glm::vec3();
		}

		glm::vec3 GetTransformMat_inRootFrame(glm::vec3 transformMat);

		void SolveIK(glm::vec3 goal) const;

	private:
		Link* root_;
		Link* end_;
		int link_;
};




