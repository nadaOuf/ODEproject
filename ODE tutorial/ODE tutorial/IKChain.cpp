#include "IKChain.h"

//link = numJoints - 1
IKChain::IKChain(Link* root, Link* end, int link)
{
	//check if the chain is valid. (root->end)
	if(root == NULL || end == NULL)
		throw new std::runtime_error("IKChain constructor error - Link can't be null.");

	Link* current = end;
	for(int i = 0; i < link; i++)
		current = current->GetParent();
	
	if(current != root)
		throw new std::runtime_error("IKChain constructor error - chaining error.");

	root_ = root;
	end_ = end;
	link_ = link;
}

IKChain::~IKChain()
{
}

//2D CCD rotating around Y-axis
void IKChain::SolveIK(glm::vec3 goal) const
{
	if(glm::distance2(goal, end_->GetGlobalPosition()) < EPSILON)
		return;
	
	for(int i = 0; i < MAX_ITER; i++)
	{
		Link* rotating_joint = end_->GetParent();

		for(int j = 0; j < link_; j++)
		{
			glm::vec3 err = goal - end_->GetGlobalPosition();
			glm::vec3 r = end_->GetGlobalPosition() -
				rotating_joint->GetGlobalPosition();
			
			glm::vec3 r_cross_err = glm::cross(r, err);
			float r_cross_err_length = glm::length(r_cross_err);

			float r_dot_err = glm::dot(r, err);
			float r_dot_r = glm::dot(r, r);

			float denom = r_dot_r + r_dot_err;

			if(denom != 0 && r_cross_err_length != 0)
			{
				//calculate axis and delta angle
				glm::vec3 rotAxis = r_cross_err / r_cross_err_length;

				float angle = r_cross_err_length;
				angle /= denom;

				/*	if the result is producing an error
					(the angle is too large that tan angle != angle) 
					uncomment the line below.*/
				angle = atan(angle); 

				rotAxis = glm::vec3(glm::inverse(rotating_joint->GetTransformMatrix()) * glm::vec4(rotAxis, 0));
				rotating_joint->RotateBy(rotAxis, angle);

				//if less than threshold -> CCD done
				if(glm::distance2(goal, end_->GetGlobalPosition()) < EPSILON)
					return; 
			}

			//move up the chain
			rotating_joint = rotating_joint->GetParent();
		}
	}
}