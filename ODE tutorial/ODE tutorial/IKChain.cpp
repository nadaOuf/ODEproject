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
	if(glm::distance2(goal, end_->GetJointGlobalPosition_Current()) < EPSILON)
		return;

	Link* rotating_joint = end_;

	//set up!
	std::vector<glm::vec3> jointPos;
	while(true)
	{
		rotating_joint->ResetTargetRotation();
		if(rotating_joint->GetParent() == NULL)
			break;
		rotating_joint = rotating_joint->GetParent();
	}
	
	//every calculation must be performed in the root's frame.
	for(int i = 0; i < MAX_ITER; i++)
	{
		rotating_joint = end_->GetParent();

		for(int j = 0; j < link_; j++)
		{
			glm::vec3 tmp = end_->GetJointGlobalPosition_Target();

			glm::vec3 err = goal - end_->GetJointGlobalPosition_Target();
			glm::vec3 r = end_->GetJointGlobalPosition_Target() -
				rotating_joint->GetJointGlobalPosition_Target();
			
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
				glm::mat4 transformation = rotating_joint->GetTransformMatrix_Target();
				//float det = glm::determinant(transformation);
				//glm::mat4 rot_inverse = glm::inverse(transformation);
				//rotAxis = glm::vec3( rot_inverse * glm::vec4(rotAxis, 0));

				//if(rotAxis.y < 0) rotAxis = glm::vec3(0,-1,0);
				//else rotAxis = glm::vec3(0,1,0);
				rotating_joint->RotateBy(rotAxis, angle);
				
				//if less than threshold -> CCD done
				if(glm::distance2(goal, end_->GetJointGlobalPosition_Target()) < EPSILON)
					return; 
			}

			//move up the chain
			rotating_joint = rotating_joint->GetParent();
		}
	}
}