#pragma once

#include "headers.h"
#include "Controller.h"
#include <glm/gtx/quaternion.hpp>

class Link
{
	public:
		Link(Link* parent);
		~Link(void);
		
		int AddChild(Link* child); // return an index
		Link* GetParent() { return parent_; }
		Link* GetChild(int index); 
		Link* GetChild(); //get the first child.
		int NumChildren() { return children_.size(); }

		void RotateBy(glm::vec3 axis, float angleRad);

		// TODO


		dBodyID bodyID;
		dGeomID geomID;
		dJointID jointID;
		dJointID motorID;
		dReal mass, radius, length, totalLength;
		dReal cX, cY, cZ; //C.M. for the link
		dReal tX, tY, tZ; //Target angles for the joint
		Controller* PD;

	private:
		Link* parent_;
		std::vector<Link*> children_;
};

