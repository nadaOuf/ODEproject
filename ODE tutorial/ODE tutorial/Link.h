#pragma once

#include "headers.h"
#include "Controller.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

class Link
{
	public:
		Link();
		Link(Link* parent);
		~Link(void);
		
		int AddChild(Link* child); // return an index
		Link* GetParent() { return parent_; }
		Link* GetChild(int index); 
		Link* GetChild(); //get the first child.
		int NumChildren() { return children_.size(); }

		void RotateBy(glm::vec3 axis, float angleRad);

		// IK
		glm::vec3 GetCMGlobalPosition();
		glm::mat4 GetTransformMatrix_Current();
		glm::vec3 GetJointGlobalPosition_Current();
		glm::vec3 GetCurrentAngles();
		void ResetTargetRotation();
		void UpdateTargetTransformMat(bool updateChild);
		
		glm::mat4 GetTransformMatrix_Target() { return target_transform_mat_; }
		glm::vec3 GetJointGlobalPosition_Target() { 
			return glm::vec3(target_transform_mat_ * glm::vec4(0,0,0,1));}

		void AddTorque(glm::vec3 torque);
		glm::vec3 getTorque()
		{
			return torque_;
		};

		void clearTorque()
		{
			torque_ = glm::vec3();
		};

		dBodyID body;
		dGeomID geom;
		dJointID joint;
		dJointID aMotor;
		enum JOINT_TYPE  {BALL, HINGE, UNIVERSAL};
		dReal mass, radius, length, totalLength;
		dReal cX, cY, cZ; //C.M. for the link
		Controller* PD;
		int jointType;

		 //Target angles for the joint
		dReal tX() { return target_orientation_euler_[0]; };
		dReal tY() { return target_orientation_euler_[1]; };
		dReal tZ() { return target_orientation_euler_[2]; };

	private:
		Link* parent_;
		std::vector<Link*> children_;

		glm::vec3 torque_;

		glm::quat target_orientation_;
		glm::vec3 target_orientation_euler_;
		glm::mat4 target_transform_mat_;

};

