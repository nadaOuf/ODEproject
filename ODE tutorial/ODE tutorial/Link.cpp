#include "Link.h"

Link::Link()
{
	parent_ = NULL;
}

Link::Link(Link* parent)
{
	parent_ = parent;
	children_ = std::vector<Link*>();

	target_orientation_ = glm::quat();
	target_orientation_euler_ = glm::vec3();

	if(parent_ != NULL) parent_->AddChild(this);
}

Link::~Link(void)
{
}

int Link::AddChild(Link* child)
{
	children_.push_back(child);
	return children_.size();
}

Link* Link::GetChild(int index)
{
	if(children_.size() > index && index >= 0)
		return children_[index];

	return NULL;
}
	
Link* Link::GetChild()
{
	if(children_.size() > 0)
		return children_[0];

	return NULL;
}

//just add the rotation to the current quaternion
void Link::RotateBy(glm::vec3 axis, float angleRad)
{
	glm::quat rot = glm::angleAxis(angleRad,axis);
	target_orientation_ = rot * target_orientation_;

	target_orientation_euler_ = glm::eulerAngles(target_orientation_);



	UpdateTargetTransformMat(true);
}

void Link::ResetTargetRotation()
{
	target_transform_mat_ = GetTransformMatrix_Current();
	target_orientation_euler_ = GetCurrentAngles();
	target_orientation_ = glm::quat(target_orientation_euler_);
}

void Link::UpdateTargetTransformMat(bool updateChild)
{
	glm::mat4 parentMat = parent_ == NULL? glm::mat4(1.0) : parent_->target_transform_mat_;

	if(parent_->parent_ == NULL)
	{
		target_transform_mat_ = glm::translate(glm::mat4(1.0), GetJointGlobalPosition_Current()) * glm::toMat4(target_orientation_);
		return;
	}

	glm::mat4 rot = glm::toMat4(target_orientation_);
	glm::mat4 displacement = glm::translate(glm::mat4(1.0), glm::vec3(0,0,-(parent_->totalLength + parent_->radius)));	
	target_transform_mat_ = parentMat * displacement * rot;
	
	if(updateChild)
	{
		for(int i = 0; i < children_.size(); i++)
			children_[i]->UpdateTargetTransformMat(true);
	}
}

glm::vec3 Link::GetCurrentAngles()
{
	glm::vec3 angles = glm::vec3();
	switch(jointType)
	{
		case 0:
			angles.x = dJointGetAMotorAngle(aMotor, 0);
			angles.y = dJointGetAMotorAngle(aMotor, 1);
			angles.z = dJointGetAMotorAngle(aMotor, 2);
			break;
		case 1:
			angles.x = 0;
			angles.y = dJointGetHingeAngle(joint);
			angles.z = 0;
			break;
		case 2:
			angles.x = 0;
			angles.y = dJointGetUniversalAngle1(joint);
			angles.z = dJointGetUniversalAngle1(joint);;
			break;
		default:
			break;
	}
	return angles;
}

glm::mat4 Link::GetTransformMatrix_Current()
{
	glm::vec3 angles = GetCurrentAngles();
	dMatrix3 rotation;
	dRFromEulerAngles(rotation, angles.x, angles.y, angles.z);
	glm::mat4 result = glm::mat4();

	result[0][0] = rotation[0];
	result[0][1] = rotation[1];
	result[0][2] = rotation[2];
	result[0][3] = rotation[3];

	result[1][0] = rotation[4];
	result[1][1] = rotation[5];
	result[1][2] = rotation[6];
	result[1][3] = rotation[7];

	result[2][0] = rotation[8];
	result[2][1] = rotation[9];
	result[2][2] = rotation[10];
	result[2][3] = rotation[11];

	
	glm::vec3 pos = GetJointGlobalPosition_Current();
	if(parent_ == NULL)
		pos = glm::vec3();

	result[3][0] = pos.x;
	result[3][1] = pos.y;
	result[3][2] = pos.z;
	result[3][3] = 1;

	return result;
}
glm::vec3 Link::GetJointGlobalPosition_Current()
{
	dVector3 pos;
	switch(jointType)
	{
		case 0:
			dJointGetBallAnchor(joint, pos);
			break;
		case 1:
			dJointGetHingeAnchor(joint, pos);
			break;
		case 2:
			dJointGetUniversalAnchor(joint, pos);
			break;
		default:
			break;
	}
	glm::vec3 result = glm::vec3();

	for(int i = 0; i < 3; ++i)
		result[i] = pos[i];	

	return result;
}

glm::vec3 Link::GetCMGlobalPosition()
{
	const dReal* pos = dBodyGetPosition(body);
	glm::vec3 result = glm::vec3();

	for(int i = 0; i < 3; ++i)
		result[i] = pos[i];	

	return result;
}

void Link::AddTorque(glm::vec3 torque)
{
	torque_ += torque;
}