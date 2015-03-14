#include "Link.h"

Link::Link()
{
	parent_ = NULL;
}

Link::Link(Link* parent)
{
	parent_ = parent;
	children_ = std::vector<Link*>();

	tY = 0;
	tX = 0;
	tZ = 0;

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
}
	
Link* Link::GetChild()
{
	if(children_.size() > 0)
		return children_[0];
}


//just add the rotation to the current quaternion
void Link::RotateBy(glm::vec3 axis, float angleRad)
{
	glm::quat rot = glm::angleAxis(angleRad,axis);

	const dReal *current = dBodyGetQuaternion(this->body);
	glm::quat orientation(current[0], current[1], current[2], current[3]);

	orientation = rot * orientation;

	glm::vec3 target_angle =  glm::eulerAngles(orientation);
	tX = target_angle.x;
	tY = target_angle.y;
	tZ = target_angle.z;

}

glm::mat4 Link::GetTransformMatrix()
{
	const dReal* rotation = dBodyGetRotation(body);
	glm::mat4 result = glm::mat4();

	result[0][0] = rotation[0];
	result[0][1] = rotation[1];
	result[0][2] = rotation[2];
	result[0][3] = 0;

	result[1][0] = rotation[3];
	result[1][1] = rotation[4];
	result[1][2] = rotation[5];
	result[1][3] = 0;

	result[2][0] = rotation[6];
	result[2][1] = rotation[7];
	result[2][2] = rotation[8];
	result[2][3] = 0;

	result[3][0] = rotation[9];
	result[3][1] = rotation[10];
	result[3][2] = rotation[11];
	result[3][3] = 1;

	return result;
}
glm::vec3 Link::GetJointGlobalPosition()
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