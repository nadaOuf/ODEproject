#include "Link.h"

Link::Link(Link* parent)
{
	parent_ = parent;
	children_ = std::vector<Link*>();

	if(parent_ != NULL) parent_->AddChild(this);
}

Link::~Link(void)
{
}

int Link::AddChild(Link* child)
{
	children_.push_back(child);
	return children_.size() - 1;
}

Link* Link::GetChild(int index)
{
	if(children_.size() > index && index > 0)
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

	const dReal *current = dBodyGetQuaternion(this->bodyID);
	glm::quat orientation(current[0], current[1], current[2], current[3]);

	orientation = rot * orientation;

	glm::vec3 target_angle =  glm::eulerAngles(orientation);
	tX = target_angle.x;
	tY = target_angle.y;
	tZ = target_angle.z;

}
