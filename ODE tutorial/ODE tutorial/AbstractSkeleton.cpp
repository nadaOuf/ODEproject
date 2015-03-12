#include "AbstractSkeleton.h"

AbstractSkeleton::AbstractSkeleton()
{

}

AbstractSkeleton::~AbstractSkeleton()
{

}

void AbstractSkeleton::setLegFrames(LegFrame* shoulderFrame, LegFrame* hipFrame)
{
	m_legFrames[0] = shoulderFrame;
	m_legFrames[1] = hipFrame;
}

LegFrame* AbstractSkeleton::getLegFrame(int index)
{
	return m_legFrames[index];
}
