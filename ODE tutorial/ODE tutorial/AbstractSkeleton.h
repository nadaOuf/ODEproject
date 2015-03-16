#ifndef _ABSTRACTSKELETON_H
#define _ABSTRACTSKELETON_H

#include "headers.h"
#include "Controller.h"
#include "LegFrame.h"

class AbstractSkeleton
{
	public:
		AbstractSkeleton();
		~AbstractSkeleton();

		void setLegFrames(LegFrame* shoulder, LegFrame* hip);
		LegFrame* getLegFrame(int index);

		void setSkeletonRoot(Link* root);
		Link* getSkeletonRoot();

		enum LEGFRAME{SHOULDER, HIP};

private:
	LegFrame* m_legFrames[2]; // 0 -- shoulder frame, 1 -- hip frame
	Link* m_root;

};

#endif