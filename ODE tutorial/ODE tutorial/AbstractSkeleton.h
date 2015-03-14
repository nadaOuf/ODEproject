#ifndef _ABSTRACTSKELETON_H
#define _ABSTRACTSKELETON_H

#include "headers.h"
#include "Controller.h"

typedef struct {
	MyLink* leg[2][3]; // 0 -- left, 1 -- right
	bool legMode[2]; // true -- stance, false swing
} MLegFrame;

class AbstractSkeleton
{
public:
	AbstractSkeleton();
	~AbstractSkeleton();

	void setLegFrames(LegFrame* shoulder, LegFrame* hip);
	LegFrame* getLegFrame(int index);

	enum LEGFRAME{SHOULDER, HIP};

private:
	LegFrame* m_legFrames[2]; // 0 -- shoulder frame, 1 -- hip frame

};

#endif