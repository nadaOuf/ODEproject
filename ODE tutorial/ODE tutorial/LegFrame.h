#pragma once

#include "IKChain.h"

class LegFrame
{
	public:
		LegFrame(void);
		~LegFrame(void);

		IKChain* left_leg;
		IKChain* right_leg;

		bool left_leg_mode;
		bool right_leg_mode;

};

