#pragma once

#include "IKChain.h"

enum LEG_MODE { stance = 0, swing };

class LegFrame
{
	public:
		LegFrame(void);
		~LegFrame(void);

		IKChain* left_leg;
		IKChain* right_leg;

		LEG_MODE left_leg_mode;
		LEG_MODE right_leg_mode;

};

