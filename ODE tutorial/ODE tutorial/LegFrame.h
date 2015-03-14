#pragma once

#include "IKChain.h"

enum LEG_MODE { stance = 0, swing };

class LegFrame
{
public:
		LegFrame(void);
		LegFrame(IKChain* left, IKChain* right)
		{
			m_left_leg = left;
			m_right_leg = right;
		};
		Link* getLeftRoot()
		{
			return m_left_leg->getRoot();
		}

		Link* getRightRoot()
		{
			return m_right_leg->getRoot();
		}
		~LegFrame(void);

private:

		IKChain* m_left_leg;
		IKChain* m_right_leg;

		LEG_MODE m_left_leg_mode;
		LEG_MODE m_right_leg_mode;

};

