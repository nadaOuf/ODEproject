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
		};

		Link* getRightRoot()
		{
			return m_right_leg->getRoot();
		};

		void setLeftLegID(int id)
		{
			m_left_leg_ID = id;
		};

		int getLeftLegID()
		{
			return m_left_leg_ID;
		};

		void setRightLegID(int id)
		{
			m_right_leg_ID = id;
		};

		int getRightLegID()
		{
			return m_right_leg_ID;
		};

		void solveIKForLeftLeg(glm::vec3 position)
		{
			m_left_leg->SolveIK(position);
		};

		void solveIKForRightLeg(glm::vec3 position)
		{
			m_right_leg->SolveIK(position);
		};

		glm::vec3 getLeftLegEndJointPosition()
		{
			return m_left_leg->getEndJointPosition_Current();
		};

		glm::vec3 getRightLegEndJointPosition()
		{
			return m_right_leg->getEndJointPosition_Current();
		};

		~LegFrame(void);

private:

		IKChain* m_left_leg;
		IKChain* m_right_leg;

		int m_left_leg_ID;
		int m_right_leg_ID;

		LEG_MODE m_left_leg_mode;
		LEG_MODE m_right_leg_mode;

};

