#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget

	vec3 pos = m_Guide.getGlobalTranslation() + m_pSkeleton->getRootNode()->getGlobalTranslation();
	pos[1] = 0;
	m_Guide.setGlobalTranslation(pos);

	vec3 forward = (guideTargetPos - pos).Normalize();
	m_Guide.setGlobalRotation(mat3(forward.Cross(axisY), axisY, forward));

}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	AJoint *root = m_pSkeleton->getRootNode();
	vec3 pos = root->getLocalTranslation();
	pos[1] += leftHeight > rightHeight ? leftHeight : rightHeight;
	root->setLocalTranslation(pos);

	// 2.	Update the character with Limb-based IK 
	ATarget leftTarget = ATarget();
	vec3 leftPos = leftFoot->getGlobalTranslation();
	leftPos[1] += leftHeight;
	leftTarget.setGlobalTranslation(leftPos);
	m_IKController->IKSolver_Limb(m_IKController->mLfootID, leftTarget);

	ATarget rightTarget = ATarget();
	vec3 rightPos = rightFoot->getGlobalTranslation();
	rightPos[1] += rightHeight;
	rightTarget.setGlobalTranslation(rightPos);
	m_IKController->IKSolver_Limb(m_IKController->mRfootID, rightTarget);


	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		leftFoot->setLocalRotation(mat3(axisX, leftNormal, leftNormal.Cross(axisX)));
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		rightFoot->setLocalRotation(mat3(axisX, rightNormal, rightNormal.Cross(axisX)));
	}
	m_pSkeleton->update();
}
