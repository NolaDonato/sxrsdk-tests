package com.samsungxr.physics;

import com.samsungxr.unittestutils.SXRTestUtils;

public class PhysicsEventHandler implements SXRWorld.IPhysicsEvents
{
    private final SXRTestUtils mTester;
    private int mNumObjects = 0;

    PhysicsEventHandler(SXRTestUtils tester, int numObjects)
    {
        mTester = tester;
        mNumObjects = numObjects;
    }

    public void setNumObjects(int n)
    {
        mNumObjects = n;
    }

    @Override
    public void onAddRigidBody(SXRWorld world, SXRRigidBody body)
    {
        if (--mNumObjects == 0)
        {
            mTester.onAssetLoaded(null);
        }
    }

    @Override
    public void onRemoveRigidBody(SXRWorld world, SXRRigidBody body)
    {

    }

    @Override
    public void onAddConstraint(SXRWorld world, SXRConstraint constraint)
    {
        if (--mNumObjects == 0)
        {
            mTester.onAssetLoaded(null);
        }
    }

    @Override
    public void onRemoveConstraint(SXRWorld world, SXRConstraint constraint)
    {

    }

    @Override
    public void onStepPhysics(SXRWorld world)
    {

    }
};

