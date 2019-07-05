package com.samsungxr.physics;

import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.utility.Log;

public class PhysicsEventHandler implements SXRWorld.IPhysicsEvents
{
    private final SXRTestUtils mTester;
    private int mNumObjects = 0;
    private int mNumSteps = 0;

    PhysicsEventHandler(SXRTestUtils tester, int n)
    {
        mNumObjects = n;
        mTester = tester;
    }

    public void waitUntilAdded()
    {
        mTester.waitForAssetLoad();
    }

    public void waitForXSteps(int x)
    {
        Log.d("PHYSICS", "Wait for %d steps", x);
        mNumSteps = x;
        mTester.waitForAssetLoad();
    }

    @Override
    public void onAddRigidBody(SXRWorld world, SXRRigidBody body)
    {
        if (mNumObjects > 0)
        {
            if (--mNumObjects == 0)
            {
                mTester.onAssetLoaded(null);
            }
            if (mNumObjects < 0)
            {
                Log.e("PHYSICS", "RigidBody added after limit");
            }
        }
    }

    @Override
    public void onRemoveRigidBody(SXRWorld world, SXRRigidBody body)
    {

    }

    @Override
    public void onAddConstraint(SXRWorld world, SXRConstraint constraint)
    {
        if (mNumObjects > 0)
        {
            if (--mNumObjects == 0)
            {
                mTester.onAssetLoaded(null);
            }
            if (mNumObjects < 0)
            {
                Log.e("PHYSICS", "Constraint added after limit");
            }
        }
    }

    @Override
    public void onRemoveConstraint(SXRWorld world, SXRConstraint constraint)
    {

    }

    @Override
    public void onStepPhysics(SXRWorld world)
    {
        if (mNumSteps > 0)
        {
            if (--mNumSteps == 0)
            {
                Log.d("PHYSICS", "Steps finished");
                mTester.onAssetLoaded(null);
            }
        }
    }
};

