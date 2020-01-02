package com.samsungxr.physics;

import androidx.test.rule.ActivityTestRule;
import androidx.test.runner.AndroidJUnit4;

import net.jodah.concurrentunit.Waiter;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.physics.SXRRigidBody;
import com.samsungxr.physics.SXRWorld;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;
import com.samsungxr.utility.Log;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

/**
 * Instrumentation test, which will execute on an Android device.
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */

@RunWith(AndroidJUnit4.class)
public class RigidBodyAttributesTest {
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    private SXRWorld world;

    @Rule
    public ActivityTestRule<SXRTestableActivity> ActivityRule = new
            ActivityTestRule<SXRTestableActivity>(SXRTestableActivity.class);

    @Before
    public void setUp() throws TimeoutException {
        sxrTestUtils = new SXRTestUtils(ActivityRule.getActivity());
        mWaiter = new Waiter();
        sxrTestUtils.waitForOnInit();
        world = new SXRWorld(sxrTestUtils.getMainScene());
    }

    @Test
    public void createRigidBody()
    {
        SXRRigidBody mSphereRigidBody = new SXRRigidBody(sxrTestUtils.getSxrContext(), 2.5f);
        addSphere(sxrTestUtils.getMainScene(), mSphereRigidBody, 1.5f, 40.0f, -10.0f);
        mWaiter.assertTrue(mSphereRigidBody.getMass() == 2.5f);
        mWaiter.assertTrue(mSphereRigidBody.getRestitution() == 1.5f);
        mWaiter.assertTrue(mSphereRigidBody.getFriction() == 0.5f);
    }

    @Test
    public void enableRigidBody()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        world.getEventReceiver().addListener(listener);

        SXRRigidBody mSphereRigidBody = new SXRRigidBody(sxrTestUtils.getSxrContext(), 2.5f);
        addSphere(sxrTestUtils.getMainScene(), mSphereRigidBody,  1.0f, 10.0f, -10.0f);

        SXRRigidBody mSphereRigidBody2 = new SXRRigidBody(sxrTestUtils.getSxrContext(), 2.5f);
        addSphere(sxrTestUtils.getMainScene(), mSphereRigidBody2, 2.0f, 10.0f, -10.0f);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(10);

        float lastY = mSphereRigidBody.getTransform().getPositionY();
        float lastY2 =  mSphereRigidBody2.getTransform().getPositionY();
        listener.waitForXSteps(10);
        float newY = mSphereRigidBody.getTransform().getPositionY();
        float newY2 = mSphereRigidBody2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY > newY);//balls are falling
        mWaiter.assertTrue( lastY2 > newY2);

        lastY = mSphereRigidBody.getTransform().getPositionY();
        lastY2 =  mSphereRigidBody2.getTransform().getPositionY();
        mSphereRigidBody.setEnable(false);
        listener.waitForXSteps(60);
        newY = mSphereRigidBody.getTransform().getPositionY();
        newY2 = mSphereRigidBody2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY == newY); //ball1 stoped falling
        mWaiter.assertTrue( lastY2 > newY2); //ball2 is falling

        lastY = mSphereRigidBody.getTransform().getPositionY();
        lastY2 =  mSphereRigidBody2.getTransform().getPositionY();
        mSphereRigidBody.setEnable(true);
        listener.waitForXSteps(10);
        newY = mSphereRigidBody.getTransform().getPositionY();
        newY2 = mSphereRigidBody2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY > newY); //ball1 is falling again
        mWaiter.assertTrue( lastY2 > newY2); //ball2 kept falling
    }


    private SXRNode meshWithTexture(String mesh, String texture)
    {
        SXRNode object = null;
        SXRContext ctx = sxrTestUtils.getSxrContext();
        try
        {
            object = new SXRNode(ctx, new SXRAndroidResource(ctx, mesh), new SXRAndroidResource(ctx, texture));
        } catch (IOException e)
        {
            mWaiter.fail(e);
        }
        return object;
    }

    private void addSphere(SXRScene scene, SXRPhysicsWorldObject sphereBody, float x, float y, float z) {

        SXRNode sphereObject = meshWithTexture("sphere.obj", "sphere.jpg");
        sphereObject.getTransform().setPosition(x, y, z);

        // Collider
        SXRSphereCollider sphereCollider = new SXRSphereCollider(sxrTestUtils.getSxrContext());
        sphereCollider.setRadius(1.0f);
        sphereObject.attachCollider(sphereCollider);

        // Physics body
        if ((sphereBody instanceof SXRRigidBody))
        {
            ((SXRRigidBody) sphereBody).setRestitution(1.5f);
            ((SXRRigidBody) sphereBody).setFriction(0.5f);
        }
        else
        {
            ((SXRPhysicsJoint) sphereBody).setFriction(0.5f);
        }
        scene.addNode(sphereObject);
        sphereObject.attachComponent(sphereBody);
    }
}
