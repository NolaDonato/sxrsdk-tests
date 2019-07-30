package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMaterial;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTexture;
import com.samsungxr.SXRTransform;
import com.samsungxr.nodes.SXRCubeNode;
import com.samsungxr.nodes.SXRSphereNode;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;

import net.jodah.concurrentunit.Waiter;

import org.joml.Vector3f;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

/**
 * Physics Joint Tests for MultiBody support
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
@RunWith(AndroidJUnit4.class)
public class PhysicsJointTest
{
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    SXRWorld mWorld;

    @Rule
    public ActivityTestRule<SXRTestableActivity> ActivityRule = new
            ActivityTestRule<SXRTestableActivity>(SXRTestableActivity.class);

    @Before
    public void setUp() throws TimeoutException {
        mWaiter = new Waiter();
        sxrTestUtils = new SXRTestUtils(ActivityRule.getActivity());
        sxrTestUtils.waitForOnInit();
        mWorld = new SXRWorld(sxrTestUtils.getMainScene(), true);
    }

    @Test
    public void createJoint()
    {
        SXRPhysicsJoint sphereJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 2.5f, 0);
        SXRNode sphere = addSphere(sxrTestUtils.getMainScene(),1.5f, 40.0f, -10.0f);
        sxrTestUtils.getMainScene().addNode(sphere);
        sphere.attachComponent(sphereJoint);
        mWaiter.assertTrue(sphereJoint.getMass() == 2.5f);
    }

    @Test
    public void enableJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        mWorld.getEventReceiver().addListener(listener);

        SXRPhysicsJoint sphereJoint1 = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 2.5f, 1);
        SXRNode sphere1 = addSphere(sxrTestUtils.getMainScene(),1.0f, 10.0f, -10.0f);
        SXRPhysicsJoint sphereJoint2 = new SXRPhysicsJoint(sphereJoint1, 1, 2.5f);
        SXRNode sphere2 = addSphere(sxrTestUtils.getMainScene(),2.0f, 10.0f, -10.0f);
        final float jointPos[] = { -1, 0f, 0f };
        final float rotation[] = { 1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f };

        sxrTestUtils.getMainScene().addNode(sphere1);
        sphere1.addChildObject(sphere2);
        sphere1.attachComponent(sphereJoint1);
        sphere2.attachComponent(sphereJoint2);
        listener.waitUntilAdded();
        mWorld.setEnable(true);
        listener.waitForXSteps(10);

        float lastY = sphereJoint1.getTransform().getPositionY();
        float lastY2 =  sphereJoint2.getTransform().getPositionY();
        listener.waitForXSteps(10);
        float newY = sphereJoint1.getTransform().getPositionY();
        float newY2 = sphereJoint2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY > newY);//balls are falling
        mWaiter.assertTrue( lastY2 > newY2);

        lastY = sphereJoint1.getTransform().getPositionY();
        lastY2 =  sphereJoint2.getTransform().getPositionY();
        sphereJoint1.setEnable(false);
        listener.waitForXSteps(60);
        newY = sphereJoint1.getTransform().getPositionY();
        newY2 = sphereJoint2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY == newY); //ball1 stoped falling
        mWaiter.assertTrue( lastY2 > newY2); //ball2 is falling

        lastY = sphereJoint1.getTransform().getPositionY();
        lastY2 =  sphereJoint2.getTransform().getPositionY();
        sphereJoint1.setEnable(true);
        listener.waitForXSteps(10);
        newY = sphereJoint1.getTransform().getPositionY();
        newY2 = sphereJoint2.getTransform().getPositionY();
        mWaiter.assertTrue( lastY > newY); //ball1 is falling again
        mWaiter.assertTrue( lastY2 > newY2); //ball2 kept falling
    }

    @Test
    public void testTwoJoints()
    {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(context,  10.0f, 1);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, 1, 2.0f);
        SXRNode box = addCube(scene, 0, -3, 0);
        SXRNode sphere = addSphere(scene, 0, 3, 0);

        sxrTestUtils.getMainScene().addNode(box);
        box.addChildObject(sphere);
        box.attachComponent(rootJoint);
        sphere.attachComponent(firstJoint);
        mWorld.setEnable(true);
    }

    @Test
    public void testFixedConstraint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 10.0f, 1);
        SXRPhysicsJoint firstLink = new SXRPhysicsJoint(rootJoint, 1, 10.0f);

        SXRNode box1 = addCube(scene, 0f, 0.5f, -30f);
        scene.addNode(box1);

        SXRNode box2 = addCube(scene, 0f, 0, 15f);
        box1.addChildObject(box2);
        box1.attachComponent(rootJoint);

        SXRFixedConstraint constraint = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), firstLink);
        box2.attachComponent(firstLink);
        box2.attachComponent(constraint);
        sxrTestUtils.waitForXFrames(10);

        listener.waitUntilAdded();
        mWorld.setEnable(true);
        float distance = transformDistance(box1.getTransform(), box2.getTransform());

        rootJoint.applyTorque(0, 0, 200);
        listener.waitForXSteps(120);
        float rotation = Math.abs(box1.getTransform().getRotationX() - box2.getTransform().getRotationX())
            + Math.abs(box1.getTransform().getRotationY() - box2.getTransform().getRotationY())
            + Math.abs(box1.getTransform().getRotationZ() - box2.getTransform().getRotationZ());
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformDistance(box1.getTransform(), box2.getTransform())) < 0.2);

        firstLink.applyCentralForce(300, 0, 300);
        listener.waitForXSteps(180);
        rotation = Math.abs(box1.getTransform().getRotationX() - box2.getTransform().getRotationX())
            + Math.abs(box1.getTransform().getRotationY() - box2.getTransform().getRotationY())
            + Math.abs(box1.getTransform().getRotationZ() - box2.getTransform().getRotationZ());
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformDistance(box1.getTransform(), box2.getTransform())) < 0.2);
        sxrTestUtils.waitForXFrames(30);
    }


    private SXRNode addCube(SXRScene scene, float x, float y, float z)
    {
        SXRContext ctx = sxrTestUtils.getSxrContext();
        SXRMaterial blue = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        SXRCubeNode cubeObject = new SXRCubeNode(ctx,
                                                 true, blue,
                                                 new Vector3f(x, y, z));
        SXRBoxCollider boxCollider = new SXRBoxCollider(ctx);

        blue.setDiffuseColor(0, 0, 1, 1);
        cubeObject.getTransform().setPosition(x, y, z);
        cubeObject.setName("cube");
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        cubeObject.attachCollider(boxCollider);
        return cubeObject;
    }

    private SXRNode addSphere(SXRScene scene, float x, float y, float z)
    {
        SXRContext ctx = sxrTestUtils.getSxrContext();
        SXRMaterial red = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        SXRNode sphereObject = new SXRSphereNode(ctx, true, red);

        red.setDiffuseColor(1, 0, 0, 1);
        sphereObject.getTransform().setScaleX(0.5f);
        sphereObject.getTransform().setScaleY(0.5f);
        sphereObject.getTransform().setScaleZ(0.5f);
        sphereObject.getTransform().setPosition(x, y, z);
        sphereObject.setName("ball");
        SXRSphereCollider sphereCollider = new SXRSphereCollider(ctx);
        sphereCollider.setRadius(0.5f);
        sphereObject.attachCollider(sphereCollider);
        return sphereObject;
    }

    float transformDistance(SXRTransform a, SXRTransform b) {
        float x = a.getPositionX() - b.getPositionX();
        float y = a.getPositionY() - b.getPositionY();
        float z = a.getPositionZ() - b.getPositionZ();

        return (float)Math.sqrt(x * x + y * y + z * z);
    }
}
