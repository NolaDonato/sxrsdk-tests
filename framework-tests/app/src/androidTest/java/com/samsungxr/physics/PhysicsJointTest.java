package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRCameraRig;
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
import com.samsungxr.utility.Log;

import net.jodah.concurrentunit.Waiter;

import org.joml.Matrix4f;
import org.joml.Quaternionf;
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
        SXRCameraRig rig = sxrTestUtils.getMainScene().getMainCameraRig();
        rig.setNearClippingDistance(0.5f);
        rig.setFarClippingDistance(50);
    }

    @Test
    public void createJoint()
    {
        SXRPhysicsJoint sphereJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 2.5f, 1);
        SXRNode sphere = addSphere(1.5f, 40.0f, -10.0f);
        sxrTestUtils.getMainScene().addNode(sphere);
        sphere.attachComponent(sphereJoint);
        mWaiter.assertTrue(sphereJoint.getMass() == 2.5f);
    }

    @Test
    public void testEnableJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        mWorld.getEventReceiver().addListener(listener);

        SXRPhysicsJoint sphereJoint1 = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 2.5f, 2);
        SXRNode sphere1 = addSphere(1, 5, -10);
        SXRPhysicsJoint sphereJoint2 = new SXRPhysicsJoint(sphereJoint1, 1, 2.5f);
        SXRNode sphere2 = addSphere(1, 0, 0);

        sxrTestUtils.getMainScene().addNode(sphere1);
        sphere1.setName("ball1");
        sphere2.setName("ball2");
        sphere1.addChildObject(sphere2);
        sphere1.attachComponent(sphereJoint1);
        sphere2.attachComponent(sphereJoint2);
        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);

        mWorld.setEnable(true);
        listener.waitForXSteps(10);

        Vector3f lastRoot = getWorldPosition(sphereJoint1);
        Vector3f lastLink = getWorldPosition(sphereJoint2);
        listener.waitForXSteps(10);
        Vector3f nextRoot = getWorldPosition(sphereJoint1);
        Vector3f nextLink = getWorldPosition(sphereJoint2);
        mWaiter.assertTrue( lastRoot.y > nextRoot.y);//balls are falling
        mWaiter.assertTrue( lastLink.y > nextLink.y);

        lastRoot = getWorldPosition(sphereJoint1);
        lastLink = getWorldPosition(sphereJoint2);
        sphereJoint2.setEnable(false);
        listener.waitForXSteps(60);
        nextRoot = getWorldPosition(sphereJoint1);
        nextLink = getWorldPosition(sphereJoint2);
        mWaiter.assertTrue( lastRoot.y > nextRoot.y); //ball1 is falling
        mWaiter.assertTrue( Math.abs(lastLink.y - nextLink.y) < 0.0001f); //ball2 stopped falling

        lastRoot = getWorldPosition(sphereJoint1);
        lastLink = getWorldPosition(sphereJoint2);
        sphereJoint2.setEnable(true);
        listener.waitForXSteps(10);
        nextRoot = getWorldPosition(sphereJoint1);
        nextLink = getWorldPosition(sphereJoint2);
        mWaiter.assertTrue( lastRoot.y > nextRoot.y);//balls are falling
        mWaiter.assertTrue( lastLink.y > nextLink.y);
    }

    @Test
    public void testFixedConstraint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 10.0f, 2);
        SXRPhysicsJoint firstLink = new SXRPhysicsJoint(rootJoint, 1, 10.0f);
        SXRNode box1 = addCube(0f, 0.5f, -30f);
        SXRNode box2 = addCube(0f, 0, 15f);

        scene.addNode(box1);
        box1.addChildObject(box2);
        box1.attachComponent(rootJoint);

        SXRFixedConstraint constraint = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), rootJoint);
        box2.attachComponent(firstLink);
        box2.attachComponent(constraint);
        sxrTestUtils.waitForXFrames(10);

        listener.waitUntilAdded();
        mWorld.setEnable(true);
        float distance = transformDistance(box1.getTransform(), box2.getTransform());

        rootJoint.applyTorque(0, 0, 200);
        listener.waitForXSteps(120);
        Quaternionf rot1 = getWorldRotation(rootJoint);
        Quaternionf rot2 = getWorldRotation(firstLink);
        float rotation = Math.abs(rot1.x - rot2.x) + Math.abs(rot1.y - rot2.y) + Math.abs(rot1.z - rot2.z);
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformDistance(box1.getTransform(), box2.getTransform())) < 0.2);

        firstLink.applyCentralForce(300, 0, 300);
        listener.waitForXSteps(180);
        rot1 = getWorldRotation(rootJoint);
        rot2 = getWorldRotation(firstLink);
        rotation = Math.abs(rot1.x - rot2.x) + Math.abs(rot1.y - rot2.y) + Math.abs(rot1.z - rot2.z);
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformDistance(box1.getTransform(), box2.getTransform())) < 0.2);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testGenericConstraint()
    {
        final float joint[] = {-6f, 0f, 0f};
        final float rotation[] = {1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f};
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint boxJoint = new SXRPhysicsJoint(rootJoint, 1, 1.0f);
        SXRGenericConstraint constraint = new SXRGenericConstraint(sxrTestUtils.getSxrContext(), rootJoint, joint, rotation, rotation);
        SXRNode ball = addSphere(3f, 0f, -10f);
        SXRNode box = addCube(-6, 0f, 0);
        SXRTransform ballTrans = ball.getTransform();

        constraint.setAngularLowerLimits((float) -Math.PI, (float) -Math.PI, (float) -Math.PI);
        constraint.setAngularUpperLimits((float) Math.PI, (float) Math.PI, (float) Math.PI);
        constraint.setLinearLowerLimits(-3f, -10f, -3f);
        constraint.setLinearUpperLimits(3f, 10f, 3f);
        sxrTestUtils.getMainScene().addNode(ball);
        ball.addChildObject(box);
        ball.attachComponent(rootJoint);
        box.attachComponent(boxJoint);
        box.attachComponent(constraint);
        listener.waitUntilAdded();

        mWorld.setEnable(true);
        listener.waitForXSteps(30);

        float anchor[] = {ballTrans.getPositionX(), ballTrans.getPositionY(), ballTrans.getPositionZ()};
        float offsetLimit = 0.005f;

        boxJoint.applyCentralForce(100f, 200f, 100f);
        listener.waitForXSteps(90);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxJoint.applyCentralForce(-100f, 200f, -100f);
        listener.waitForXSteps(90);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxJoint.applyTorque(0f, 1000f, 0f);
        listener.waitForXSteps(180);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxJoint.applyCentralForce(-1000f, 500f, 500f);
        listener.waitForXSteps(180);
        mWaiter.assertTrue(!checkTransformOffset(ballTrans, anchor, offsetLimit));
        sxrTestUtils.waitForXFrames(30);
    }

    private SXRNode addCube(float x, float y, float z)
    {
        SXRContext ctx = sxrTestUtils.getSxrContext();
        SXRMaterial blue = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        SXRCubeNode cubeObject = new SXRCubeNode(ctx, true, blue);
        SXRBoxCollider boxCollider = new SXRBoxCollider(ctx);

        blue.setDiffuseColor(0, 0, 1, 1);
        cubeObject.getTransform().setPosition(x, y, z);
        cubeObject.setName("cube");
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        cubeObject.attachCollider(boxCollider);
        return cubeObject;
    }

    private SXRNode addSphere(float x, float y, float z)
    {
        SXRContext ctx = sxrTestUtils.getSxrContext();
        SXRMaterial red = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        SXRNode sphereObject = new SXRSphereNode(ctx, true, red);

        red.setDiffuseColor(1, 0, 0, 1);
        sphereObject.getTransform().setPosition(x, y, z);
        sphereObject.setName("ball");
        SXRSphereCollider sphereCollider = new SXRSphereCollider(ctx);
        sphereCollider.setRadius(1);
        sphereObject.attachCollider(sphereCollider);
        return sphereObject;
    }

    float transformDistance(SXRTransform a, SXRTransform b)
    {
        Matrix4f ma = a.getModelMatrix4f();
        Matrix4f mb = b.getModelMatrix4f();
        Vector3f pa = new Vector3f();
        Vector3f pb = new Vector3f();

        ma.getTranslation(pa);
        mb.getTranslation(pb);
        return pb.sub(pa).length();
    }

    Vector3f getWorldPosition(SXRPhysicsJoint j)
    {
        Matrix4f m = j.getTransform().getModelMatrix4f();
        Vector3f p = new Vector3f();
        m.getTranslation(p);
        return p;
    }

    Quaternionf getWorldRotation(SXRPhysicsJoint j)
    {
        Matrix4f m = j.getTransform().getModelMatrix4f();
        Quaternionf q = new Quaternionf();
        m.getNormalizedRotation(q);
        return q;
    }

    static boolean checkTransformOffset(SXRTransform tr, float comp[], float limit)
    {
        Matrix4f m = tr.getModelMatrix4f();
        Vector3f p = new Vector3f();

        m.getTranslation(p);
        return (Math.abs(p.x - comp[0]) < limit) &&
               (Math.abs(p.y - comp[1]) < limit) &&
               (Math.abs(p.z - comp[2]) < limit);
    }
}
