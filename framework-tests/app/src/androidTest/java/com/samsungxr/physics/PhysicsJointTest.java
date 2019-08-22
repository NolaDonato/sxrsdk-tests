package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRCameraRig;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMaterial;
import com.samsungxr.SXRMesh;
import com.samsungxr.SXRMeshCollider;
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

import org.joml.AxisAngle4f;
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
    private static final String TAG = "PHYSICS";
    private SXRWorld mWorld;
    private SXRTexture mFloorTex;

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
        try
        {
            mFloorTex = sxrTestUtils.getSxrContext().getAssetLoader().loadTexture(new SXRAndroidResource(sxrTestUtils.getSxrContext(), "floor.jpg"));
        }
        catch (IOException ex)
        {
            mWaiter.fail(ex);
        }
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
    public void testTwoSphericalJoints()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 3);
        SXRPhysicsJoint joint1 = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.SPHERICAL, 1, 1);
        SXRPhysicsJoint joint2 = new SXRPhysicsJoint(joint1, SXRPhysicsJoint.SPHERICAL, 2, 1);

        SXRNode box = addCube(0f, 8, -10);
        SXRNode ball1 = addSphere(0f, -3, 0);
        SXRNode ball2 = addSphere(0f, -3, 0);
        SXRTransform trans1 = ball1.getTransform();
        SXRTransform trans2 = ball2.getTransform();
        Vector3f pos1 = new Vector3f();
        Vector3f pos2 = new Vector3f();

        ball2.getRenderData().getMaterial().setDiffuseColor(0, 1, 0, 1);
        scene.addNode(box);
        ball1.addChildObject(ball2);
        box.addChildObject(ball1);
        ball1.setName("ball1");
        ball2.setName("ball2");
        box.attachComponent(rootJoint);
        ball1.attachComponent(joint1);
        ball2.attachComponent(joint2);
        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);

        mWorld.setEnable(true);
        listener.waitForXSteps(100);
        joint2.applyTorque(0, 0, 100);
        listener.waitForXSteps(100);
        pos1.set(trans1.getPositionX(), trans1.getPositionY(), trans1.getPositionZ());
        pos2.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        float d1 = pos1.length();
        float d2 = pos2.length();

        mWaiter.assertTrue(Math.abs(d1 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(d2 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(pos1.z) < 0.001);
        mWaiter.assertTrue(Math.abs(pos2.z) < 0.001);

        joint1.applyTorque(100, 0, 0);
        listener.waitForXSteps(100);
        pos1.set(trans1.getPositionX(), trans1.getPositionY(), trans1.getPositionZ());
        pos2.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        d1 = pos1.length();
        d2 = pos2.length();
        mWaiter.assertTrue(Math.abs(d1 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(d2 - 3) < 0.001);

        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testTwoHingeJoints()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);

        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 3);
        SXRPhysicsJoint joint1 = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.REVOLUTE, 1, 1);
        SXRPhysicsJoint joint2 = new SXRPhysicsJoint(joint1, SXRPhysicsJoint.REVOLUTE, 2, 1);
        SXRNode box = addCube(0f, 8, -10);
        SXRNode ball1 = addSphere(0f, -3, 0);
        SXRNode ball2 = addSphere(0f, -3, 0);
        SXRTransform trans1 = ball1.getTransform();
        SXRTransform trans2 = ball2.getTransform();
        Vector3f pos1 = new Vector3f();
        Vector3f pos2 = new Vector3f();

        joint1.setAxis(0, 0, 1);
        joint2.setAxis(1, 0, 0);
        ball1.setName("ball1");
        ball2.setName("ball2");
        ball2.getRenderData().getMaterial().setDiffuseColor(0, 1, 0, 1);
        scene.addNode(box);
        ball1.addChildObject(ball2);
        box.addChildObject(ball1);
        box.attachComponent(rootJoint);
        ball1.attachComponent(joint1);
        ball2.attachComponent(joint2);
        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);

        mWorld.setEnable(true);
        listener.waitForXSteps(100);
        joint2.applyTorque(100);
        listener.waitForXSteps(100);
        pos1.set(trans1.getPositionX(), trans1.getPositionY(), trans1.getPositionZ());
        pos2.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        float d1 = pos1.length();
        float d2 = pos2.length();

        mWaiter.assertTrue(Math.abs(d1 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(d2 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(pos1.x) < 0.001);
        mWaiter.assertTrue(Math.abs(pos2.x) < 0.001);

        joint1.applyTorque(100);
        listener.waitForXSteps(100);
        pos1.set(trans1.getPositionX(), trans1.getPositionY(), trans1.getPositionZ());
        pos2.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        d1 = pos1.length();
        d2 = pos2.length();
        mWaiter.assertTrue(Math.abs(d1 - 3) < 0.001);
        mWaiter.assertTrue(Math.abs(d2 - 3) < 0.001);

        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testGravity()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.SPHERICAL, 1, 10);
        SXRNode ground = addGround(0, -8, 0);
        SXRNode box = addCube(0, 3, -10);
        SXRNode ball = addSphere(0, -2, 0);

        ball.getTransform().rotateByAxisWithPivot(-30, 0, 0, 1, 0, 0, 0);
        scene.addNode(ground);
        scene.addNode(box);
        box.addChildObject(ball);
        box.attachComponent(rootJoint);
        ball.attachComponent(firstJoint);

        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);
        mWorld.setEnable(true);
        listener.waitForXSteps(100);

        Vector3f boxPos = getWorldPosition(rootJoint);
        Vector3f ballPos = getWorldPosition(firstJoint);

        mWaiter.assertTrue(boxPos.y > 2);
        mWaiter.assertTrue(ballPos.y > -6);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testSphericalMotor()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.SPHERICAL, 1, 10);
        SXRPhysicsJointMotor motor = new SXRPhysicsJointMotor(sxrTestUtils.getSxrContext(), Float.MAX_VALUE);
        SXRNode ground = addGround(0, -8, 0);
        SXRNode box = addCube(0, 3, -10);
        SXRNode ball = addSphere(0, -2, 0);
        AxisAngle4f rot = new AxisAngle4f((float) Math.PI / 4, new Vector3f(0, 0, 1));
        Quaternionf q = new Quaternionf();

        rot.get(q);
        scene.addNode(ground);
        scene.addNode(box);
        box.addChildObject(ball);
        box.attachComponent(rootJoint);
        ball.attachComponent(firstJoint);
        ball.attachComponent(motor);

        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);
        mWorld.setEnable(true);
        listener.waitForXSteps(10);

        motor.setPositionTarget(q.x, q.y, q.z, q.w);
        listener.waitForXSteps(100);

        SXRTransform t = ball.getTransform();
        Quaternionf r = new Quaternionf(t.getRotationX() - q.x,
                                        t.getRotationY() - q.y,
                                        t.getRotationZ() -q.z,
                                        t.getRotationW() - q.w);

        mWaiter.assertTrue(r.lengthSquared() < 0.0001f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testHingeMotor()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.REVOLUTE, 1, 10);
        SXRPhysicsJointMotor motor = new SXRPhysicsJointMotor(sxrTestUtils.getSxrContext(), Float.MAX_VALUE);
        SXRNode ground = addGround(0, -8, 0);
        SXRNode box = addCube(0, 3, -10);
        SXRNode ball = addSphere(0, -2, 0);
        AxisAngle4f rot = new AxisAngle4f((float) Math.PI / 4, new Vector3f(0, 0, 1));
        Quaternionf q = new Quaternionf();

        rot.get(q);

        firstJoint.setAxis(0, 0, 1);
        scene.addNode(ground);
        scene.addNode(box);
        box.addChildObject(ball);
        box.attachComponent(rootJoint);
        ball.attachComponent(firstJoint);
        ball.attachComponent(motor);

        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);
        mWorld.setEnable(true);
        listener.waitForXSteps(10);

        motor.setPositionTarget(0, (float) Math.PI / 4);
        listener.waitForXSteps(100);
        SXRTransform t = ball.getTransform();
        Quaternionf r = new Quaternionf(t.getRotationX() - q.x,
                                        t.getRotationY() - q.y,
                                        t.getRotationZ() -q.z,
                                        t.getRotationW() - q.w);

        mWaiter.assertTrue(r.lengthSquared() < 0.0001f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testHingeJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        mWorld.getEventReceiver().addListener(listener);

        SXRNode ball = addSphere( 0, 8, -10);
        SXRNode box = addCube(0, -8, 0);
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.REVOLUTE, 1, 1);

        boxTrans.rotateByAxisWithPivot(-30, 0, 0, 1, 0, 0, 0);
        firstJoint.setAxis(0, 0, 1);
        sxrTestUtils.getMainScene().addNode(ball);
        ball.addChildObject(box);
        ball.attachComponent(rootJoint);
        box.attachComponent(firstJoint);
        listener.waitUntilAdded();
        sxrTestUtils.waitForXFrames(10);

        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();
        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float dist = boxPos.sub(ballPos).length();
        mWorld.setEnable(true);

        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float d = boxPos.sub(ballPos).length();
        mWaiter.assertTrue(boxPos.y < -7);
        mWaiter.assertTrue(ballPos.y == 8.0f);
        mWaiter.assertTrue(Math.abs(d - dist) < 0.5);

        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        d = boxPos.sub(ballPos).length();
        mWaiter.assertTrue(boxPos.y < -7);
        mWaiter.assertTrue(ballPos.y == 8.0f);
        mWaiter.assertTrue(Math.abs(d - dist) < 0.5);

        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testSliderJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(0f, -8, 0);
        SXRNode box1 = addCube(3, -2.5f, -15);
        SXRNode box2 = addCube(-3, 0, 5);
        SXRPhysicsJoint body1 = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint body2 = new SXRPhysicsJoint(body1, SXRPhysicsJoint.PRISMATIC, 1, 1);

        box1.getRenderData().getMaterial().setDiffuseColor(1, 0, 0, 1);
        box1.addChildObject(box2);
        sxrTestUtils.getMainScene().addNode(ground);
        sxrTestUtils.getMainScene().addNode(box1);
        box1.setName("cube1");
        box2.setName("cube2");
        box1.attachComponent(body1);
        box2.attachComponent(body2);
        sxrTestUtils.waitForXFrames(10);

        Vector3f pos1 = getWorldPosition(body1);
        Vector3f pos2 = getWorldPosition(body2);
        Vector3f sliderAxis = new Vector3f();

        pos1.sub(pos2, sliderAxis);
        sliderAxis.normalize();

        listener.waitUntilAdded();
        mWorld.setEnable(true);
        listener.waitForXSteps(30);

        body2.applyTorque(-100);
        listener.waitForXSteps(180);
        pos1 = getWorldPosition(body1);
        pos2 = getWorldPosition(body2);
        Vector3f posDiff = new Vector3f();

        pos1.sub(pos2, posDiff);
        posDiff.normalize();
        float dp = posDiff.dot(sliderAxis);

        mWaiter.assertTrue((dp < 0.3f) || (dp > 0.8f));

        body2.applyTorque(100);
        listener.waitForXSteps(180);
        pos1 = getWorldPosition(body1);
        pos2 = getWorldPosition(body2);
        pos1.sub(pos2, posDiff);
        posDiff.normalize();
        dp = Math.abs(posDiff.dot(sliderAxis));
        mWaiter.assertTrue((dp < 0.3f) || (dp > 0.8f));
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

    private SXRNode addGround(float x, float y, float z)
    {
        SXRContext ctx = sxrTestUtils.getSxrContext();
        SXRMaterial mtl = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        SXRNode ground = new SXRCubeNode(ctx, true, mtl, new Vector3f(100, 10, 100));
        SXRMeshCollider collider = new SXRMeshCollider(ctx, true);
        SXRRigidBody body = new SXRRigidBody(ctx, 0.0f);

        mtl.setMainTexture(mFloorTex);
        ground.getTransform().setPosition(x, y, z);
        ground.setName("ground");
        ground.attachCollider(collider);
        body.setRestitution(0.5f);
        body.setFriction(1.0f);
        ground.attachComponent(body);
        return ground;
    }


    Vector3f getWorldPosition(SXRPhysicsJoint j)
    {
        Matrix4f m = j.getTransform().getModelMatrix4f();
        Vector3f p = new Vector3f();
        m.getTranslation(p);
        return p;
    }

}
