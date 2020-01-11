package com.samsungxr.physics;

import androidx.test.rule.ActivityTestRule;

import net.jodah.concurrentunit.Waiter;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMesh;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTexture;
import com.samsungxr.SXRTransform;
import com.samsungxr.physics.SXRConeTwistConstraint;
import com.samsungxr.physics.SXRFixedConstraint;
import com.samsungxr.physics.SXRGenericConstraint;
import com.samsungxr.physics.SXRHingeConstraint;
import com.samsungxr.physics.SXRPoint2PointConstraint;
import com.samsungxr.physics.SXRRigidBody;
import com.samsungxr.physics.SXRSliderConstraint;
import com.samsungxr.physics.SXRUniversalConstraint;
import com.samsungxr.physics.SXRWorld;
import com.samsungxr.nodes.SXRCubeNode;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;
import com.samsungxr.utility.Log;

import org.joml.AxisAngle4f;
import org.joml.Matrix4f;
import org.joml.Quaterniond;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.TimeoutException;

public class PhysicsConstraintTest {
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    private SXRWorld world;
    private SXRMesh cubeMesh = null;
    private SXRTexture cubeTexture = null;
    private SXRMesh sphereMesh = null;
    private SXRTexture sphereTexture = null;
    private boolean mEnableDebug = false;

    @Rule
    public ActivityTestRule<SXRTestableActivity> ActivityRule = new
            ActivityTestRule<SXRTestableActivity>(SXRTestableActivity.class);

    @Before
    public void setUp() throws TimeoutException {
        mWaiter = new Waiter();
        sxrTestUtils = new SXRTestUtils(ActivityRule.getActivity());
        sxrTestUtils.waitForOnInit();
        SXRContext ctx = sxrTestUtils.getSxrContext();
        ctx.getMainScene().getMainCameraRig().getTransform().setPosition(0.0f, 6.0f, 0.0f);
        world = new SXRWorld(sxrTestUtils.getMainScene());

        if (mEnableDebug)
        {
            SXRNode debugDraw = world.setupDebugDraw(0);
            sxrTestUtils.getMainScene().addNode(debugDraw);
            world.setDebugMode(-1);
        }
        sxrTestUtils.waitForXFrames(5);
    }

    @Test
    public void fixedConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -15f);
        SXRNode box1 = addCube(sxrTestUtils.getMainScene(), 3f, 0.75f, -15f, 1.0f);
        SXRRigidBody body1 = ((SXRRigidBody)box1.getComponent(SXRRigidBody.getComponentType()));
        SXRTransform trans1 = box1.getTransform();
        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), -3f, 0.75f, -15f, 1.0f);
        SXRRigidBody body2 = (SXRRigidBody)box2.getComponent(SXRRigidBody.getComponentType());
        SXRTransform trans2 = box2.getTransform();
        SXRFixedConstraint constraint = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), body2);

        box1.attachComponent(constraint);
        sxrTestUtils.waitForXFrames(10);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(30);
        float distance = transformsDistance(trans1, trans2);

        body1.applyTorque(100, 0, 0);
        listener.waitForXSteps(30);
        AxisAngle4f rotDir = getRotation(trans1, trans2);
        float d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(rotDir.angle) < 0.001f);
        mWaiter.assertTrue(Math.abs(distance - d) < 0.001f);

        body2.applyCentralForce(300,0,300);
        listener.waitForXSteps(30);
        rotDir = getRotation(trans1, trans2);
        mWaiter.assertTrue(Math.abs(rotDir.angle) < 0.001f);
        d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(distance - d) < 0.001f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void point2pointConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        float pivotInA[] = {0f, -1.5f, 0f};
        float pivotInB[] = {-8f, -1.5f, 0f};
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0.0f, 10.0f, -10.0f, 0.0f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), 8.0f, 10.0f, -10.0f, 1.0f);
        SXRRigidBody boxBody = ((SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType()));
        SXRRigidBody ballBody = ((SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType()));
        SXRPoint2PointConstraint constraint = new SXRPoint2PointConstraint(sxrTestUtils.getSxrContext(), ballBody, pivotInA, pivotInB);

        box.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);
        float distance = transformsDistance(ball.getTransform(), box.getTransform());
        mWaiter.assertTrue(distance < 9.8);

        listener.waitForXSteps(60);
        distance = transformsDistance(ball.getTransform(), box.getTransform());
        mWaiter.assertTrue(distance < 9.8);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void hingeConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        float pivotInA[] = {0f, -3f, 0f};
        float pivotInB[] = {0f, 3f, 0f};
        float axisIn[] = {1f, 0f, 0f};
        final float maxDistanceX = 0.000001f;
        final float minDistanceY = 3.f + 1.62f;
        final float maxDIstanceZ = 2.52f;

        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0.0f, 10.0f, -10.0f, 0.0f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), 0.0f, 4.0f, -10.0f, 1.0f);
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();
        SXRRigidBody boxBody = (SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType());
        SXRRigidBody ballBody = ((SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType()));
        SXRHingeConstraint constraint = new SXRHingeConstraint(sxrTestUtils.getSxrContext(),
                ballBody, pivotInA, pivotInB, axisIn);

        constraint.setLimits(-1f, 1f);
        box.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);

        listener.waitForXSteps(30);
        boxBody.applyCentralForce(0f, 0f, 1000f); // Must move towards camera
        listener.waitForXSteps(180);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float dx = Math.abs(ballPos.x - boxPos.x);
        float dy = Math.abs(ballPos.y - boxPos.y);
        float dz = Math.abs(ballPos.z - boxPos.z);
        Log.d("PHYSICS", "dx = %f, dy = %f, dz = %f", dx, dy, dz);
        mWaiter.assertTrue(dx < maxDistanceX && dy > minDistanceY && dz < maxDIstanceZ);

        boxBody.applyCentralForce(0f, 1000f, 0f); // May have some effect
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        dy = Math.abs(ballPos.y - boxPos.y);
        dz = Math.abs(ballPos.z - boxPos.z);
        Log.d("PHYSICS", "dx = %f, dy = %f, dz = %f", dx, dy, dz);
        mWaiter.assertTrue(dx < maxDistanceX && dy > minDistanceY && dz < maxDIstanceZ);

        boxBody.applyCentralForce(1000f, 0f, 0f); // Must have no effect
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        dy = Math.abs(ballPos.y - boxPos.y);
        dz = Math.abs(ballPos.z - boxPos.z);
        Log.d("PHYSICS", "dx = %f, dy = %f, dz = %f", dx, dy, dz);
        mWaiter.assertTrue(dx < maxDistanceX && dy > minDistanceY && dz < maxDIstanceZ);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void sliderConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -15f);
        SXRNode box1 = addCube(sxrTestUtils.getMainScene(), 3.0f, 2, -15.0f, 1.0f);
        SXRRigidBody body1 = ((SXRRigidBody) box1.getComponent(SXRRigidBody.getComponentType()));
        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), -2.0f, 2, -15.0f, 1.0f);
        SXRRigidBody body2 = ((SXRRigidBody) box2.getComponent(SXRRigidBody.getComponentType()));
        SXRSliderConstraint constraint = new SXRSliderConstraint(sxrTestUtils.getSxrContext(), body1,
                                                                 new Vector3f(0, 0, 0),
                                                                 new Vector3f(-5, 0, 0));
        SXRTransform trans1 = box1.getTransform();
        SXRTransform trans2 = box2.getTransform();
        Vector3f pos2 = new Vector3f(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        Vector3f sliderAxis = new Vector3f(pos2.x - trans1.getPositionX(),
                                           pos2.y - trans1.getPositionY(),
                                           pos2.z - trans1.getPositionZ());

        sliderAxis.normalize();
        sxrTestUtils.waitForXFrames(10);
        box2.attachComponent(constraint);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(10);

        body2.applyCentralForce(100f, 0f, 0f);
        listener.waitForXSteps(180);
        Vector3f p = new Vector3f(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        Vector3f dir = new Vector3f(p.x - trans1.getPositionX(),
                                    p.y - trans1.getPositionY(),
                                    p.z - trans1.getPositionZ());
        dir.normalize();
        float dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.01f);

        body2.applyCentralForce(-100f, 0f, 0f);
        listener.waitForXSteps(180);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        p.sub(pos2);
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.01f);
        mWaiter.assertTrue(p.x < 0.3f);
        mWaiter.assertTrue(p.y < 0.01f);
        mWaiter.assertTrue(p.z < 0.01f);

        body1.applyCentralForce(100f, 0f, -100f);
        listener.waitForXSteps(100);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.5f);

        body2.applyTorque(0, 100f, 0f);
        listener.waitForXSteps(100);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.5f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void sliderConstraintLimitTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -10);

        SXRNode box1 = addCube(sxrTestUtils.getMainScene(), 2, 0.75f, -15, 1);
        SXRRigidBody body1 = ((SXRRigidBody) box1.getComponent(SXRRigidBody.getComponentType()));
        SXRTransform trans1 = box1.getTransform();
        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), -2, 0.75f, -10, 1);
        SXRRigidBody body2 = ((SXRRigidBody) box2.getComponent(SXRRigidBody.getComponentType()));
        SXRTransform trans2 = box2.getTransform();
        Vector3f    slideDir = getDirection(trans1, trans2);
        SXRSliderConstraint constraint = new SXRSliderConstraint(sxrTestUtils.getSxrContext(), body1);
        float deg30 = (float) Math.toRadians(30);
        float linearLimit = 2.0f;

        constraint.setAngularLowerLimit(-deg30);
        constraint.setAngularUpperLimit(deg30);
        constraint.setLinearLowerLimit(-linearLimit);
        constraint.setLinearUpperLimit(linearLimit);
        slideDir.normalize();
        sxrTestUtils.waitForXFrames(10);
        box2.attachComponent(constraint);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(50);
        Vector3f dir = getDirection(trans1, trans2);
        float origDist = dir.length();
        mWaiter.assertTrue(origDist <= linearLimit);

        body2.applyCentralForce(100f, 0f, 0f);
        listener.waitForXSteps(100);
        dir = getDirection(trans1, trans2);
        float d = dir.length();
        mWaiter.assertTrue(d <= linearLimit);
        dir.normalize();
        d = Math.abs(dir.dot(slideDir));
        mWaiter.assertTrue((1.0f - d) < 0.01f);

        body2.applyCentralForce(-100f, 0f, 0f);
        listener.waitForXSteps(100);
        dir = getDirection(trans1, trans2);
        d = dir.length();
        mWaiter.assertTrue(d <= linearLimit);
        dir.normalize();
        d = Math.abs(dir.dot(slideDir));
        mWaiter.assertTrue((1.0f - d) < 0.01f);

        body1.applyTorque(-200, 0, 0);
        listener.waitForXSteps(50);
        AxisAngle4f aa = getRotation(trans2, trans1);
        mWaiter.assertTrue(Math.abs(aa.angle) < deg30);

        body1.applyCentralForce(-100f, 0f, -100f);
        listener.waitForXSteps(100);
        dir = getDirection(trans1, trans2);
        d = dir.length();
        mWaiter.assertTrue(d <= linearLimit);

        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void coneTwistConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        SXRNode box = addCube(sxrTestUtils.getMainScene(), 0f, 5f, -15f, 0);
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0, -5f, -15f, 1);
        SXRTransform boxTrans = box.getTransform();
        SXRTransform ballTrans = ball.getTransform();
        SXRRigidBody boxBody = (SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType());
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        final Vector3f pivotA = new Vector3f(0, 0, 0);
        final Vector3f pivotB = new Vector3f(0f, 5f, 0f);
        final Vector3f coneAxis = new Vector3f(0, -1, 0);
        final float maxDistance = (float) (Math.sin(Math.PI * 0.375) * 10.0);
        float d;

        SXRConeTwistConstraint constraint = new SXRConeTwistConstraint(sxrTestUtils.getSxrContext(),
                boxBody, pivotA, pivotB, coneAxis);

        boxBody.setSimulationType(SXRRigidBody.KINEMATIC);
        sxrTestUtils.waitForXFrames(10);
        ball.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);

        ballBody.applyCentralForce(100f, 0f, 100f);
        listener.waitForXSteps(180);
        d = transformsDistance(ballTrans, boxTrans);
        mWaiter.assertTrue(maxDistance >= d);

        ballBody.applyCentralForce(-500f, 0f, 0f);
        listener.waitForXSteps(180);
        d = transformsDistance(ballTrans, boxTrans);
        mWaiter.assertTrue(maxDistance >= d);
        sxrTestUtils.waitForXFrames(30);
   }

    @Test
    public void genericConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, -0.5f, -15f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), -3f, 0f, -10f, 1f);
        SXRRigidBody boxBody = (SXRRigidBody)box.getComponent(SXRRigidBody.getComponentType());
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 3f, 0f, -10f, 1f);
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        final Vector3f pivotA = new Vector3f(0, 0f, 0f);
        final Vector3f pivotB = new Vector3f(6f, 0, 0);
        SXRGenericConstraint constraint = new SXRGenericConstraint(sxrTestUtils.getSxrContext(), ballBody, pivotA, pivotB);
        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();

        sxrTestUtils.waitForXFrames(10);
        box.attachComponent(constraint);
        listener.waitUntilAdded();

        world.setEnable(true);
        listener.waitForXSteps(30);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float dx = Math.abs(ballPos.x - boxPos.x);
        float dy = Math.abs(ballPos.y - boxPos.y);
        float dz = Math.abs(ballPos.z - boxPos.z);
        float roll = boxTrans.getRotationRoll() % (float) Math.PI;
        float boxY = boxTrans.getPositionY();
        float dist = dx * dx + dy * dy + dz * dz;
        float origDist = dist;

        boxBody.applyCentralForce(300f, 0, 0);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        dy = Math.abs(ballPos.y - boxPos.y);
        dz = Math.abs(ballPos.z - boxPos.z);
        dist = dx * dx + dy * dy + dz * dz;
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(dist - origDist) < 0.5f);
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 0.01f);
        mWaiter.assertTrue(Math.abs(roll) < 3);

        boxBody.applyCentralForce(-300f, 0, -300f);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        dy = Math.abs(ballPos.y - boxPos.y);
        dz = Math.abs(ballPos.z - boxPos.z);
        dist = dx * dx + dy * dy + dz * dz;
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(dist - origDist) < 0.5f);
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 0.01f);
        mWaiter.assertTrue(Math.abs(roll) < 3);

        boxBody.applyTorque(300f, 300, 0);
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        dy = Math.abs(ballPos.y - boxPos.y);
        dz = Math.abs(ballPos.z - boxPos.z);
        dist = dx * dx + dy * dy + dz * dz;
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(dist - origDist) < 0.5f);
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 0.01f);
        mWaiter.assertTrue(Math.abs(roll) < 3);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void genericLinearLimitsTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, -0.5f, -15f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), -3f, 0f, -10f, 1f);
        SXRRigidBody boxBody = (SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType());
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 3f, 0f, -10f, 1f);
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        final Vector3f pivotA = new Vector3f(0, 0f, 0f);
        final Vector3f pivotB = new Vector3f(6f, 0, 0);
        SXRGenericConstraint constraint = new SXRGenericConstraint(sxrTestUtils.getSxrContext(), ballBody, pivotA, pivotB);
        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();

        constraint.setLinearLowerLimits(-1, 0, 0);
        constraint.setLinearUpperLimits(1, 0, 0);
        sxrTestUtils.waitForXFrames(10);
        box.attachComponent(constraint);
        listener.waitUntilAdded();

        world.setEnable(true);
        listener.waitForXSteps(30);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float dx = Math.abs(ballPos.x - boxPos.x);
        float dy = Math.abs(ballPos.y - boxPos.y);
        float dz = Math.abs(ballPos.z - boxPos.z);
        float roll = boxTrans.getRotationRoll() % (float) Math.PI;
        float boxY = boxTrans.getPositionY();

        boxBody.applyCentralForce(300f, 0, 0);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dx) < 7);
        mWaiter.assertTrue(Math.abs(roll) < 4);

        boxBody.applyCentralForce(0, 0, -300f);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dx) < 6);
        mWaiter.assertTrue(Math.abs(roll) < 4);
        constraint.setLinearLowerLimits(0, 0, -1);
        constraint.setLinearUpperLimits(0, 0, 1);

        ballBody.applyCentralForce(-300f, 0, 0);
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dx = Math.abs(ballPos.x - boxPos.x);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dx) < 6);
        mWaiter.assertTrue(Math.abs(roll) < 4);

        boxBody.applyCentralForce(300f, 0, 0);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dz = Math.abs(ballPos.z - boxPos.z);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dz) < 7);
        mWaiter.assertTrue(Math.abs(roll) < 4);

        boxBody.applyCentralForce(0, 0, -300f);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dz = Math.abs(ballPos.z - boxPos.z);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dz) < 6);
        mWaiter.assertTrue(Math.abs(roll) < 4);

        ballBody.applyCentralForce(-300f, 0, 0);
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dz = Math.abs(ballPos.z - boxPos.z);
        roll = boxTrans.getRotationRoll() % (float) Math.PI;
        mWaiter.assertTrue(Math.abs(boxY - boxPos.y) < 1);
        mWaiter.assertTrue(Math.abs(dz) < 6);
        mWaiter.assertTrue(Math.abs(roll) < 4);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void genericAngularLimitsTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, -0.5f, -15f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), -3f, 0f, -10f, 1f);
        SXRRigidBody boxBody = (SXRRigidBody)box.getComponent(SXRRigidBody.getComponentType());
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 3f, 0f, -10f, 1f);
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        final Vector3f pivotA = new Vector3f(0, 0f, 0f);
        final Vector3f pivotB = new Vector3f(6f, 0, 0);
        SXRGenericConstraint constraint = new SXRGenericConstraint(sxrTestUtils.getSxrContext(), ballBody, pivotA, pivotB);
        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();
        float deg45 = (float) Math.PI / 4;

        constraint.setAngularLowerLimits(0, -deg45, 0);
        constraint.setAngularUpperLimits(0, deg45, 0);
        sxrTestUtils.waitForXFrames(10);
        box.attachComponent(constraint);
        listener.waitUntilAdded();

        world.setEnable(true);
        listener.waitForXSteps(30);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        Matrix4f frameB = new Matrix4f();
        Vector3f rotB = new Vector3f();
        float dist = ballPos.distance(boxPos);
        float origDist = dist;

        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        boxBody.applyCentralForce(300f, 0, 0);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);
        mWaiter.assertTrue(Math.abs(rotB.y) <= (deg45 + 0.1f));

        boxBody.applyCentralForce(0, 0, -300f);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);
        mWaiter.assertTrue(Math.abs(rotB.y) <= (deg45 + 0.01f));

        ballBody.applyCentralForce(-300f, 0, 0);
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);
        mWaiter.assertTrue(Math.abs(rotB.y) <= (deg45 + 0.1f));

        constraint.setAngularLowerLimits(0, -deg45, 0);
        constraint.setAngularUpperLimits(0, deg45, 0);
        boxBody.applyCentralForce(300f, 0, 0);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);
        mWaiter.assertTrue(Math.abs(rotB.y) <= (deg45 + 0.1f));

        boxBody.applyCentralForce(0, 0, -300f);
        listener.waitForXSteps(90);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);

        ballBody.applyCentralForce(-300f, 0, 0);
        listener.waitForXSteps(180);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 1);
        mWaiter.assertTrue(Math.abs(rotB.y) <= (deg45 + 0.1f));
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void universalConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 5);
        world.getEventReceiver().addListener(listener);
        SXRNode root = addCube(sxrTestUtils.getMainScene(), 0, 10, -10f, 0);
        SXRRigidBody rootBody = (SXRRigidBody) root.getComponent(SXRRigidBody.getComponentType());
        SXRNode box = addCube(sxrTestUtils.getMainScene(), 0, 10, -10f, 1);
        SXRRigidBody boxBody = (SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType());
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0, 5, -10f, 1f);
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        SXRFixedConstraint fixed = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), rootBody);
        SXRUniversalConstraint constraint = new SXRUniversalConstraint(sxrTestUtils.getSxrContext(),
                                                                       boxBody,
                                                                       new Vector3f(0, 10, -10),
                                                                       new Vector3f(0, 0, 1),
                                                                       new Vector3f(1, 0, 0));
        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();
        float deg45 = (float) Math.PI / 4;

        sxrTestUtils.waitForXFrames(10);
        rootBody.setIgnoreCollisionCheck(boxBody, true);
        boxBody.setIgnoreCollisionCheck(rootBody, true);
        box.attachComponent(fixed);
        ball.attachComponent(constraint);
        listener.waitUntilAdded();

        world.setEnable(true);
        listener.waitForXSteps(200);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        Matrix4f frameB = new Matrix4f();
        Vector3f rotB = new Vector3f();
        float dist = ballPos.distance(boxPos);
        float origDist = dist;

        ballBody.applyCentralForce(300, 0, 0);
        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        boxMtx.invert().mul(ballMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 3);
        mWaiter.assertTrue(Math.abs(frameB.m30()) < 2.0);
        mWaiter.assertTrue(Math.abs(frameB.m32()) < 1.0);

        ballBody.applyCentralForce(0, 0, 300);
        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        boxMtx.invert().mul(ballMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 3);
        mWaiter.assertTrue(Math.abs(frameB.m30()) < 2.0);
        mWaiter.assertTrue(Math.abs(frameB.m32()) < 1.0);

        ballBody.applyTorque(100, 0, 0);
        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        boxMtx.invert().mul(ballMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        dist = ballPos.distance(boxPos);
        mWaiter.assertTrue(Math.abs(dist - origDist) < 3);
        mWaiter.assertTrue(Math.abs(frameB.m30()) < 2.0);
        mWaiter.assertTrue(Math.abs(frameB.m32()) < 1.0);

        constraint.setAngularLowerLimits(0,  -deg45, 0);
        constraint.setAngularUpperLimits(0,  deg45, 0);
        boxBody.applyCentralForce(150, 0, -150f);
        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        mWaiter.assertTrue(Math.abs(rotB.x) <= (deg45 + 0.1f));

        ballBody.applyCentralForce(-150f, 0, 150f);
        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();
        ballMtx.invert().mul(boxMtx, frameB);
        frameB.getEulerAnglesZYX(rotB);
        mWaiter.assertTrue(Math.abs(rotB.x) <= (deg45 + 0.1f));

        sxrTestUtils.waitForXFrames(30);
    }
    /*
    * Function to add a sphere of dimension and position specified in the
    * Bullet physics world and scene graph
    */
    private SXRNode addSphere(SXRScene scene, float x, float y, float z, float mass) {

        if (sphereMesh == null) {
            try {
                sphereMesh = sxrTestUtils.getSxrContext().getAssetLoader().loadMesh(
                        new SXRAndroidResource(sxrTestUtils.getSxrContext(), "sphere.obj"));
                sphereTexture = sxrTestUtils.getSxrContext().getAssetLoader().loadTexture(
                        new SXRAndroidResource(sxrTestUtils.getSxrContext(), "sphere.jpg"));
            } catch (IOException e) {

            }
        }

        SXRNode sphereObject = new SXRNode(sxrTestUtils.getSxrContext(), sphereMesh, sphereTexture);

        sphereObject.getTransform().setScaleX(0.5f);
        sphereObject.getTransform().setScaleY(0.5f);
        sphereObject.getTransform().setScaleZ(0.5f);
        sphereObject.getTransform().setPosition(x, y, z);
        sphereObject.setName("ball");
        scene.addNode(sphereObject);

        // Collider
        SXRSphereCollider sphereCollider = new SXRSphereCollider(sxrTestUtils.getSxrContext());
        sphereCollider.setRadius(0.5f);
        sphereObject.attachCollider(sphereCollider);

        // Physics body
        SXRRigidBody mSphereRigidBody = new SXRRigidBody(sxrTestUtils.getSxrContext(), mass);

        sphereObject.attachComponent(mSphereRigidBody);
        return sphereObject;
    }

    private SXRNode addCube(SXRScene scene, float x, float y, float z, float mass) {

        if (cubeMesh == null) {
            try {
                cubeMesh = sxrTestUtils.getSxrContext().getAssetLoader().loadMesh(
                        new SXRAndroidResource(sxrTestUtils.getSxrContext(), "cube.obj"));
                cubeTexture = sxrTestUtils.getSxrContext().getAssetLoader().loadTexture(
                        new SXRAndroidResource(sxrTestUtils.getSxrContext(), "cube.jpg"));
            } catch (IOException e) {

            }
        }

        SXRNode cubeObject = new SXRNode(sxrTestUtils.getSxrContext(), cubeMesh, cubeTexture);

        cubeObject.getTransform().setPosition(x, y, z);
        cubeObject.setName("box");
        scene.addNode(cubeObject);

        // Collider
        SXRBoxCollider boxCollider = new SXRBoxCollider(sxrTestUtils.getSxrContext());
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        cubeObject.attachCollider(boxCollider);

        // Physics body
        SXRRigidBody body = new SXRRigidBody(sxrTestUtils.getSxrContext(), mass);
        cubeObject.attachComponent(body);
        return cubeObject;
    }

    private SXRNode addGround(SXRScene scene, float x, float y, float z) {

        SXRNode groundObject = new SXRCubeNode(sxrTestUtils.getSxrContext());

        groundObject.getTransform().setScale(100f, 0.5f, 100f);
        groundObject.getTransform().setPosition(x, y, z);
        groundObject.setName("ground");
        scene.addNode(groundObject);

        SXRBoxCollider boxCollider = new SXRBoxCollider(sxrTestUtils.getSxrContext());
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        groundObject.attachCollider(boxCollider);

        SXRRigidBody body = new SXRRigidBody(sxrTestUtils.getSxrContext(), 0.0f);
        body.setScale(100, 0.5f, 100);
        groundObject.attachComponent(body);


        return groundObject;
    }

    Vector3f getWorldPosition(SXRPhysicsWorldObject j)
    {
        Matrix4f m = j.getTransform().getModelMatrix4f();
        Vector3f p = new Vector3f();
        m.getTranslation(p);
        return p;
    }

    /*
     * Returns the vector to go from the origin of TransformA
     * space to the origin of TransformB space in the
     * context of TransformA
     */
    Vector3f getDirection(SXRTransform a, SXRTransform b)
    {
        Matrix4f ma = a.getModelMatrix4f();
        Matrix4f mb = b.getModelMatrix4f();
        Vector3f pb = new Vector3f();

        ma.invert();
        ma.mul(mb, mb);
        mb.getTranslation(pb);
        return pb;
    }

    /*
     * Returns the rotation to go from the TransformA
     * space to TransformB space in the context of TransformA
     */
    AxisAngle4f getRotation(SXRTransform a, SXRTransform b)
    {
        Matrix4f ma = a.getModelMatrix4f();
        Matrix4f mb = b.getModelMatrix4f();
        AxisAngle4f aa = new AxisAngle4f();

        ma.invert();
        ma.mul(mb, mb);
        mb.getRotation(aa);
        return aa;
    }

    static float transformsDistance(SXRTransform a, SXRTransform b) {
        float x = a.getPositionX() - b.getPositionX();
        float y = a.getPositionY() - b.getPositionY();
        float z = a.getPositionZ() - b.getPositionZ();

        return (float)Math.sqrt(x * x + y * y + z * z);
    }

    static boolean checkTransformOffset(SXRTransform tr, float comp[], float limit) {
        return Math.abs(tr.getPositionX() - comp[0]) < limit
                && Math.abs(tr.getPositionY() - comp[1]) < limit
                && Math.abs(tr.getPositionZ() - comp[2]) < limit;
    }
}
