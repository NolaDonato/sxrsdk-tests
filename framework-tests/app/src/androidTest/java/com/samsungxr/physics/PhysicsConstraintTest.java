package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;

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
import com.samsungxr.physics.SXRWorld;
import com.samsungxr.nodes.SXRCubeNode;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;
import com.samsungxr.utility.Log;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

public class PhysicsConstraintTest {
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    SXRWorld world;

    private SXRMesh cubeMesh = null;
    private SXRTexture cubeTexture = null;

    private SXRMesh sphereMesh = null;
    private SXRTexture sphereTexture = null;

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
        sxrTestUtils.waitForXFrames(5);
    }

    @Test
    public void fixedConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -15f);

        SXRNode box1 = addCube(sxrTestUtils.getMainScene(), 0f, 0.5f, -30f, 1.0f);
        SXRRigidBody body1 = ((SXRRigidBody)box1.getComponent(SXRRigidBody.getComponentType()));
        body1.setSimulationType(SXRRigidBody.DYNAMIC);

        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), 0f, 0.5f, -15f, 1.0f);
        SXRRigidBody body2 = (SXRRigidBody)box2.getComponent(SXRRigidBody.getComponentType());
        body2.setSimulationType(SXRRigidBody.DYNAMIC);

        SXRFixedConstraint constraint = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), body2);
        box1.attachComponent(constraint);
        sxrTestUtils.waitForXFrames(10);

        listener.waitUntilAdded();
        world.setEnable(true);
        float distance = transformsDistance(box1.getTransform(), box2.getTransform());

        body1.applyTorque(0, 0, 200);
        listener.waitForXSteps(120);
        float rotation = Math.abs(box1.getTransform().getRotationX() - box2.getTransform().getRotationX())
                + Math.abs(box1.getTransform().getRotationY() - box2.getTransform().getRotationY())
                + Math.abs(box1.getTransform().getRotationZ() - box2.getTransform().getRotationZ());
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformsDistance(box1.getTransform(), box2.getTransform())) < 0.2);

        body2.applyCentralForce(300,0,300);
        listener.waitForXSteps(180);
        rotation = Math.abs(box1.getTransform().getRotationX() - box2.getTransform().getRotationX())
                + Math.abs(box1.getTransform().getRotationY() - box2.getTransform().getRotationY())
                + Math.abs(box1.getTransform().getRotationZ() - box2.getTransform().getRotationZ());
        mWaiter.assertTrue(rotation < 0.2f);
        mWaiter.assertTrue(Math.abs(distance - transformsDistance(box1.getTransform(), box2.getTransform())) < 0.2);
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

        boxBody.setSimulationType(SXRRigidBody.DYNAMIC);
        box.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);
        float distance = transformsDistance(ball.getTransform(), box.getTransform());
        mWaiter.assertTrue(distance < 9.8);

        listener.waitForXSteps(60);
        distance = transformsDistance(ball.getTransform(), box.getTransform());
        mWaiter.assertTrue(distance < 9.5);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void hingeConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        float pivotInA[] = {0f, -3f, 0f};
        float pivotInB[] = {0f, 3f, 0f};
        float axisInA[] = {1f, 0f, 0f};
        float axisInB[] = {1f, 0f, 0f};
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
                ballBody, pivotInA, pivotInB, axisInA, axisInB);

        boxBody.setSimulationType(SXRRigidBody.DYNAMIC);
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

        SXRNode box1 = addCube(sxrTestUtils.getMainScene(), 3.0f, 0.5f, -15.0f, 1.0f);
        SXRRigidBody body1 = ((SXRRigidBody) box1.getComponent(SXRRigidBody.getComponentType()));
        body1.setSimulationType(SXRRigidBody.DYNAMIC);

        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), -2.0f, 0.5f, -15.0f, 1.0f);
        SXRRigidBody body2 = ((SXRRigidBody) box1.getComponent(SXRRigidBody.getComponentType()));
        body2.setSimulationType(SXRRigidBody.DYNAMIC);

        SXRSliderConstraint constraint = new SXRSliderConstraint(sxrTestUtils.getSxrContext(), body1);
        constraint.setAngularLowerLimit(-2f);
        constraint.setAngularUpperLimit(2f);
        constraint.setLinearLowerLimit(-5f);
        constraint.setLinearUpperLimit(-2f);

        sxrTestUtils.waitForXFrames(10);
        box2.attachComponent(constraint);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(30);

        body2.applyCentralForce(400f, 0f, 0f);
        listener.waitForXSteps(180);
        float d = transformsDistance(box1.getTransform(), box2.getTransform());
        mWaiter.assertTrue(d >= 2.0f && d <= 5.0f);

        body2.applyCentralForce(-500f, 0f, 0f);
        listener.waitForXSteps(180);
        d = transformsDistance(box1.getTransform(), box2.getTransform());
        mWaiter.assertTrue(d >= 2.0f && d <= 5.0f);

        ((SXRRigidBody)box1.getComponent(SXRRigidBody.getComponentType())).applyTorque(100f, 0f, 0f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void ConeTwistConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        SXRNode box = addCube(sxrTestUtils.getMainScene(), 0f, -5f, -15f, 0);
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0, 5f, -15f, 1);
        SXRTransform boxTrans = box.getTransform();
        SXRTransform ballTrans = ball.getTransform();
        SXRRigidBody boxBody = (SXRRigidBody) box.getComponent(SXRRigidBody.getComponentType());
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        float pivot[] = {0f, -5f, 0f};
        float rotation[] = {0f, -1f, 0f, 1f, 0f, 0f, 0f, 0f, 1f};
        final float maxDistance = (float) (Math.sin(Math.PI * 0.375) * 10.0);
        float d;

        SXRConeTwistConstraint constraint = new SXRConeTwistConstraint(sxrTestUtils.getSxrContext(),
                ballBody, pivot, rotation, rotation);

        sxrTestUtils.waitForXFrames(10);
        box.attachComponent(constraint);
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
    public void GenericConstraintTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, -0.5f, -15f);
        SXRNode box = addCube(sxrTestUtils.getMainScene(), -3f, 0f, -10f, 1f);
        SXRRigidBody boxBody = (SXRRigidBody)box.getComponent(SXRRigidBody.getComponentType());
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 3f, 0f, -10f, 1f);
        SXRRigidBody ballBody = (SXRRigidBody) ball.getComponent(SXRRigidBody.getComponentType());
        final float joint[] = {-6f, 0f, 0f};
        final float rotation[] = {1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f};
        SXRGenericConstraint constraint = new SXRGenericConstraint(sxrTestUtils.getSxrContext(), ballBody, joint, rotation, rotation);
        SXRTransform ballTrans = ball.getTransform();

        boxBody.setSimulationType(SXRRigidBody.DYNAMIC);
        constraint.setAngularLowerLimits((float)-Math.PI, (float)-Math.PI, (float)-Math.PI);
        constraint.setAngularUpperLimits((float)Math.PI, (float)Math.PI, (float)Math.PI);
        constraint.setLinearLowerLimits(-3f, -10f, -3f);
        constraint.setLinearUpperLimits(3f, 10f, 3f);
        box.attachComponent(constraint);
        listener.waitUntilAdded();

        world.setEnable(true);
        listener.waitForXSteps(30);

        float anchor[] = {ballTrans.getPositionX(), ballTrans.getPositionY(), ballTrans.getPositionZ()};
        float offsetLimit = 0.005f;

        boxBody.applyCentralForce(100f, 200f, 100f);
        listener.waitForXSteps(90);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxBody.applyCentralForce(-100f, 200f, -100f);
        listener.waitForXSteps(90);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxBody.applyTorque(0f, 1000f, 0f);
        listener.waitForXSteps(180);
        mWaiter.assertTrue(checkTransformOffset(ballTrans, anchor, offsetLimit));

        boxBody.applyCentralForce(-1000f, 500f, 500f);
        listener.waitForXSteps(180);
        mWaiter.assertTrue(!checkTransformOffset(ballTrans, anchor, offsetLimit));
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
        body.setSimulationType(SXRRigidBody.KINEMATIC);
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
        groundObject.attachComponent(body);


        return groundObject;
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
