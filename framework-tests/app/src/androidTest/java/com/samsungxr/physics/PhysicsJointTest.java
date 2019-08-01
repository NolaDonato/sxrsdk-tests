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
    public void testTorque()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, 1, 10);
        SXRNode ground = addGround(0, -8, 0);
        SXRNode box = addCube(0f, 3, -10);
        SXRNode ball = addSphere(0f, -2, 0);

        scene.addNode(ground);
        scene.addNode(box);
        box.addChildObject(ball);
        box.attachComponent(rootJoint);
        ball.attachComponent(firstJoint);

        listener.waitUntilAdded();
        mWorld.setEnable(true);
        firstJoint.applyTorque(100,0,0);
        listener.waitForXSteps(100);

        Vector3f boxPos = getWorldPosition(rootJoint);
        Vector3f ballPos = getWorldPosition(firstJoint);

        mWaiter.assertTrue(boxPos.y > -8);
        mWaiter.assertTrue(ballPos.y > -8);
        sxrTestUtils.waitForXFrames(30);
    }


    @Test
    public void testGravity()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 10, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, 1, 10);
        SXRNode ground = addGround(0, -8, 0);
        SXRNode box = addCube(0f, 3, -10);
        SXRNode ball = addSphere(0f, -2, 0);

        scene.addNode(ground);
        scene.addNode(box);
        box.addChildObject(ball);
        box.attachComponent(rootJoint);
        ball.attachComponent(firstJoint);

        listener.waitUntilAdded();
        mWorld.setEnable(true);
        listener.waitForXSteps(100);

        Vector3f boxPos = getWorldPosition(rootJoint);
        Vector3f ballPos = getWorldPosition(firstJoint);

        mWaiter.assertTrue(boxPos.y > -3);
        mWaiter.assertTrue(ballPos.y > -3);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void testHingeConstraint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        mWorld.getEventReceiver().addListener(listener);

        float pivotInA[] = { 0f, -3f, 0f };
        float pivotInB[] = { 0f, 3f, 0f };
        float axisInA[] = { 0, 0, 1 };
        float axisInB[] = { 0, 0, 1 };

        Vector3f ballPos = new Vector3f();
        Vector3f boxPos = new Vector3f();
        SXRNode ball = addSphere( 0.0f, 8.0f, -10.0f);
        SXRNode box = addCube(0.0f, -8.0f, 0);
        SXRTransform ballTrans = ball.getTransform();
        SXRTransform boxTrans = box.getTransform();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), 0, 2);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, 1, 1);
        SXRHingeConstraint constraint = new SXRHingeConstraint(sxrTestUtils.getSxrContext(),
                rootJoint, pivotInA, pivotInB, axisInA, axisInB);

        constraint.setLimits(-1f, 1f);
        sxrTestUtils.getMainScene().addNode(ball);
        ball.addChildObject(box);
        ball.attachComponent(rootJoint);
        box.attachComponent(firstJoint);
        box.attachComponent(constraint);
        listener.waitUntilAdded();
        mWorld.setEnable(true);

        listener.waitForXSteps(10);
        firstJoint.applyTorque(100, 100, 100);
        listener.waitForXSteps(100);
        Matrix4f ballMtx = ballTrans.getLocalModelMatrix4f();
        Matrix4f boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        float dist = boxPos.sub(ballPos).length();
        mWaiter.assertTrue(boxPos.y < -7);
        mWaiter.assertTrue(Math.abs(boxPos.x) < 1.3);
        mWaiter.assertTrue(ballPos.y == 8);
        mWaiter.assertTrue((dist < 19) && (dist > 18));

        listener.waitForXSteps(100);
        ballMtx = ballTrans.getLocalModelMatrix4f();
        boxMtx = boxTrans.getLocalModelMatrix4f();

        ballMtx.getTranslation(ballPos);
        boxMtx.getTranslation(boxPos);
        dist = boxPos.sub(ballPos).length();
        mWaiter.assertTrue(boxPos.y < -7);
        mWaiter.assertTrue(ballPos.y == 8);
        mWaiter.assertTrue(Math.abs(boxPos.x) < 1.3);
        mWaiter.assertTrue((dist < 19) && (dist > 18));

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

    private SXRNode addGround(float x, float y, float z)
    {
        try
        {
            SXRContext ctx = sxrTestUtils.getSxrContext();
            SXRTexture tex = ctx.getAssetLoader().loadTexture(new SXRAndroidResource(ctx, "floor.jpg"));
            SXRMaterial mtl = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
            SXRNode ground = new SXRCubeNode(ctx, true, mtl, new Vector3f(100, 10, 100));
            SXRMeshCollider collider = new SXRMeshCollider(ctx, true);
            SXRRigidBody body = new SXRRigidBody(ctx, 0.0f);

            mtl.setMainTexture(tex);
            ground.getTransform().setPosition(x, y, z);
            ground.setName("ground");
            ground.attachCollider(collider);
            body.setRestitution(0.5f);
            body.setFriction(1.0f);
            ground.attachComponent(body);
            return ground;
        }
        catch (IOException exception)
        {
           mWaiter.fail(exception);
           return null;
        }
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
