package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMaterial;
import com.samsungxr.SXRMesh;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTexture;
import com.samsungxr.SXRTransform;
import com.samsungxr.nodes.SXRCubeNode;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;
import com.samsungxr.utility.Log;

import net.jodah.concurrentunit.Waiter;

import org.joml.AxisAngle4f;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

public class PhysicsJointConstraintTest
{
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
        world = new SXRWorld(sxrTestUtils.getMainScene(), true);
        sxrTestUtils.waitForXFrames(5);
    }

    @Test
    public void fixedConstraintMixed()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0.75f, -10);
        SXRPhysicsJoint body1 = addJoint(sxrTestUtils.getMainScene(), -2, 4, -10, 1.0f);
        SXRNode box1 = body1.getOwnerObject();
        SXRTransform trans1 = box1.getTransform();
        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), 2, 0.75f, -10, 1.0f);
        SXRRigidBody body2 = ((SXRRigidBody) box2.getComponent(SXRRigidBody.getComponentType()));
        SXRTransform trans2 = box2.getTransform();
        SXRFixedConstraint constraint = new SXRFixedConstraint(sxrTestUtils.getSxrContext(), body2);

        body2.setSimulationType(SXRRigidBody.DYNAMIC);
        box1.attachComponent(constraint);
        sxrTestUtils.waitForXFrames(10);

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(30);

        body1.applyTorque(100, 0, 0);
        listener.waitForXSteps(30);
        float d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(d) < 1);

        body2.applyCentralForce(100,0,100);
        listener.waitForXSteps(30);
        d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(d) < 1);
    }

    @Test
    public void fixedConstraintJoints()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -10);
        SXRPhysicsJoint body1 = addJoint(sxrTestUtils.getMainScene(), -2, 0.75f, -10, 1.0f);
        SXRNode box1 = body1.getOwnerObject();
        SXRTransform trans1 = box1.getTransform();
        SXRPhysicsJoint body2 = addJoint(sxrTestUtils.getMainScene(), 2, 0.75f, -10, 1.0f);
        SXRNode box2 = body2.getOwnerObject();
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
        float d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(d) < 1);

        body2.applyCentralForce(100, 0, 100);
        listener.waitForXSteps(30);
        d = transformsDistance(trans1, trans2);
        mWaiter.assertTrue(Math.abs(d) < 1);
    }

    @Test
    public void point2pointConstraintMixed()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        float pivotInA[] = { 0f, -1.5f, 0f };
        float pivotInB[] = { -8f, -1.5f, 0f };
        SXRNode ball = addSphere(sxrTestUtils.getMainScene(), 0.0f, 10.0f, -10.0f, 0.0f);
        SXRPhysicsJoint boxBody = addJoint(sxrTestUtils.getMainScene(), 8.0f, 10.0f, -10.0f, 1.0f);
        SXRNode box = boxBody.getOwnerObject();
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
    public void point2pointConstraintJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 3);
        world.getEventReceiver().addListener(listener);

        float pivotInA[] = { 0f, -1.5f, 0f };
        float pivotInB[] = { -8f, -1.5f, 0f };
        SXRPhysicsJoint joint0 = addJoint(sxrTestUtils.getMainScene(), 0.0f, 10.0f, -10.0f, 0.0f);
        SXRPhysicsJoint joint1 = addJoint(sxrTestUtils.getMainScene(), 8.0f, 10.0f, -10.0f, 1.0f);
        SXRNode box0 = joint0.getOwnerObject();
        SXRNode box1 = joint1.getOwnerObject();
        SXRPoint2PointConstraint constraint = new SXRPoint2PointConstraint(sxrTestUtils.getSxrContext(), joint0, pivotInA, pivotInB);

        box1.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(60);

        float distance = transformsDistance(box0.getTransform(), box1.getTransform());
        mWaiter.assertTrue(distance < 9.8);

        listener.waitForXSteps(60);
        distance = transformsDistance(box0.getTransform(), box1.getTransform());
        mWaiter.assertTrue(distance < 9.8);
        sxrTestUtils.waitForXFrames(30);
    }


    @Test
    public void sliderConstraintMixed()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -10f);
        SXRPhysicsJoint joint1 = addJoint(sxrTestUtils.getMainScene(), 3.0f, 0.75f, -10.0f, 1.0f);
        SXRNode box1 = joint1.getOwnerObject();
        SXRNode box2 = addCube(sxrTestUtils.getMainScene(), -2.0f, 0.75f, -10.0f, 1.0f);
        SXRRigidBody body2 = ((SXRRigidBody) box2.getComponent(SXRRigidBody.getComponentType()));
        SXRSliderConstraint constraint = new SXRSliderConstraint(sxrTestUtils.getSxrContext(), joint1,
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
        body2.setSimulationType(SXRRigidBody.DYNAMIC);
        box2.attachComponent(constraint);
        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(10);

        body2.applyCentralForce(200f, 0f, 0f);
        listener.waitForXSteps(60);
        Vector3f p = new Vector3f(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        Vector3f dir = new Vector3f(p.x - trans1.getPositionX(),
                                    p.y - trans1.getPositionY(),
                                    p.z - trans1.getPositionZ());
        dir.normalize();
        float dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.01f);

        body2.applyCentralForce(-200f, 0f, 0f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        p.sub(pos2);
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);

        joint1.applyCentralForce(200f, 0f, -100f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);

        body2.applyTorque(0, 100f, 0f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);
        sxrTestUtils.waitForXFrames(30);
    }

    @Test
    public void sliderConstraintJoint()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 4);
        world.getEventReceiver().addListener(listener);

        SXRNode ground = addGround(sxrTestUtils.getMainScene(), 0f, 0f, -10f);
        SXRPhysicsJoint joint1 = addJoint(sxrTestUtils.getMainScene(), 3.0f, 0.75f, -10.0f, 1.0f);
        SXRNode box1 = joint1.getOwnerObject();
        SXRPhysicsJoint joint2 = addJoint(sxrTestUtils.getMainScene(), -2.0f, 0.75f, -10.0f, 1.0f);
        SXRNode box2 = joint2.getOwnerObject();
        SXRSliderConstraint constraint = new SXRSliderConstraint(sxrTestUtils.getSxrContext(), joint1,
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
        listener.waitForXSteps(30);

        joint2.applyCentralForce(200f, 0f, 0f);
        listener.waitForXSteps(60);
        Vector3f p = new Vector3f(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        Vector3f dir = new Vector3f(p.x - trans1.getPositionX(),
                                    p.y - trans1.getPositionY(),
                                    p.z - trans1.getPositionZ());
        dir.normalize();
        float dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.01f);

        joint2.applyCentralForce(-200f, 0f, 0f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        p.sub(pos2);
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);

        joint1.applyCentralForce(200f, 0f, -100f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);

        joint2.applyTorque(0, 100f, 0f);
        listener.waitForXSteps(60);
        p.set(trans2.getPositionX(), trans2.getPositionY(), trans2.getPositionZ());
        dir = new Vector3f(p.x - trans1.getPositionX(),
                           p.y - trans1.getPositionY(),
                           p.z - trans1.getPositionZ());
        dir.normalize();
        dot = dir.dot(sliderAxis);
        mWaiter.assertTrue((1 - Math.abs(dot)) < 0.03f);
        sxrTestUtils.waitForXFrames(30);
    }

//    @Test
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
        body2.setSimulationType(SXRRigidBody.DYNAMIC);
        body1.setSimulationType(SXRRigidBody.DYNAMIC);
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

    private SXRPhysicsJoint addJoint(SXRScene scene, float x, float y, float z, float mass)
    {
        SXRMaterial mtl1 = new SXRMaterial(sxrTestUtils.getSxrContext(), SXRMaterial.SXRShaderType.Phong.ID);
        SXRNode rootCube = new SXRCubeNode(sxrTestUtils.getSxrContext(), true, mtl1);

        rootCube.getTransform().setPosition(x, y, z);
        rootCube.setName("joint0");
        mtl1.setDiffuseColor(1, 0.5f, 1, 1);
        scene.addNode(rootCube);

        // Colliders
        SXRBoxCollider collider0 = new SXRBoxCollider(sxrTestUtils.getSxrContext());

        collider0.setHalfExtents(0.5f, 0.5f, 0.5f);
        rootCube.attachCollider(collider0);

        // Physics joint
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), mass, 1);

        rootCube.attachComponent(rootJoint);
        return rootJoint;
    }

    private SXRPhysicsJoint addJoints(SXRScene scene, float x, float y, float z, float mass)
    {
        SXRMaterial mtl1 = new SXRMaterial(sxrTestUtils.getSxrContext(), SXRMaterial.SXRShaderType.Phong.ID);
        SXRMaterial mtl2 = new SXRMaterial(sxrTestUtils.getSxrContext(), SXRMaterial.SXRShaderType.Phong.ID);
        SXRNode rootCube = new SXRCubeNode(sxrTestUtils.getSxrContext(), true, mtl1);
        SXRNode childCube = new SXRCubeNode(sxrTestUtils.getSxrContext(), true, mtl2);

        rootCube.getTransform().setPosition(x, y, z);
        rootCube.setName("joint0");
        childCube.getTransform().setPosition(0, -2, 0);
        mtl1.setDiffuseColor(1, 0.5f, 1, 1);
        mtl2.setDiffuseColor(0.5f, 1, 1,1);
        rootCube.addChildObject(childCube);
        scene.addNode(rootCube);

        // Colliders
        SXRBoxCollider collider0 = new SXRBoxCollider(sxrTestUtils.getSxrContext());
        SXRBoxCollider collider1 = new SXRBoxCollider(sxrTestUtils.getSxrContext());

        collider0.setHalfExtents(0.5f, 0.5f, 0.5f);
        rootCube.attachCollider(collider0);
        collider1.setHalfExtents(0.5f, 0.5f, 0.5f);
        childCube.attachCollider(collider1);

        // Physics joint
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(sxrTestUtils.getSxrContext(), mass, 2);
        SXRPhysicsJoint childJoint = new SXRPhysicsJoint(rootJoint, SXRPhysicsJoint.SPHERICAL, 1, mass);

        rootCube.attachComponent(rootJoint);
        childCube.attachComponent(childJoint);
        return rootJoint;
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
