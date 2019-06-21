package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;

import net.jodah.concurrentunit.Waiter;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMesh;
import com.samsungxr.SXRMeshCollider;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTexture;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;
import com.samsungxr.utility.Log;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

// kinematic equation
// d = 0.5*g(m/s^2)*t(s)^2
// (d/(0.5*g))^0.5 = t
// Xframes = t * 60
// tested on note4, no other apps running
// time to fall 2800ms or 168 frames, rounded up

@RunWith(AndroidJUnit4.class)
public class PhysicsSimulationTest {
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
        world = new SXRWorld(ctx);
        ctx.getMainScene().getRoot().attachComponent(world);
    }

    @Test
    public void updatedOwnerTransformTest() throws Exception {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 10);
        world.getEventReceiver().addListener(listener);
        addGroundMesh(sxrTestUtils.getMainScene(), 0.0f,0.0f,0.0f, 0.0f);

        SXRNode[] objects = new SXRNode[10];
        SXRRigidBody[] bodies = new SXRRigidBody[10];

        for(int i = 0; i < 10; i = i + 2) {
            SXRBoxCollider boxCollider = new SXRBoxCollider(sxrTestUtils.getSxrContext());
            boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
            objects[i] = meshWithTexture("cube.obj", "cube.jpg");
            objects[i].getTransform().setPosition(0.0f -i, 6.0f, -10.0f - (2.0f*i));
            objects[i].attachCollider(boxCollider);
            switch (i) {
                case 0 : bodies[i] = new SXRRigidBody(sxrTestUtils.getSxrContext()); //object moves, rigid body doesn't
                         bodies[i].setSimulationType(SXRRigidBody.STATIC);
                         break;
                case 2 : bodies[i] = new SXRRigidBody(sxrTestUtils.getSxrContext(), 1.0f);  //rigidbody is a "trigger  collider"
                         bodies[i].setSimulationType(SXRRigidBody.STATIC);
                         break;
                case 4 : bodies[i] = new SXRRigidBody(sxrTestUtils.getSxrContext());  //object moves, rigid body doesn't
                         bodies[i].setSimulationType(SXRRigidBody.KINEMATIC);
                         break;
                case 6 : bodies[i] = new SXRRigidBody(sxrTestUtils.getSxrContext(), 1.0f); //rigid body is "sleeping", until it is hit by another
                         bodies[i].setSimulationType(SXRRigidBody.KINEMATIC);
                         break;
                case 8 : bodies[i] = new SXRRigidBody(sxrTestUtils.getSxrContext(), 1.0f); //rigid body is obbeys all external forces
                         bodies[i].setSimulationType(SXRRigidBody.DYNAMIC);
                         break;
            }
            objects[i].attachComponent(bodies[i]);
            sxrTestUtils.getMainScene().addNode(objects[i]);

            SXRSphereCollider sphereCollider = new SXRSphereCollider(sxrTestUtils.getSxrContext());
            sphereCollider.setRadius(0.5f);
            objects[i+1] = meshWithTexture("sphere.obj", "sphere.jpg");
            objects[i+1].getTransform().setScale(0.5f,0.5f,0.5f);
            objects[i+1].getTransform().setPosition(0.0f -i, 9.0f, -10.0f - (2.0f*i));
            objects[i+1].attachCollider(sphereCollider);
            bodies[i+1] = new SXRRigidBody(sxrTestUtils.getSxrContext(), 10.0f);
            objects[i+1].attachComponent(bodies[i+1]);
            sxrTestUtils.getMainScene().addNode(objects[i+1]);

        }

        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(47);
        for(int i = 0; i < 5; i++) {
            objects[i*2].getTransform().setPositionX(2.0f);
        }

        sxrTestUtils.waitForXFrames(600);

        float d = (bodies[1].getTransform().getPositionY()
                - bodies[0].getTransform().getPositionY()); //sphere is on top of the cube rigid body
        mWaiter.assertTrue(d > 0.5f);

        d = (bodies[3].getTransform().getPositionY()
                - bodies[2].getTransform().getPositionY()); //sphere went thru the cube
        mWaiter.assertTrue(d <= 0.5f);

        d = (bodies[5].getTransform().getPositionY()
                - bodies[4].getTransform().getPositionY()); //sphere is on top of the cube rigid body
        mWaiter.assertTrue(d > 0.5f);

        d = (bodies[7].getTransform().getPositionY()
                - bodies[6].getTransform().getPositionY()); //sphere fell of the cube
        mWaiter.assertTrue(d <= 0.5f);

        d = (bodies[9].getTransform().getPositionY()
                - bodies[8].getTransform().getPositionY()); //sphere fell of the cube
        mWaiter.assertTrue(d <= 0.5f);

        for(int i = 0; i < 5; i++) {
            mWaiter.assertTrue(objects[i*2].getTransform().getPositionX() == bodies[i*2].getTransform().getPositionX());
        }


        sxrTestUtils.waitForXFrames(60);
    }

    @Test
    public void freeFallTest() throws Exception {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        world.getEventReceiver().addListener(listener);
        CollisionHandler mCollisionHandler = new CollisionHandler();
        mCollisionHandler.extimatedTime = 2800; //... + round up
        mCollisionHandler.collisionCounter = 0;

        SXRNode cube = addCube(sxrTestUtils.getMainScene(), 0.0f, 1.0f, -10.0f, 0.0f);
        SXRNode sphere = addSphere(sxrTestUtils.getMainScene(), mCollisionHandler, 0.0f, 40.0f, -10.0f, 1.0f);

        mCollisionHandler.startTime = System.currentTimeMillis();
        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(168);
        float d = (sphere.getTransform().getPositionY() - cube.getTransform().getPositionY()); //sphere is on top of the cube
        mWaiter.assertTrue( d <= 1.6f);
        //Log.d("PHYSICS", "    Number of collisions:" + mCollisionHandler.collisionCounter);
        mWaiter.assertTrue(mCollisionHandler.collisionCounter >= 1);
    }

    @Test
    public void spacedFreeFallTest() throws Exception {
        CollisionTest collisionTest = new CollisionTest(100);
        collisionTest.mCollisionHandler.startTime = System.currentTimeMillis();
        sxrTestUtils.waitForXFrames(168);
        collisionTest.runTest();
        mWaiter.assertTrue(collisionTest.mCollisionHandler.collisionCounter >= collisionTest.length);
    }

    @Test
    public void simultaneousFreeFallTest() throws Exception {
        CollisionTestAll collisionTest = new CollisionTestAll(100);

        collisionTest.mCollisionHandler.startTime = System.currentTimeMillis();
        sxrTestUtils.waitForXFrames(168);
        collisionTest.runTest();
        mWaiter.assertTrue(collisionTest.mCollisionHandler.collisionCounter >= collisionTest.length);
    }

    @Test
    public void applyingForcesTest() throws Exception {
        float distance;
        float rotation;
        addGroundMesh(sxrTestUtils.getMainScene(), 0.0f,0.0f,0.0f, 0.0f);
        SXRNode cube = addCube(sxrTestUtils.getMainScene(), 0.0f, 0.6f, -5.0f, 0.01f);
        sxrTestUtils.waitForSceneRendering();
        sxrTestUtils.waitForXFrames(10);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyCentralForce(0,0,-1);
        sxrTestUtils.waitForXFrames(60);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertTrue(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyForce(0,0,-1,
                0.0f, 0.5f, 0.0f);
        sxrTestUtils.waitForXFrames(120);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertFalse(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyCentralImpulse(0,0,-0.02f);
        sxrTestUtils.waitForXFrames(120);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertTrue(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyImpulse(0,0, -0.02f,
                0.0f, 0.5f, 0.0f);
        sxrTestUtils.waitForXFrames(120);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertFalse(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyTorque(-1, 0, 0);
        sxrTestUtils.waitForXFrames(120);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertFalse(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);

        distance = cube.getTransform().getPositionZ();
        rotation = cube.getTransform().getRotationPitch();
        ((SXRRigidBody)cube.getComponent(
                SXRRigidBody.getComponentType())).applyTorqueImpulse(-0.1f, 0, 0);
        sxrTestUtils.waitForXFrames(60);
        mWaiter.assertFalse(distance - cube.getTransform().getPositionZ() == 0);
        mWaiter.assertFalse(Math.abs(rotation - cube.getTransform().getRotationPitch()) < 1);
    }

    class CollisionTest
    {
        protected int length;
        protected SXRNode cube[];
        protected SXRNode sphere[];
        protected CollisionHandler mCollisionHandler;
        protected PhysicsEventHandler mEventHandler;

        CollisionTest(int length)
        {
            cube = new SXRNode[length];
            sphere = new SXRNode[length];
            world.setEnable(false);
            mEventHandler = new PhysicsEventHandler(sxrTestUtils, length);
            world.getEventReceiver().addListener(mEventHandler);
            mCollisionHandler = new CollisionHandler();
            mCollisionHandler.extimatedTime = 2800; //... + round up
            mCollisionHandler.collisionCounter = 0;
            this.length = length;
            addObjects();
            sxrTestUtils.waitForAssetLoad();
            world.setEnable(true);
        }

        public void addObjects()
        {
            int x = -100;
            for (int i = 0; i < length; i++)
            {
                x += 2;
                cube[i] = addCube(sxrTestUtils.getMainScene(), (float) x, 1.0f, -10.0f, 0.0f);
                sphere[i] = addSphere(sxrTestUtils.getMainScene(), mCollisionHandler,
                                      (float) x, 40.0f, -10.0f, 1.0f);
            }
        }

        public void runTest() throws  Exception
        {
            for (int i = 0; i < length; i++)
            {
                float cubeX = cube[i].getTransform().getPositionX();
                float cubeY = cube[i].getTransform().getPositionY();
                float cubeZ = cube[i].getTransform().getPositionZ();
                float sphereX = sphere[i].getTransform().getPositionX();
                float sphereY = sphere[i].getTransform().getPositionY();
                float sphereZ = sphere[i].getTransform().getPositionZ();

                //Log.d("runTest", "[" + i + "] cube: " + cubeX + ", " + cubeY + ", " + cubeZ +
                //        " sphere: " + sphereX + ", " + sphereY + ", " + sphereZ);

                float d = (sphereY - cubeY);
                //sphere is on top of the cube

                mWaiter.assertTrue( d <= 1.6f);
                mWaiter.assertTrue(sphereX == cubeX);
                mWaiter.assertTrue(sphereZ == cubeZ);
                //Log.d("PHYSICS", "    Index:" + i + "    Collision distance:" + d);
            }
        }
    }

    class CollisionTestAll extends CollisionTest
    {
        CollisionTestAll(int length)
        {
            super(length);
        }

        public void addObjects()
        {
            int x = -25;
            int j = 0;
            int l = 10;
            int z = -10;
            for (int i = 0; i < length; i++)
            {
                int step = 50 / l;
                x += step;
                cube[i] = addCube(sxrTestUtils.getMainScene(), (float) x, 1.0f, (float) z, 0.0f);
                sphere[i] = addSphere(sxrTestUtils.getMainScene(), mCollisionHandler,
                                      (float) x, 40.0f, (float) z, 1.0f);
                j++;
                if (j == l)
                {
                    x = -25;
                    z -= 10;
                    j = 0;
                }
            }
        }

    }


    public class CollisionHandler implements ICollisionEvents {
        public long startTime;
        public long extimatedTime;
        public long lastCollisionTime;
        public int collisionCounter;

        public void onEnter(SXRNode sceneObj0, SXRNode sceneObj1, float normal[], float distance) {
            lastCollisionTime = System.currentTimeMillis() - startTime;
            collisionCounter++;
        }

        public void onExit(SXRNode sceneObj0, SXRNode sceneObj1, float normal[], float distance) {
        }

    }

    private SXRNode meshWithTexture(String mesh, String texture) {
        SXRNode object = null;
        try {
            object = new SXRNode(sxrTestUtils.getSxrContext(), new SXRAndroidResource(
                    sxrTestUtils.getSxrContext(), mesh), new SXRAndroidResource(sxrTestUtils.getSxrContext(),
                    texture));
        } catch (IOException e) {
            mWaiter.fail(e);
        }
        return object;
    }

    /*
     * Function to add a sphere of dimension and position specified in the
     * Bullet physics world and scene graph
     */
    private SXRNode addSphere(SXRScene scene, ICollisionEvents mCollisionHandler, float x, float y, float z, float mass) {

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

        // Collider
        SXRSphereCollider sphereCollider = new SXRSphereCollider(sxrTestUtils.getSxrContext());
        sphereCollider.setRadius(0.5f);
        sphereObject.attachCollider(sphereCollider);

        // Physics body
        SXRRigidBody mSphereRigidBody = new SXRRigidBody(sxrTestUtils.getSxrContext(), mass);

        sphereObject.getEventReceiver().addListener(mCollisionHandler);

        sphereObject.attachComponent(mSphereRigidBody);

        scene.addNode(sphereObject);
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
                e.printStackTrace();
            }
        }

        SXRNode cubeObject = new SXRNode(sxrTestUtils.getSxrContext(), cubeMesh, cubeTexture);

        cubeObject.getTransform().setPosition(x, y, z);

        // Collider
        SXRBoxCollider boxCollider = new SXRBoxCollider(sxrTestUtils.getSxrContext());
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        cubeObject.attachCollider(boxCollider);

        // Physics body
        SXRRigidBody body = new SXRRigidBody(sxrTestUtils.getSxrContext(), mass);

        cubeObject.attachComponent(body);

        scene.addNode(cubeObject);
        return cubeObject;
    }

    private void addGroundMesh(SXRScene scene, float x, float y, float z, float mass) {
        try {
            SXRMesh mesh = sxrTestUtils.getSxrContext().createQuad(100.0f, 100.0f);
            SXRTexture texture =
                    sxrTestUtils.getSxrContext().getAssetLoader().loadTexture(new SXRAndroidResource(sxrTestUtils.getSxrContext(), "floor.jpg"));
            SXRNode meshObject = new SXRNode(sxrTestUtils.getSxrContext(), mesh, texture);

            meshObject.getTransform().setPosition(x, y, z);
            meshObject.getTransform().setRotationByAxis(-90.0f, 1.0f, 0.0f, 0.0f);

            // Collider
            SXRMeshCollider meshCollider = new SXRMeshCollider(sxrTestUtils.getSxrContext(), mesh);
            meshObject.attachCollider(meshCollider);

            // Physics body
            SXRRigidBody body = new SXRRigidBody(sxrTestUtils.getSxrContext());

            body.setRestitution(0.5f);
            body.setFriction(1.0f);

            meshObject.attachComponent(body);

            scene.addNode(meshObject);
        } catch (IOException exception) {
            Log.d("sxrf", exception.toString());
        }
    }
}
