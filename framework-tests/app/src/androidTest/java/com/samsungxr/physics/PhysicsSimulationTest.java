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
    private static final String TAG = "PHYSICS";
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    SXRWorld world;
    SXRScene scene;
    SXRContext context;
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
        context = sxrTestUtils.getSxrContext();
        scene = sxrTestUtils.getMainScene();
        scene.getMainCameraRig().getTransform().setPosition(0.0f, 6.0f, 0.0f);
        world = new SXRWorld(scene);
    }

    @Test
    public void updatedOwnerTransformTest()
    {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 10);
        world.getEventReceiver().addListener(listener);
        addGroundMesh(scene, 0.0f,0.0f,0.0f, 0.0f);

        SXRNode[] objects = new SXRNode[10];
        SXRRigidBody[] bodies = new SXRRigidBody[10];

        for(int i = 0; i < 10; i = i + 2) {
            SXRBoxCollider boxCollider = new SXRBoxCollider(context);
            boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
            objects[i] = meshWithTexture("cube.obj", "cube.jpg");
            objects[i].getTransform().setPosition(0.0f -i, 6.0f, -10.0f - (2.0f*i));
            objects[i].attachCollider(boxCollider);
            switch (i) {
                case 0 : bodies[i] = new SXRRigidBody(context); //object moves, rigid body doesn't
                         bodies[i].setSimulationType(SXRRigidBody.STATIC);
                         break;
                case 2 : bodies[i] = new SXRRigidBody(context, 1.0f);  //rigidbody is a "trigger  collider"
                         bodies[i].setSimulationType(SXRRigidBody.STATIC);
                         break;
                case 4 : bodies[i] = new SXRRigidBody(context);  //object moves, rigid body doesn't
                         bodies[i].setSimulationType(SXRRigidBody.KINEMATIC);
                         break;
                case 6 : bodies[i] = new SXRRigidBody(context, 1.0f); //rigid body is "sleeping", until it is hit by another
                         bodies[i].setSimulationType(SXRRigidBody.KINEMATIC);
                         break;
                case 8 : bodies[i] = new SXRRigidBody(context, 1.0f); //rigid body is obbeys all external forces
                         bodies[i].setSimulationType(SXRRigidBody.DYNAMIC);
                         break;
            }
            scene.addNode(objects[i]);
            objects[i].attachComponent(bodies[i]);

            SXRSphereCollider sphereCollider = new SXRSphereCollider(context);
            sphereCollider.setRadius(0.5f);
            objects[i+1] = meshWithTexture("sphere.obj", "sphere.jpg");
            scene.addNode(objects[i+1]);
            objects[i+1].getTransform().setScale(0.5f,0.5f,0.5f);
            objects[i+1].getTransform().setPosition(0.0f -i, 9.0f, -10.0f - (2.0f*i));
            objects[i+1].attachCollider(sphereCollider);
            bodies[i+1] = new SXRRigidBody(context, 10.0f);
            objects[i+1].attachComponent(bodies[i+1]);
        }

        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(47);
        for(int i = 0; i < 5; i++) {
            objects[i*2].getTransform().setPositionX(2.0f);
        }

        listener.waitForXSteps(600);

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
        listener.waitForXSteps(60);
    }

    @Test
    public void freeFallTest()  {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        world.getEventReceiver().addListener(listener);
        CollisionHandler mCollisionHandler = new CollisionHandler();
        mCollisionHandler.extimatedTime = 2800; //... + round up
        mCollisionHandler.collisionCounter = 0;

        SXRNode cube = addCube(scene, 0.0f, 1.0f, -10.0f, 0.0f);
        SXRNode sphere = addSphere(scene, mCollisionHandler, 0.0f, 40.0f, -10.0f, 1.0f);

        mCollisionHandler.startTime = System.currentTimeMillis();
        listener.waitUntilAdded();
        world.setEnable(true);
        listener.waitForXSteps(168);
        float d = (sphere.getTransform().getPositionY() - cube.getTransform().getPositionY()); //sphere is on top of the cubeROP
        mWaiter.assertTrue( d <= 1.6f);
        Log.d("PHYSICS", "    Number of collisions:" + mCollisionHandler.collisionCounter);
        mWaiter.assertTrue(mCollisionHandler.collisionCounter >= 1);
    }

    @Test
    public void spacedFreeFallTest() {
        CollisionTest collisionTest = new CollisionTest(100);
        collisionTest.mCollisionHandler.startTime = System.currentTimeMillis();
        collisionTest.runTest(168);
        //sxrTestUtils.getMainScene().getRoot().detachComponent(SXRWorld.getComponentType());
        Log.d("PHYSICS", "    Number of collisions: " + collisionTest.mCollisionHandler.collisionCounter);
        mWaiter.assertTrue(collisionTest.mCollisionHandler.collisionCounter >= collisionTest.length);
    }

    @Test
    public void simultaneousFreeFallTest() {
        CollisionTestAll collisionTest = new CollisionTestAll(100);

        collisionTest.mCollisionHandler.startTime = System.currentTimeMillis();
        collisionTest.runTest(168);
        //sxrTestUtils.getMainScene().getRoot().detachComponent(SXRWorld.getComponentType());
        Log.d("PHYSICS", "    Number of collisions: " + collisionTest.mCollisionHandler.collisionCounter);
        mWaiter.assertTrue(collisionTest.mCollisionHandler.collisionCounter >= collisionTest.length);
    }

    @Test
    public void applyingForcesTest()  {
        PhysicsEventHandler listener = new PhysicsEventHandler(sxrTestUtils, 2);
        world.getEventReceiver().addListener(listener);

        float distance;
        float pitch;
        float deltaZ;
        float deltaPitch;
        addGroundMesh(scene, 0.0f,0.0f,0.0f, 0.0f);
        SXRNode cube = addCube(scene, 0.0f, 0.6f, -5.0f, 0.01f);
        SXRRigidBody body = (SXRRigidBody) cube.getComponent(SXRRigidBody.getComponentType());
        listener.waitUntilAdded();
        world.setEnable(true);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyCentralForce(0,0,-1);
        listener.waitForXSteps(60);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertTrue(deltaPitch < 1);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyForce(0,0,-1, 0.0f, 0.5f, 0.0f);
        listener.waitForXSteps(120);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertFalse(deltaPitch < 1);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyCentralImpulse(0,0,-0.02f);
        listener.waitForXSteps(120);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertTrue(deltaPitch < 1);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyImpulse(0,0, -0.02f,
                0.0f, 0.5f, 0.0f);
        listener.waitForXSteps(120);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertFalse(deltaPitch < 1);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyTorque(-1, 0, 0);
        listener.waitForXSteps(120);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertFalse(deltaPitch < 1);

        distance = cube.getTransform().getPositionZ();
        pitch = cube.getTransform().getRotationPitch();
        body.applyTorqueImpulse(-0.1f, 0, 0);
        listener.waitForXSteps(60);
        deltaZ = Math.abs(distance - cube.getTransform().getPositionZ());
        deltaPitch = Math.abs(pitch - cube.getTransform().getRotationPitch());
        mWaiter.assertTrue(deltaZ > 0.0001f);
        mWaiter.assertFalse(deltaPitch < 1);
    }

    class CollisionTest
    {
        protected int length;
        protected SXRNode cube[];
        protected SXRNode sphere[];
        protected CollisionHandler mCollisionHandler;
        protected PhysicsEventHandler mEventHandler;
        protected SXRScene mScene;

        CollisionTest(int length)
        {
            cube = new SXRNode[length];
            sphere = new SXRNode[length];
            mEventHandler = new PhysicsEventHandler(sxrTestUtils, 2 * length);
            world.getEventReceiver().addListener(mEventHandler);
            mCollisionHandler = new CollisionHandler();
            mCollisionHandler.extimatedTime = 2800; //... + round up
            mCollisionHandler.collisionCounter = 0;
            mScene = sxrTestUtils.getMainScene();
            this.length = length;
            addObjects();
            mEventHandler.waitUntilAdded();
        }

        public void addObjects()
        {
            int x = -100;
            for (int i = 0; i < length; i++)
            {
                x += 2;
                cube[i] = addCube(mScene, (float) x, 1.0f, -10.0f, 0.0f);
                sphere[i] = addSphere(mScene, mCollisionHandler, (float) x, 40.0f, -10.0f, 1.0f);
            }
        }

        public void runTest(int nsteps)
        {
            world.setEnable(true);
            mEventHandler.waitForXSteps(nsteps);
            for (int i = 0; i < length; i++)
            {
                float cubeX = cube[i].getTransform().getPositionX();
                float cubeY = cube[i].getTransform().getPositionY();
                float cubeZ = cube[i].getTransform().getPositionZ();
                float sphereX = sphere[i].getTransform().getPositionX();
                float sphereY = sphere[i].getTransform().getPositionY();
                float sphereZ = sphere[i].getTransform().getPositionZ();

                Log.d("PHYSICS", "[" + i + "] cube: " + cubeX + ", " + cubeY + ", " + cubeZ +
                        " sphere: " + sphereX + ", " + sphereY + ", " + sphereZ);

                float d = (sphereY - cubeY);
                //sphere is on top of the cube

                mWaiter.assertTrue( d <= 1.6f);
                mWaiter.assertTrue(Math.abs(sphereX - cubeX) < 0.05f);
                mWaiter.assertTrue(Math.abs(sphereZ - cubeZ) < 0.05f);
                //Log.d("PHYSICS", "    Index:" + i + "    Collision distance:" + d);
            }
            mEventHandler.waitForXSteps(30);
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
                cube[i] = addCube(mScene, (float) x, 1.0f, (float) z, 0.0f);
                sphere[i] = addSphere(mScene, mCollisionHandler,
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
            object = new SXRNode(context,
                    new SXRAndroidResource(context, mesh),
                    new SXRAndroidResource(context, texture));
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
                sphereMesh = context.getAssetLoader().loadMesh(
                        new SXRAndroidResource(context, "sphere.obj"));
                sphereTexture = context.getAssetLoader().loadTexture(
                        new SXRAndroidResource(context, "sphere.jpg"));
            } catch (IOException e) {

            }
        }

        SXRNode sphereObject = new SXRNode(context, sphereMesh, sphereTexture);

        sphereObject.getTransform().setScaleX(0.5f);
        sphereObject.getTransform().setScaleY(0.5f);
        sphereObject.getTransform().setScaleZ(0.5f);
        sphereObject.getTransform().setPosition(x, y, z);
        sphereObject.setName("sphere");
        scene.addNode(sphereObject);

        // Collider
        SXRSphereCollider sphereCollider = new SXRSphereCollider(context);
        sphereCollider.setRadius(0.5f);
        sphereObject.attachCollider(sphereCollider);

        // Physics body
        SXRRigidBody mSphereRigidBody = new SXRRigidBody(context, mass);

        sphereObject.getEventReceiver().addListener(mCollisionHandler);

        sphereObject.attachComponent(mSphereRigidBody);

        return sphereObject;
    }

    private SXRNode addCube(SXRScene scene, float x, float y, float z, float mass) {

        if (cubeMesh == null) {
            try {
                cubeMesh = context.getAssetLoader().loadMesh(
                        new SXRAndroidResource(context, "cube.obj"));
                cubeTexture = context.getAssetLoader().loadTexture(
                        new SXRAndroidResource(context, "cube.jpg"));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        SXRNode cubeObject = new SXRNode(context, cubeMesh, cubeTexture);

        cubeObject.getTransform().setPosition(x, y, z);
        cubeObject.setName("cube");
        scene.addNode(cubeObject);

        // Collider
        SXRBoxCollider boxCollider = new SXRBoxCollider(context);
        boxCollider.setHalfExtents(0.5f, 0.5f, 0.5f);
        cubeObject.attachCollider(boxCollider);

        // Physics body
        SXRRigidBody body = new SXRRigidBody(context, mass);

        cubeObject.attachComponent(body);

        return cubeObject;
    }

    private void addGroundMesh(SXRScene scene, float x, float y, float z, float mass) {
        try {
            SXRMesh mesh = context.createQuad(100.0f, 100.0f);
            SXRTexture texture =
                    context.getAssetLoader().loadTexture(new SXRAndroidResource(context, "floor.jpg"));
            SXRNode meshObject = new SXRNode(context, mesh, texture);

            meshObject.getTransform().setPosition(x, y, z);
            meshObject.getTransform().setRotationByAxis(-90.0f, 1.0f, 0.0f, 0.0f);
            meshObject.setName("ground");
            scene.addNode(meshObject);

            // Collider
            SXRMeshCollider meshCollider = new SXRMeshCollider(context, mesh);
            meshObject.attachCollider(meshCollider);

            // Physics body
            SXRRigidBody body = new SXRRigidBody(context);

            body.setRestitution(0.5f);
            body.setFriction(1.0f);

            meshObject.attachComponent(body);

        } catch (IOException exception) {
            Log.d("sxrf", exception.toString());
        }
    }
}
