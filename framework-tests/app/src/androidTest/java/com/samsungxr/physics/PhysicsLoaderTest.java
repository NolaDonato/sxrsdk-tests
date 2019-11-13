package com.samsungxr.physics;

import android.support.test.InstrumentationRegistry;
import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;
import android.content.res.AssetManager;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMaterial;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRScene;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.nodes.SXRCubeNode;
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
 * Instrumentation test, which will execute on an Android device.
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
@RunWith(AndroidJUnit4.class)
public class PhysicsLoaderTest
{
    private SXRTestUtils sxrTestUtils;
    private Waiter mWaiter;
    private SXRWorld mWorld;
    private AssetManager mAssetManager;
    private SXRPhysicsLoader mLoader;
    private boolean mDoCompare = true;

    @Rule
    public ActivityTestRule<SXRTestableActivity> ActivityRule = new
            ActivityTestRule<SXRTestableActivity>(SXRTestableActivity.class);

    @Before
    public void setUp() throws TimeoutException {
        mWaiter = new Waiter();
        sxrTestUtils = new SXRTestUtils(ActivityRule.getActivity());
        sxrTestUtils.waitForOnInit();
        mAssetManager = InstrumentationRegistry.getTargetContext().getAssets();
    }

    private SXRPhysicsLoader.IPhysicsLoaderEvents mLoadHandler = new SXRPhysicsLoader.IPhysicsLoaderEvents()
    {
        @Override
        public void onPhysicsLoaded(SXRPhysicsContent world, SXRSkeleton skel, String filename)
        {
            if (world != null)
            {
                SXRNode root = world.getOwnerObject();

                if (world != mWorld)
                {
                    world.getSXRContext().getMainScene().addNode(root);
                    mWorld.merge(world);
                }
                mWorld.enable();
                sxrTestUtils.onAssetLoaded(root);
            }
            else
            {
                mWorld.enable();
                sxrTestUtils.onAssetLoaded(null);
            }
        }

        @Override
        public void onLoadError(SXRPhysicsContent world, String filename, String errors)
        {
            if (errors.contains("open"))
            {
                mWaiter.fail(errors);
            }
            onPhysicsLoaded(world, null, filename);
        }
    };

    @Test
    public void loadBullet() throws TimeoutException, IOException
    {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRWorld world;
        SXRNode root = new SXRNode(context);
        SXRAndroidResource r;

        scene.getMainCameraRig().getCenterCamera().getTransform().setPosition(0, 3, 40);
        mWorld = new SXRWorld(sxrTestUtils.getMainScene());
        createFloor(context);
        mWorld.disable();

        SXRNode debugDraw = mWorld.setupDebugDraw();
        scene.addNode(debugDraw);
        mWorld.setDebugMode(-1);
        mLoader = new SXRPhysicsLoader(sxrTestUtils.getSxrContext(), mAssetManager);
        mLoader.setMultiBody(false);
        mLoader.getEventReceiver().addListener(mLoadHandler);
        root.setName("PhysicsRoot");
        world = new SXRWorld(root, false);
        root.attachComponent(world);
        r = new SXRAndroidResource(context, "scene3.bullet");
        mLoader.loadPhysics(world, r, false);
        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(5);
        world.disable();
        sxrTestUtils.screenShot(getClass().getSimpleName(), "loadBullet", mWaiter, mDoCompare);
    }

    @Test
    public void loadBulletMultiBody() throws TimeoutException
    {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();

        scene.getMainCameraRig().getCenterCamera().getTransform().setPosition(0, 0.5f, 1);
        mWorld = new SXRWorld(sxrTestUtils.getMainScene(), true);
        createFloor(context);
        mWorld.disable();

        SXRNode debugDraw = mWorld.setupDebugDraw();
        scene.addNode(debugDraw);
        mWorld.setDebugMode(-1);
        mLoader = new SXRPhysicsLoader(context, mAssetManager);
        mLoader.setMultiBody(true);
        mLoader.getEventReceiver().addListener(mLoadHandler);
        mLoader.loadPhysics(scene, "quadruped.bullet");
        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(5);
        sxrTestUtils.screenShot(getClass().getSimpleName(), "loadBulletMultiBody", mWaiter, mDoCompare);
    }

    @Test
    public void loadURDF() throws TimeoutException, IOException
    {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRAndroidResource r = new SXRAndroidResource(context, "quadruped.urdf");
        SXRWorld world;
        SXRNode root = new SXRNode(context);

        scene.getMainCameraRig().getCenterCamera().getTransform().setPosition(0, 0.5f, 1);
        root.setName("PhysicsRoot");
        world = new SXRWorld(root, false);
        root.attachComponent(world);

        mWorld = new SXRWorld(sxrTestUtils.getMainScene());
        createFloor(context);
        mWorld.disable();

        SXRNode debugDraw = mWorld.setupDebugDraw();
        scene.addNode(debugDraw);
        mWorld.setDebugMode(-1);

        mLoader = new SXRPhysicsLoader(sxrTestUtils.getSxrContext(), mAssetManager);
        mLoader.setMultiBody(false);
        mLoader.getEventReceiver().addListener(mLoadHandler);
        mLoader.loadPhysics(world, r, false);
        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(5);
        mWorld.disable();
        sxrTestUtils.screenShot(getClass().getSimpleName(), "loadURDF", mWaiter, mDoCompare);
    }

    @Test
    public void loadURDFMultiBody() throws TimeoutException {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();

        scene.getMainCameraRig().getCenterCamera().getTransform().setPosition(0, 0.5f, 1);
        mWorld = new SXRWorld(sxrTestUtils.getMainScene(), true);
        createFloor(context);
        mWorld.disable();

        SXRNode debugDraw = mWorld.setupDebugDraw();
        scene.addNode(debugDraw);
        mWorld.setDebugMode(-1);

        mLoader = new SXRPhysicsLoader(sxrTestUtils.getSxrContext(), mAssetManager);
        mLoader.setMultiBody(true);
        mLoader.getEventReceiver().addListener(mLoadHandler);
        mLoader.loadPhysics(scene, "quadruped.urdf");
        sxrTestUtils.waitForAssetLoad();
        sxrTestUtils.waitForXFrames(5);
        mWorld.disable();
        sxrTestUtils.screenShot(getClass().getSimpleName(), "loadURDFMultiBody", mWaiter, mDoCompare);
    }

    private SXRNode createFloor(SXRContext ctx)
    {
        SXRMaterial mtl = new SXRMaterial(ctx, SXRMaterial.SXRShaderType.Phong.ID);
        mtl.setDiffuseColor(0.7f, 0.4f, 0.3f, 1f);
        SXRCubeNode floor = new SXRCubeNode(ctx, true, new Vector3f(100, 10, 100));
        floor.getTransform().setPosition(0, -15, 0);
        floor.getRenderData().setMaterial(mtl);
        floor.attachComponent(new SXRBoxCollider(ctx));
        ctx.getMainScene().addNode(floor);
        SXRRigidBody floorRb = new SXRRigidBody(ctx, 0f);
        floor.attachComponent(floorRb);
        return floor;
    }
}
