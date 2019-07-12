package com.samsungxr.physics;

import android.support.test.rule.ActivityTestRule;
import android.support.test.runner.AndroidJUnit4;

import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRMaterial;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.nodes.SXRCubeNode;
import com.samsungxr.nodes.SXRSphereNode;
import com.samsungxr.unittestutils.SXRTestUtils;
import com.samsungxr.unittestutils.SXRTestableActivity;

import net.jodah.concurrentunit.Waiter;

import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.util.concurrent.TimeoutException;

/**
 * Instrumentation test, which will execute on an Android device.
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
        mWorld = new SXRWorld(sxrTestUtils.getMainScene());
    }

    @Test
    public void testTwoJoints()
    {
        SXRContext context = sxrTestUtils.getSxrContext();
        SXRScene scene = sxrTestUtils.getMainScene();
        SXRPhysicsJoint rootJoint = new SXRPhysicsJoint(context,  10.0f, 1);
        SXRPhysicsJoint firstJoint = new SXRPhysicsJoint(rootJoint, 0, 2.0f);
        SXRMaterial blue = new SXRMaterial(context, SXRMaterial.SXRShaderType.Phong.ID);
        SXRMaterial red = new SXRMaterial(context, SXRMaterial.SXRShaderType.Phong.ID);
        SXRCubeNode box = new SXRCubeNode(context, true, blue);
        SXRSphereNode sphere = new SXRSphereNode(context, true, red);
        SXRSphereCollider collider1 = new SXRSphereCollider(context);
        SXRBoxCollider collider2 = new SXRBoxCollider(context);

        box.getTransform().setPositionY(-3);
        sphere.getTransform().setPositionY(3);
        box.addChildObject(sphere);
        scene.addNode(box);

        box.attachCollider(collider2);
        sphere.attachCollider(collider1);
        box.attachComponent(rootJoint);
        sphere.attachComponent(firstJoint);
        mWorld.setEnable(true);
    }
}
