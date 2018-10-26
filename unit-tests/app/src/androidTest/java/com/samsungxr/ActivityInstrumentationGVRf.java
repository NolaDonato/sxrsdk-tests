package com.samsungxr;

import android.app.Activity;
import android.content.Intent;
import android.test.ActivityInstrumentationTestCase2;

import junit.framework.Test;
import junit.framework.TestSuite;

//import org.junit.runner.RunWith;
//import org.junit.runners.JUnit4;
//import org.junit.BeforeClass;
//import org.junit.runner.RunWith;
//import org.junit.runners.JUnit4;

//@RunWith(JUnit4.class)

public class ActivityInstrumentationSXRf extends
        ActivityInstrumentationTestCase2<DefaultSXRTestActivity> {

    public static Activity mActivity = null;

    public ActivityInstrumentationSXRf() {
        super(DefaultSXRTestActivity.class);
    }

    public ActivityInstrumentationSXRf(Class activityClass) {
        super(activityClass);
    }


    @Override
    public synchronized void setUp() throws Exception {

        if (!DefaultSXRTestActivity.sContextLoaded) {
            if (mActivity == null) {
                mActivity = getActivity();
            }
            synchronized (DefaultSXRTestActivity.class) {
                try {
                    DefaultSXRTestActivity.class.wait();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public synchronized void tearDown() throws Exception {
        //Intent intent = mActivity.getIntent();
        //mActivity.finish();
        //mActivity.startActivity(intent);
        //DefaultSXRTestActivity.sContextLoaded=false;
        // Scrub out members - protects against memory leaks in the case where someone
        // creates a non-static inner class (thus referencing the test case) and gives it to
        // someone else to hold onto
        scrubClass(ActivityInstrumentationTestCase2.class);
        System.gc();
        //super.tearDown();
     }

    @Override
    protected synchronized void finalize() throws Throwable {
        DefaultSXRTestActivity.getInstance().store(this);
        super.finalize();
    }
}
