#include <jni.h>

extern "C" {

JNIEXPORT jlongArray JNICALL
Java_com_samsungxr_tester_StateSortTests_getRenderDataVector(JNIEnv *env, jclass type) {
    jlongArray result = env->NewLongArray(1);

    return result;
}

}
