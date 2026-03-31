#include "frc_robot_jni_FunJNI.h"
#include "jni.h"
#include "driver.h"
#include <vector>

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
  // Check to ensure the JNI version is valid

  JNIEnv *env;
  if (vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) != JNI_OK)
    return JNI_ERR;

  // In here is also where you store things like class references
  // if they are ever needed

  return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM *vm, void *reserved) {}

JNIEXPORT jint JNICALL Java_frc_robot_jni_FunJNI_initialize(JNIEnv *, jclass) {
  return 0;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_jni_FunJNI_solveSwerve(JNIEnv* env, jclass, jdoubleArray current_vel, jdoubleArray desired_vel) {
  int current_n = env->GetArrayLength(current_vel);
  int desired_n = env->GetArrayLength(desired_vel);
  if (current_n != 3 || desired_n != 3) {
    jclass exClass = env->FindClass("java/lang/IllegalArgumentException");
    env->ThrowNew(exClass, "Arrays must be length 3");
    return nullptr;
  }

  std::vector<double> current_vel_vec(current_n);
  std::vector<double> desired_vel_vec(desired_n);

  jdouble* current = env->GetDoubleArrayElements(current_vel, nullptr);
  jdouble* desired = env->GetDoubleArrayElements(desired_vel, nullptr);

  for(int i = 0; i < current_n; i++) {
    current_vel_vec[i] = current[i];
  }

  for(int i = 0; i < desired_n; i++) {
    desired_vel_vec[i] = desired[i];
  }

  env->ReleaseDoubleArrayElements(current_vel, current, JNI_ABORT);
  env->ReleaseDoubleArrayElements(desired_vel, desired, JNI_ABORT);

  auto out = solveSwerve(current_vel_vec, desired_vel_vec);

  jdoubleArray result = env->NewDoubleArray(out.size());
  env->SetDoubleArrayRegion(result, 0, out.size(), out.data());

  return result;
}