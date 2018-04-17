//
// Created by zhibo on 18-4-16.
//

#include <jni.h>
#include <string>
#include <gmapping/sensor/sensor_base/sensor.h>
#include "slam_gmapping.h"
using namespace GMapping;
extern "C" JNIEXPORT jstring

JNICALL
Java_com_orbbec_zhibo_gmapping_1java_1cpp_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    Sensor* sensor=new Sensor("Astra");
    std::string sensorName = sensor->getName();

    return env->NewStringUTF(sensorName.c_str());
}

