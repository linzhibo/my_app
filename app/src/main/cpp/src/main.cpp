//
// Created by zhibo on 18-4-16.
//

#include <jni.h>
#include <string>
#include <gmapping/sensor/sensor_base/sensor.h>
#include "gslam.h"

bool isWhitespace(std::string s){
    for(int index = 0; index < s.length(); index++){
        if(!std::isspace(s[index]))
            return false;
    }
    return true;
}


using namespace GMapping;
extern "C" JNIEXPORT jstring

JNICALL
Java_com_orbbec_zhibo_gmapping_1java_1cpp_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {


    Sensor* sensor=new Sensor("Astra_mini");
    std::string sensorName = sensor->getName();

    GSlam gslam;
    gslam.startLiveSlam();

    return env->NewStringUTF(sensorName.c_str());
}

