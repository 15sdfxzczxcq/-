// motion.cpp
#include "motion.hpp"
#include <fstream>
#include <iostream>
#include <cmath>

Motion::Motion() {
    string jsonPath = "../src/config/config.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good()) {
        std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
        exit(-1);
    }

    nlohmann::json js_value;
    config_is >> js_value;

    try {
        params = js_value.get<Params>();
    } catch (const nlohmann::detail::exception& e) {
        std::cerr << "Json Params Parse failed :" << e.what() << '\n';
        exit(-1);
    }

    speed = params.speedLow;
   
    // 其他初始化输出...
}

float Motion::lowPassFilter(float input, float* prevOutput, float alpha) {
    return alpha * (*prevOutput) + (1 - alpha) * input;
}

void Motion::poseCtrl(int controlCenter) {
    servoPwm = (uint16_t)(controlCenter);
}

void Motion::speedCtrl(bool enable, bool slowDown, ControlCenter control) {
    uint8_t controlLow = 0;
    uint8_t controlMid = 5;
    uint8_t controlHigh = 10;

    if (slowDown) {
        countShift = controlLow;
        speed = params.speedDown;
    } 
    else if (enable) {
        if (control.centerEdge.size() < 10) {
            speed = params.speedLow;
            countShift = controlLow;
            return;
        }
        
        if (abs(control.sigmaCenter) < 45.0) {
            countShift++;
            if (countShift > controlHigh) countShift = controlHigh;
        } 
        else if (abs(control.sigmaCenter) > 2000.0) {
            speed = params.speedLow;
            return;
        }
        else {
            countShift--;
            if (countShift < controlLow) countShift = controlLow;
        }

        speed = (countShift > controlMid) ? params.speedHigh : params.speedLow;
    } 
    else {
        countShift = controlLow;
        speed = params.speedLow;
    }
}