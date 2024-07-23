#ifndef _LINEAR_CONVERSION_
#define _LINEAR_CONVERSION_

float findSlope(float x1, float y1, float x2, float y2) {
    float slope = (y2 - y1) / (x2 - x1);
    return slope;
}

float findYInstercept(float slope, float x1, float y1) {
    float yIntercept = y1 - (slope * x1);
    return yIntercept;
}

// Third input value is the current value that you're trying to map. Example: an encoder's current value.
float findLinearConversion(float slope, float yIntercept, float valueToMap) {
    float yOut = (slope * valueToMap) + yIntercept;
    return yOut;
}

// Brian's map function that works with floats
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    float newValue;

    newValue = value * ((outMax-outMin)/(inMax-inMin)) + outMin;
    return newValue;
}

#endif