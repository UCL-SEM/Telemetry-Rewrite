#include <Arduino.h>
#include <math.h>
#include <vector>

class Geofence {
public:
    Geofence();
    void init(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD);
    bool isInside(float lat, float lng);

private:
    double haversine(float r, double lat1, double lng1, double lat2, double lng2);
    float latA, lngA, latB, lngB, latC, lngC, latD, lngD;
    float disAB, disBC, disCD, disDA;
    float r; // Earth's radius
};
/*
class DataProcessor {
public:
    void addGeofence(Geofence geofence);
    void processGeofences(float lat, float lng);

private:
    std::vector<Geofence> geofences;
};
*/