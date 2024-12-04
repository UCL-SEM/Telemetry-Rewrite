#include "lib/DataProcessing.h"

Geofence::Geofence() {}

/**
 * Initializes a geofence with the given coordinates.
 *
 * @param latA The latitude of the first corner of the geofence.
 * @param lngA The longitude of the first corner of the geofence.
 * @param latB The latitude of the second corner of the geofence.
 * @param lngB The longitude of the second corner of the geofence.
 * @param latC The latitude of the third corner of the geofence.
 * @param lngC The longitude of the third corner of the geofence.
 * @param latD The latitude of the fourth corner of the geofence.
 * @param lngD The longitude of the fourth corner of the geofence.
 *
 * @return None
 *
 * @throws None
 */
void Geofence::init(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD) {
  this->latA = latA;
  this->lngA = lngA;
  this->latB = latB;
  this->lngB = lngB;
  this->latC = latC;
  this->lngC = lngC;
  this->latD = latD;
  this->lngD = lngD;
  this->r = 6378000;
  // Calculate distances between corners
  disAB = haversine(r, latA, lngA, latB, lngB);
  disBC = haversine(r, latB, lngB, latC, lngC);
  disCD = haversine(r, latC, lngC, latD, lngD);
  disDA = haversine(r, latD, lngD, latA, lngA);
}

/**
 * Checks if a given point is inside the geofence.
 *
 * This function uses the haversine formula to calculate distances from the point to each corner of the geofence.
 * It then uses these distances to determine if the point is inside the geofence.
 *
 * @param lat The latitude of the point to check.
 * @param lng The longitude of the point to check.
 *
 * @return True if the point is inside the geofence, false otherwise.
 *
 * @throws None
 */
bool Geofence::isInside(float lat, float lng) {
  // Implement the logic to check if the point is inside the geofence
  // This involves calculating distances from the point to each corner and using the haversine formula
  float disPA = haversine(r, lat, lng, latA, lngA);
  float disPB = haversine(r, lat, lng, latB, lngB);
  float disPC = haversine(r, lat, lng, latC, lngC);
  float disPD = haversine(r, lat, lng, latD, lngD);

  float alpha = acos((disDA * disDA - disPA * disPA - disPD * disPD) / (-2 * disPA * disPD));
  float beta = acos((disCD * disCD - disPC * disPC - disPD * disPD) / (-2 * disPC * disPD));
  float gamma = acos((disBC * disBC - disPB * disPB - disPC * disPC) / (-2 * disPB * disPC));
  float delta = acos((disAB * disAB - disPA * disPA - disPB * disPB) / (-2 * disPA * disPB));

  if (2 * M_PI - abs(alpha + beta + gamma + delta) <= 0.001) {
    return true;
  } else {
    return false;
  }
}

/**
 * Calculates the distance between two points on a sphere using the haversine formula.
 *
 * @param r The radius of the sphere.
 * @param lat1 The latitude of the first point in radians.
 * @param lng1 The longitude of the first point in radians.
 * @param lat2 The latitude of the second point in radians.
 * @param lng2 The longitude of the second point in radians.
 *
 * @return The distance between the two points.
 *
 * @throws None
 */
double Geofence::haversine(float r, double lat1, double lng1, double lat2, double lng2) {
  // Implement the haversine formula
  return 2 * r * asin(sqrt((1 - cos(lat2 - lat1) + cos(lat1) * cos(lat2) * (1 - cos(lng2 - lng1))) / 2));
}
/*
void DataProcessor::addGeofence(Geofence geofence) {
    geofences.push_back(geofence);
}

void DataProcessor::processGeofences(float lat, float lng) {
    for (auto& geofence : geofences) {
        if (geofence.isInside(lat, lng)) {
            // Trigger the geofence action
        }
    }
}
*/