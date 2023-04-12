#include "utils.h"
#include "math.h"

const double EARTH_RADIUS = 6371;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

int deg2ccr(long deg)
{
	return map(deg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_PWM, SERVO_MAX_PWM);
}

/**
constrain - getting input value and keeping it between min and max values
*/
int32_t constrain(int32_t in_val, int32_t min_val, int32_t max_val)
{
	if (in_val > max_val)
		return max_val;
	else if (in_val < min_val)
		return min_val;
	else
		return in_val;
}

float constrainf(float in_val, float min_val, float max_val)
{
	if (in_val > max_val)
		return max_val;
	else if (in_val < min_val)
		return min_val;
	else
		return in_val;
}

double bearing(double lat1, double lon1, double lat2, double lon2) // return bearing to target -pi to pi
{
	// Calculate the heading to the center of the loiter
	double dLon = lon2 - lon1;
	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) -
						 sin(lat1) * cos(lat2) * cos(dLon);
	double bearing = atan2(y, x);

	return bearing;
}

/* inputs:
currentHeading - -pi to pi
targetBearing- -pi to pi
output:
double -pi to pi
*/
double getRelativeBearing(double currentHeading, double targetBearing)
{
	double relativeBearing = targetBearing - currentHeading;
  return relativeBearing;
}

double rad2circleDeg(double radians) {
	double deg = fmod(radians * 180.0 / PI, 360.0);
	if(deg < 0)
			deg += 360.0;
  return deg;
}


double haversineDistance(Point p1, Point p2) {
  double lat1 = p1.lat * PI / 180;
  double lat2 = p2.lat * PI / 180;
  double lon1 = p1.lon * PI / 180;
  double lon2 = p2.lon * PI / 180;
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS * 1000 * c;
}

Point nearestPointOnCircle(Point center, double radius, Point point) {
  Point result;
  double distance = haversineDistance(center, point);

  if (distance == radius) {
    result = point;
  } else {
    double a = radius / distance;
    double x = (1 - a) * center.lat + a * point.lat;
    double y = (1 - a) * center.lon + a * point.lon;
    result.lat = x;
    result.lon = y;
  }
  return result;
}

Point getDestLatLon(Point point, double bearing, double distance)
{
	Point result;
	unsigned int R = EARTH_RADIUS * 1000;
	double lat = point.lat * (PI/180);
	double lon = point.lon * (PI/180);
	bearing = bearing * (PI/180);
	result.lat = asin(sin(lat)*cos(distance/R) + cos(lat)*sin(distance/R)*cos(bearing));
	result.lon = lon + atan2(sin(bearing) * sin(distance/R) * cos(lat), cos(distance/R) - sin(lat) * sin(result.lat));
	result.lat = result.lat * (180/PI);
	result.lon = result.lon * (180/PI);
	return result;
}

double unwrap(double angle) {
    static double prev_angle = 0.0;
    const double two_pi = 2.0 * PI;

    double diff = angle - prev_angle;
    diff = fmod(diff + PI, two_pi) - PI;
    prev_angle += diff;

    return prev_angle;
}

double deg2rad(double degrees) {
    return degrees * PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / PI;
}
