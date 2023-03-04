#ifndef __UTILS_H__
#define __UTILS_H__

#include "stm32f10x.h"
#include "math.h"

#define SERVO_MIN_PWM	500
#define SERVO_MAX_PWM 2500
#define SERVO_MIN_DEG -90
#define SERVO_MAX_DEG 90

#define PI 3.14159265358979323846

long map(long x, long in_min, long in_max, long out_min, long out_max);
int deg_to_ccr(long deg);
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
int32_t constrain(int32_t in_val, int32_t min_val, int32_t max_val);
float constrainf(float in_val, float min_val, float max_val);
double bearing(double lat1, double lon1, double lat2, double lon2);
double getRelativeBearing(double currentHeading, double targetBearing);
double rad2circleDeg(double radians);
double unwrap(double phase);
double deg2rad(double degrees);
double rad2deg(double radians);
double angle_diff(double angle1, double angle2);

struct Point {
  double lat;
  double lon;
};

double haversineDistance(Point p1, Point p2);
Point nearestPointOnCircle(Point center, double radius, Point point);
Point getDestLatLon(Point point, double bearing, double distance);

struct GPS
	{
		double lat;
		double lon;
	};

struct PositionsAngles_s
{
	float throttle;
	float pitch;
	float roll;
	float yaw;
	float alt;
	float speed;
	float rollRate;
	float pitchRate;
	float yawRate;
	
	GPS gps;
};

#endif
