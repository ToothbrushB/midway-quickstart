
#include "PurePursuit.hpp"
#include "CatmullRom.h"
#include <cmath>
#include <esp_timer.h>
#include <esp_log.h>

#define PI 3.14159265358979

const char* TAG = "PurePursuit";
// Constructor.
PurePursuit::PurePursuit(double LOOK_AHEAD, unsigned long INTERVAL)
{
    _LOOK_AHEAD = LOOK_AHEAD;
	_INTERVAL = INTERVAL;
	_interpolationStep = DEFAULT_INTERPOLATION_STEP;
	_stop = false;
}

double PurePursuit::getDistance(float x0, float y0, float x1, float y1)
{
	return sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0));
}

// This is called internally from PurePursuit to start the path following.
void PurePursuit::start()
{
	_stop = false;
	(*_path).resetIterator(_interpolationStep);
}

// Check that the robot should be stopped.
// _stop is set to false after path following is complete.
// _stop is false by default, and set to true when starting path following.
bool PurePursuit::checkStop()
{
	return _stop;
}

// Compute the next goal point. Find the point on the path that is closest to the look ahead distance.
void PurePursuit::computeNextGoalPoint(double xPos, double yPos)
{
	CatmullRom& path = *_path;

	float dist = INFINITY;
	float prevDist = 0;
	float minDist = INFINITY;
	float minIterX = 0;
	float minIterY = 0;

	// TODO: No need to use minimum since break condition is prevDist < dist.
	// Simplify code.
	while (true)
	{
		double iterX = path.getIterationX();
		double iterY = path.getIterationY();

		prevDist = dist;
		dist = fabs(getDistance(xPos, yPos, iterX, iterY) - _LOOK_AHEAD);
		if (dist < minDist)
		{
			minDist = dist;
			minIterX = iterX;
			minIterY = iterY;
		}

		// Serial.print(iterX); Serial.print("\t");
		// Serial.print(iterY); Serial.print("\t");
		// Serial.print(prevDist); Serial.print("\t");
		// Serial.println(dist);

		if (prevDist < dist || !path.hasNext()) // break when we reach the end of the path or the distance starts increasing
		{
			break;
		}
		else
		{
			path.next();
		}
	}
	_goalX = minIterX;
	_goalY = minIterY;

	// Ensures that the stop condition can be met.
	// Stop condition involves path.hasNext();
	if (path.hasNext())
	{
		path.prev();
	}
}

// Returns the curvature computed by the pure pursuit algorithm. 
double PurePursuit::getCurvature()
{
	return _curvature;
}

// This is called frequently during path following.
// It computes the next goal point and the curvature.
void PurePursuit::compute(double xPos, double yPos, double heading)
{
	
	if (esp_timer_get_time() - _prevComputeTime > _INTERVAL)
	{
		CatmullRom& path = (*_path);

		// 1. Get the current location of the vehicle.

		double phi = heading + PI / 2.0;
		// 2, 3. Find the goal point.
		computeNextGoalPoint(xPos, yPos);


		// 4. Transform the goal point to vehicle coordinates.
		// First translate it such that the origin is the robot.
		float xGoalTranslated = _goalX - xPos;
		float yGoalTranslated = _goalY - yPos;
		// Second rotate the coordinate system such that the wheel axle points to the x-axis.
		double xGoalRelRobot = xGoalTranslated * cos(phi) + yGoalTranslated * sin(phi);

		// 5. Calculate the curvature.
		_curvature = 2 * xGoalRelRobot / _LOOK_AHEAD / _LOOK_AHEAD;

		// double radius = 1 / curvature;

		// 6. Update the vehicle's position.
		// Vehicle's position is updated using dead reckoning at set interval.

		// printf("%f,%f,%f,%f,%f,%f,%f,%llu\n", xPos, yPos, heading, _goalX, _goalY, xGoalRelRobot, _curvature, esp_timer_get_time());
		// Print debug info
		// Serial.print("c: "); Serial.print(_curvature);
		// Serial.print(", xPos: "); Serial.print(xPos);
		// Serial.print(", yPos: "); Serial.print(yPos);
		// Serial.print(", xGoal: "); Serial.print(_goalX);
		// Serial.print(", yGoal: "); Serial.println(_goalY);

		// Serial.print(_goalX); Serial.print("\t");
		// Serial.println(_goalY);

		// 7. Check for stop condition
		if (!path.hasNext())
		{
			// Check for stoping condition
			int length = path.getLength();
			float p0x = path.getX(length - 2);
			float p0y = path.getY(length - 2);
			float p1x = path.getX(length - 1);
			float p1y = path.getY(length - 1);
			float p2x = xPos;
			float p2y = yPos;
			float phi1 = atan2(p1y - p0y, p1x - p0x);
			float phi2 = atan2(p2y - p1y, p2x - p1x);
			float theta = fabs(phi2 - phi1);
			if (theta < PI/2 || theta > 3*PI/2)
			{
				// Serial.println("STOP TRUE");
				// Serial.print(p0x); Serial.print("\t");
				// Serial.print(p0y); Serial.print("\t");
				// Serial.print(p1x); Serial.print("\t");
				// Serial.print(p1y); Serial.print("\t");
				// Serial.print(p2x); Serial.print("\t");
				// Serial.print(p2y); Serial.print("\t");
				// Serial.print(phi1); Serial.print("\t");
				// Serial.print(phi2); Serial.print("\t");
				// Serial.println(theta);
				_stop = true;
			}
		}

		_prevComputeTime = esp_timer_get_time();
	}
}

// Set the path for the path follower to follow. 
void PurePursuit::setPath(float* pathX, float* pathY, int pathLength)
{
	if (pathLength < MIN_PATH_LENGTH)
	{
		ESP_LOGI(TAG, "ERROR: Path length must be minimum of %d", MIN_PATH_LENGTH);
		return;
	}
	if (_path == nullptr)
	{
		_path = new CatmullRom();
	}
	(*_path).setPoints(pathX, pathY, pathLength);
}

// Set the path for the path follower to follow.
// The path is encapsulated within the CatmullRom class.
// See setPath(float* pathX, float* pathY, int pathLength) as well
void PurePursuit::setPath(CatmullRom* path)
{
	delete _path;
	_path = path;
}

// Return the path. The path is encapsulated within the CatmullRom class.
CatmullRom* PurePursuit::getPath()
{
	return _path;
}

// Return the pointer to the x path array.
float* PurePursuit::getArrayX()
{
	return (*_path).getArrayX();
}

// Return the pointer to the y path array.
float* PurePursuit::getArrayY()
{
	return (*_path).getArrayY();
}

// Return the current goal point x value.
float PurePursuit::getGoalX()
{
	return (float)_goalX;
}

// Return the current goal point y value.
float PurePursuit::getGoalY()
{
	return (float)_goalY;
}

// Set the target interpolation step used. 
// Note that CatmullRom class ecapsulates the path. 
// The camullrom interpolation is done on the path to make path following smoother with less data points.
void PurePursuit::setInterpolationStep(float interpolationStep)
{
	_interpolationStep = interpolationStep;
}

// Print the path neatly. 
// void PurePursuit::printPath()
// {
// 	(*_path).printPath();
// }


/*
void PreMo::startPathFollowing(float* pathX, float* pathY, int pathLength, bool isForward, bool setLocation)
{
	// Set starting position to first path location point with robot pointing towards the next point.
	double x0 = pathX[0];
	double y0 = pathY[0];
	double x1 = pathX[1];
	double y1 = pathY[1];

	double heading = atan2(y1 - y0, x1 - x0);

	// Serial.print("x0: "); Serial.print(x0);
	// Serial.print("\ty0: "); Serial.print(y0);
	// Serial.print("\tx1: "); Serial.print(x1);
	// Serial.print("\ty1: "); Serial.print(y1);
	// Serial.print("\theading: "); Serial.println(heading);

	// Set the location to the initial coordinates.
	DeadReckoner& dr = (*_deadReckoner);
	// Set the heading to the angle of the vector from the first to the second point.
	heading = isForward ? heading : heading + PI;

	if (setLocation)
	{
		// Set the starting location as the first path point.
		dr.setX(x0);
		dr.setY(y0);

		// Set the heading such that the robot points to the initial direction that the path will follow.
		dr.setHeading(heading);
	}

	(*_purePursuit).setPath(pathX, pathY, pathLength);
	// (*_purePursuit).printPath();

	_moveReverse = !isForward;
	_isFollowingPath = true;
	(*_purePursuit).start();
	
	// TODO Remove
	// double x = dr.getX();
	// double y = dr.getY();
	// double t = dr.getHeading();
	// Serial.println("DEAD RECKONER SET");
	// Serial.print("x: "); Serial.print(x);
	// Serial.print(", y: "); Serial.print(y);
	// Serial.print(", t: "); Serial.println(t);
}

void PreMo::continuePathFollowing()
{
	// Pure pursuit
	(*_purePursuit).compute();

	// Compute PID
	_input = (*_purePursuit).getCurvature();
	(*_pid).Compute();

	// Move the robot
	// Serial.print("output: "); Serial.println(_output);
	moveMotors(_motorSpeed, _output);

	if ((*_purePursuit).checkStop())
	{
		_isFollowingPath = false;
		stop();
	}
}
*/