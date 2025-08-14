// Code modified from https://github.com/purwar2016/PreciseMovement-library

#include "CatmullRom.h"

#pragma once


class PurePursuit
{
public:
    static constexpr int MIN_PATH_LENGTH = 4;
    static constexpr float DEFAULT_INTERPOLATION_STEP = 20; // mm
    PurePursuit(double LOOK_AHEAD);
    void start();
    bool checkStop();
    void compute(double xPos, double yPos, double heading);
    void setPath(float* pathX, float* pathY, int pathLength);
    void setPath(CatmullRom* path);
    CatmullRom* getPath();
    float* getArrayX();
    float* getArrayY();
    float getGoalX();
    float getGoalY();
    void setInterpolationStep(float interpolationStep);
    void computeNextGoalPoint(double xPos, double yPos);
    double getCurvature();
    double static getDistance(float x0, float y0, float x1, float y1);
    // void printPath();

private:
    // The camullrom interpolation is done on the path to make path following smoother with less data points.
    // Target interpolation step for catmull interpolation.
    float _interpolationStep;

    //  The path is encapsulated within the CatmullRom class.
    CatmullRom* _path = nullptr;

    // _stop is set to false after path following is complete.
    // _stop is false by default, and set to true when starting path following.
    bool _stop;


    // Look ahead pure pursuit algoirthm parameter.
    double _LOOK_AHEAD;

    // Curvature computed by the pure pursuit algorithm.
    double _curvature;

    // Current goal point x value.
    double _goalX;

    // Current goal point y value.
    double _goalY;
};
