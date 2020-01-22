#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <utility>
#include <array>
#include <algorithm>

// (x, y) point
using point = std::pair<double, double>;

// A line segment represent by two point ends
using lineSeg = std::pair<point, point>;

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Definition of our robot.
struct Robot
{
    // Type = {c,s,p}.  Circle, square, or point robot
    char type;
    // The location of the robot in the environment
    double x, y;
    // The orientation of the square robot.  Undefined for point or circle robot
    double theta;
    // The length of a side of the square robot or the radius of the circle robot
    // Undefined for the point robot.
    double length;
};

// Intersect the point (x,y) with the set of rectangles. If the point lies
// outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Intersect a circle with center (x,y) and given radius with the set of
// rectangles. If the circle lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

// Custom debugging/development code. Takes the list of robots, the list of
// obstacles, and whether or not each configuation should be valid or not.
void debugMode(const std::vector<Robot> &robots, const std::vector<Rectangle> &obstacles,
               const std::vector<bool> &valid);

// Calculate the Euclidean norm of vector from (x1, y1) to (x2, y2).
// Helper function for isValidCircle
double vectorNorm(double x1, double y1, double x2, double y2);

// return a point(x, y) in workspace
// localX, localY: point coordinate in local coordinate frame
// wspX, wspY: rotation reference point in workspace
// theta: orientation of object
point pointRotate(double localX, double localY, double wspX, double wspY, double theta);

// return true if a square robot intersects a rectangle obstacle
bool isIntersect(const std::array<point, 4>& robot, const Rectangle& rect);

// return perimenter of a rectangle object
std::array<lineSeg, 4> perimeter(const point& ll, const point& lr, const point& ul, const point& ur);

// return true if two line segments intersect
bool isLineIntersect(const lineSeg&, const lineSeg&);

// return orientation of pqr, helper function of isLineIntersect
int orientation(const point& p, const point& q, const point& r);

// return true if one rectangle is inside another rectangle
// rbLL, rbLR, rbUL, rbUR: vertices of smaller one
// rectLL, rectLR, rectUL, rectUR: vertices of big one
bool isRectInRect(const point& rbLL, const point& rbLR, const point& rbUL, const point& rbUR, 
                const point& rectLL, const point& rectLR, const point& rectUL, const point& rectUR);

#endif
