///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Zhenwei Feng 
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for (const auto& rect : obstacles)
    {
        if (x >= rect.x && x <= rect.x + rect.width && y >= rect.y && y <= rect.y + rect.height)
            return false;
    }
    return true;
}

double vectorNorm(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for (const auto& rect : obstacles)
    {
        if (vectorNorm(x, y, rect.x, rect.y) <= radius ||
            vectorNorm(x, y, rect.x + rect.width, rect.y) <= radius ||
            vectorNorm(x, y, rect.x, rect.y + rect.height) <= radius ||
            vectorNorm(x, y, rect.x + rect.width, rect.y + rect.height) <= radius ||
            (x >= rect.x - radius && x <= rect.x + rect.width + radius && y >= rect.y && y <= rect.y +  rect.height) ||
            (x >= rect.x && x <= rect.x + rect.width && y >= rect.y - radius && y <= rect.y + rect.height + radius))
            return false;
    }
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    double halfSide = sideLength / 2.0;
    point wspLowerLeft = pointRotate(-halfSide, -halfSide, x, y, theta);
    
    point wspLowerRight = pointRotate(halfSide, -halfSide, x, y, theta);
    point wspUpperLeft = pointRotate(-halfSide, halfSide, x, y, theta);
    point wspUpperRight = pointRotate(halfSide, halfSide, x, y, theta);
    //std::cout << wspLowerLeft.first << " " << wspLowerLeft.second << std::endl
    //        << wspUpperLeft.first << " " << wspUpperLeft.second << std::endl
    //        << wspLowerRight.first << " " << wspLowerRight.second << std::endl
    //        << wspUpperRight.first << " " << wspUpperRight.second << std::endl;
    std::array<point, 4> robot {wspLowerLeft, wspLowerRight, wspUpperLeft, wspUpperRight};
    for (const auto& rect : obstacles)
        if (isIntersect(robot, rect))
            return false;
    //std::cout << "valid\n";
    return true;
}

// Return a point(x, y) rotated by theta in workspace
point pointRotate(double localX, double localY, double wspX, double wspY, double theta)
{
    point p;
    double cosTh = std::cos(theta);
    double sinTh = std::sin(theta);
    //p.first = localX * cosTh - localY * sinTh + wspX * (1 - cosTh) + wspY * sinTh;
    //p.second = localX * sinTh + localY * cosTh + wspY * (1 - cosTh) - wspX * sinTh;
    p.first = localX * cosTh - localY * sinTh + wspX;
    p.second = localX * sinTh + localY * cosTh + wspY;
    return p;
}

// Check if a square robot intersects a rectangular obstacle
bool isIntersect(const std::array<point, 4>& robot, const Rectangle& rect)
{
    point rbLL = robot[0];
    point rbLR = robot[1];
    point rbUL = robot[2];
    point rbUR = robot[3];
    point rectLL{rect.x, rect.y};
    point rectLR{rect.x + rect.width, rect.y};
    point rectUL{rect.x, rect.y + rect.height};
    point rectUR{rect.x + rect.width, rect.y + rect.height};
    std::array<lineSeg, 4> rbPerim = perimeter(rbLL, rbLR, rbUL, rbUR);
    std::array<lineSeg, 4> rectPerim = perimeter(rectLL, rectLR, rectUL, rectUR);
    //std::cout << isLineIntersect(lineSeg{point{2.5,0}, point{2.5,1}}, lineSeg{point{2.49, 0.23}, point{2.61784, -0.15}}) << std::endl;
    if (isRectInRect(rbLL, rbLR, rbUL, rbUR, rectLL, rectLR, rectUL, rectUR) ||
        isRectInRect(rectLL, rectLR, rectUL, rectUR, rbLL, rbLR, rbUL, rbUR))
        return true;
    for (const auto& rbLine : rbPerim)
        for (const auto& rectLine: rectPerim)           
            if (isLineIntersect(rbLine, rectLine))
                return true;
    return false;
}

// Return perimeter of a rectangle object
std::array<lineSeg, 4> perimeter(const point& ll, const point& lr, const point& ul, const point& ur)
{
    return std::array<lineSeg, 4>{lineSeg{ll, lr}, lineSeg{ll, ul}, lineSeg{lr, ur}, lineSeg{ul, ur}};
}

// Check if two line segments intersect
bool isLineIntersect(const lineSeg& ls1, const lineSeg& ls2)
{
    const point& p1 = ls1.first;
    const point& q1 = ls1.second;
    const point& p2 = ls2.first;
    const point& q2 = ls2.second;
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    //std::cout << o1 << o2 << o3 << o4 << std::endl;
    if (o1 != o2 && o3 != o4)
        return true;
    return false;
}

// Get the orientation of pqr
int orientation(const point& p, const point& q, const point& r)
{
    //std::cout << p.first << " " << p.second << std::endl;
    //std::cout << q.first << " " << q.second <<std::endl;
    //std::cout << r.first << " " << r.second << std::endl;
    double val = (q.second - p.second) * (r.first - q.first) - 
              (q.first - p.first) * (r.second - q.second);
    //std::cout << val << std::endl << std::endl;
    if (val == 0) return 0;
    return (val > 0) ? 1 : 2;
}

// Check if one rectangle lies in another rectangle
bool isRectInRect(const point& rbLL, const point& rbLR, const point& rbUL, const point& rbUR, 
                const point& rectLL, const point& rectLR, const point& rectUL, const point& rectUR)
{
    std::vector<double> rectX{rectLL.first, rectLR.first, rectUL.first, rectUR.first};
    std::vector<double> rectY{rectLL.second, rectLR.second, rectUL.second, rectUR.second};
    double minX = *std::min_element(rectX.begin(), rectX.end());
    double maxX = *std::max_element(rectX.begin(), rectX.end());
    double minY = *std::min_element(rectY.begin(), rectY.end());
    double maxY = *std::max_element(rectY.begin(), rectY.end());
    return rbLL.first >= minX && rbLL.first <= maxX && rbLL.second >= minY && rbLL.second <= maxY &&
            rbLR.first >= minX && rbLR.first <= maxX && rbLR.second >= minY && rbLR.second <= maxY &&
            rbUL.first >= minX && rbUL.first <= maxX && rbUL.second >= minY && rbUL.second <= maxY &&
            rbUR.first >= minX && rbUR.first <= maxX && rbUR.second >= minY && rbUR.second <= maxY;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/)
{
}
