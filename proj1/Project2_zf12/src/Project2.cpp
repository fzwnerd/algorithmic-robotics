///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Zhenwei Feng 
//////////////////////////////////////

#include <fstream>
#include <iostream>
#include <cstring>

#include "CollisionChecking.h"

// Retrieve a representation of the environment
void getObstacles(std::vector<Rectangle> &obstacles)
{
    Rectangle rect;

    // middle
    rect.x = 0.5;
    rect.y = 0.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // upper middle
    rect.x = -1.0;
    rect.y = 2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // lower middle
    rect.x = -1.0;
    rect.y = -2.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);

    // left
    rect.x = -3.0;
    rect.y = -2.5;
    rect.height = 5.0;
    rect.width = 1.0;
    obstacles.push_back(rect);

    // right
    rect.x = 3.35;
    rect.y = -1.25;
    rect.height = 3.75;
    rect.width = 0.5;
    obstacles.push_back(rect);
}

bool readOptions(int argc, char **argv, std::vector<Robot> &robots, std::vector<bool> &valid, bool debug)
{
    if (argc < 2)
    {
        std::cout << "Project 2: Shapes on a Plane" << std::endl;
        std::cout << "  Usage: ./Project2 [-d] inputfile" << std::endl;
        std::cout << "    - Supply the -d flag to enable debug mode" << std::endl;
        std::cout << std::endl;
        std::cout << "The input file consists of a list of robot configurations to test" << std::endl;
        std::cout << "Robot configurations have the following format: [type] [valid] [configuration...]" << std::endl;
        std::cout << "  Where type={p,c,s} for the point, circle, or square robot" << std::endl;
        std::cout << "  Valid is a boolean that states whether the configuration is valid or not" << std::endl;
        std::cout << "  The remaining configuration entries are:  x y [theta] [radius/side length]" << std::endl;
        std::cout << "     theta is defined for square robot only." << std::endl;
        std::cout << "     radius/side length is defined only for circle and square robot, respectively" << std::endl;
        std::cout << "  Examples: " << std::endl;
        std::cout << "    Point robot at the origin (which is valid): p 1 0 0" << std::endl;
        std::cout << "    Circle robot with radius=0.2 at the origin (valid): c 1 0 0 0.2" << std::endl;
        std::cout << "    Square robot with side length=1.0 at the origin with orientation=pi/2 (invalid): s 0 0 0 "
                     "1.57079 1.0"
                  << std::endl;
        return false;
    }

    const char *filename = argv[1];
    if (argc == 3)  // check for debug
    {
        if (strcmp(argv[1], "-d") == 0)
        {
            debug = true;
            filename = argv[2];
        }
        else
            std::cout << "Unknown argument: " << argv[1] << ".  Ignoring." << std::endl;
    }

    // Opening filename
    std::ifstream fin;
    fin.open(filename);
    if (!fin)
    {
        std::cerr << "Fatal: Failed to open " << filename << std::endl;
        return false;
    }

    robots.clear();
    valid.clear();

    // Reading in all configs
    while (true)
    {
        // Read in the robot type
        Robot robot;
        fin >> robot.type;
        if (fin.eof())  // end of file
            break;

        // Something bad happened.  Read in something we did not expect.
        if (robot.type != 'p' && robot.type != 'c' && robot.type != 's')
        {
            std::cout << "Fatal: Malformed input file.  Unknown robot type " << robot.type << std::endl;
            return false;
        }

        // Reading in the rest of the configuration, some of these values depend on type
        bool v;
        fin >> v;
        valid.push_back(v);

        fin >> robot.x >> robot.y;

        if (robot.type == 'c')  // circle
            fin >> robot.length;

        else if (robot.type == 's')  // square
            fin >> robot.theta >> robot.length;

        robots.push_back(robot);
    }

    return true;
}

// Return true if the robot does not collide with any of the obstacles
bool isValid(const Robot &robot, const std::vector<Rectangle> &obstacles)
{
    switch (robot.type)
    {
        case 'p':
            return isValidPoint(robot.x, robot.y, obstacles);
        case 'c':
            return isValidCircle(robot.x, robot.y, robot.length, obstacles);
        case 's':
            return isValidSquare(robot.x, robot.y, robot.theta, robot.length, obstacles);
        default:
            std::cerr << "Invalid robot type specified!" << std::endl;
            throw;
    }
}

int main(int argc, char **argv)
{
    std::vector<Robot> robots;
    std::vector<bool> valid;
    bool debug = false;

    // Read everything from the command line
    if (readOptions(argc, argv, robots, valid, debug))
    {
        std::vector<Rectangle> obstacles;
        getObstacles(obstacles);

        // If debug mode, execute user code instead of collision checking
        if (debug)
            debugMode(robots, obstacles, valid);
        else
        {
            // The list of misclassified configurations
            std::vector<int> wrong;

            // Collision check everything
            for (size_t i = 0; i < robots.size(); ++i)
            {
                if (isValid(robots[i], obstacles) != valid[i])
                    wrong.push_back(i);
            }

            // Print statistics
            double pct = (double)(robots.size() - wrong.size()) / (double)robots.size();
            std::cout << robots.size() - wrong.size() << " / " << robots.size()
                      << " configurations classified correctly [" << pct * 100.0 << "%]" << std::endl;

            if (wrong.size())
            {
                std::cout << "The following configurations were classified incorrectly (starting at zero): "
                          << std::endl;
                for (size_t i = 0; i < wrong.size(); ++i)
                    std::cout << wrong[i] << " ";
                std::cout << std::endl;
            }
        }
    }
}
