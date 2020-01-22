COMP/ELEC/MECH 450/550
Project 2

What are we doing?
------------------

This project can be completed by modifying only CollisionChecking.cpp; this is
where the collision checking routines are located. You may freely add code to
CollisionChecking.h/.cpp, so long as the function prototypes for isValidPoint,
isValidCircle, isValidSquare, and debugMode remain unchanged. Your
implementation must be self-contained. In other words, you may NOT invoke other
programs or link against other libraries to perform any of the project
exercises; the implementation must be your own. Contact the TAs if you feel that
modifying Project2.cpp or any of the function prototypes is warranted.

All code for this project should be placed under the `src` directory.


How do I run the code?
----------------------

A (minimal) Makefile is provided. Depending on your system, you may have to
adjust parts of the Makefile. Once compiled, a binary named Project2 is
generated that accepts two arguments: an (optional) debugging flag and
(required) a file with a set of robot configurations to check whether the robot
is in collision with the environment or not. Project2 will then verify your
collision checkers against the known configurations. Any discrepancies found are
printed to the console for further inspection by you.

Verify that you have a working template by running
    ./Project2 configs/points_random.txt

You should see the following output:
    214 / 1000 configurations classified correctly [21.4%]
    The following configurations were classified incorrectly (starting at zero):
    0 1 2...

The debug flag is optional. When -d is specified, Project2 will execute the
'debugMode' function of CollisionChecking.cpp instead of verifying your
implementation. Any debugging code you submit will not be executed or graded,
but must compile.


How do I check my code?
-----------------------

The accuracy of your implementation will be assessed. To help, we provide a set
of test configurations and their collision result. These sets assume that the
environment hard-coded in Project2.cpp remains unchanged. Your implementation
will be assessed on the following configuration sets:
    - points_random.txt
    - circles_random.txt
    - circles_motion.txt
    - squares_random.txt
    - squares_motion.txt
    - squares_rotation.txt

The set 'squares_hard.txt' contains additional configurations that stretch the
assumptions made on the robot and environment geometries. It is not required
(but recommended) that your implementation covers these cases as well. The super
special case where line segments overlap (collide along a continuum of points)
does NOT have to be supported by your implementation and does not appear in the
configuration sets.


What is the 'known configuration file'?
---------------------------------------

The known configuration file is just a set of configurations for a point,
circle, or a square robot. In this file, robot configurations have the following
format:
    [type] [valid] [configuration...]

Where type={p,c,s} indicates a point, circle, or square robot, respectively.
Valid is a Boolean {0,1} that states whether the configuration is valid or not
The remaining configuration entries are;
    x y [theta] [radius/side length]
        - Theta is defined for square robot only.
        - radius/side length is defined only for circle and square robot, respectively

The validity of the configuration assumes the local origin for the circle robot
is at its center, and that the square robot is axis-aligned in its local
coordinate frame with the origin at the center.

Examples:

Point robot at the origin (valid configuration): p 1 0 0

Circle robot with radius=0.2 at the origin (valid configuration): c 1 0 0 0.2

Square robot with side length=1.0 at the origin with orientation=pi/2
(invalid configuration): s 0 0 0 1.57079 1.0

