CS 633, Computational Geometry
Programming Assignment #1: Minimum Boxes (Convex Hull and Rotating Calipers)
Due: Midnight on October 2nd, 2019 
Student Name: William Austin

In this project we have implemented Melkman's algorithm for computing the convex hull from a simple polyline and then used the rotating calipers technique to find minimal bounding boxes for the original polygon, based on the computed convex hull.

Some notes about the code changes are below:
- In this zip file, all code is located in the "src/" directory. No files have been added, moved, or deleted from the original project specification.
- I added several helper functions in these files to make my code more modular, but I maintained the definition for all given functions.
- I added includes for intersection.h and Matrix.h so I could re-use some provided functionality.
- All of the data types that I used were built-in, provided by the project, or included in the standard library (vector and list).
- Comments have been added where appropriated. Console output that I added for testing has been removed or commented out.
- While testing the hull2d() function, I added code in main.cpp to output just an image of the convex hull. I kept this functionality, and the additional output is provided for the given samples. 
- As expected, all of the code modifications are in the following places:
  1. main.cpp
  2. chull.h / chull.cpp
  3. bbox2d.h / bbox2d.cpp

Finally, as requested, I have provided the two samples of input/output for the application. These are located in the "samples/" directory. These are:
(1) simple3.poly - a sample that can successfully be squeezed inside a 13 x 13 square.
(2) simple4.poly - a sample that has a different bounding box for min area and min perimeter. This example is also highly non-convex.

* In addition to the .poly and .svg files, I have also included a "console_output.png" file that shows the console output that is created when running the program with the given sample.

