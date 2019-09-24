#  GMU CS633 Programming Assignment 01: Rotating Caliper for Computing the Bounding Boxes

### Due: Oct. 2nd., before mid night
### What to submit: 

A zip (\*.zip) file that includes: (1) A short readme.txt file explaining  (2) Two examples of input (\*.poly) and output (\*.svg) produced by your code. (3) Your code.

### Goals: 
1. Implement convex hull algorithm, "On-line construction of the convex hull of a simple polygon", by A. Melkman.
   - You can find a short introduction of the paper and code here: http://geomalgorithms.com/a12-_hull-3.html
   - Understand the paper and the code, and re-implement it using the provided frameowrk
      - This means use the predefined data structures only: such as polygon, vector, point and matrix classes;
  
2. Implement the rotating calipers algorithm to compute the smallest bounding boxes of a polygon
   - Use the convex hull build from Step#1, compute the smallest bounding boxes
   - Rotating calipers: https://en.wikipedia.org/wiki/Rotating_calipers
   - Solving Geometric Problems with the Rotating Calipers by G. Toussaint https://www.cs.swarthmore.edu/~adanner/cs97/s08/pdf/calipers.pdf
    - An interactive demo: http://bkiers-demos.appspot.com/rotating-calipers
