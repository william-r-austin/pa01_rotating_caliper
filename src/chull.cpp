//------------------------------------------------------------------------------
//  Copyright 2007-2019 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "chull.h"
#include "intersection.h"

namespace masc{
namespace polygon{

///////////////////////////////////////////////////////////////////////////////
#include <cassert>
using namespace std;



bool isLeft(const Point2d& point0, const Point2d& point1, const Point2d& point2) {
    double x[2] = {4, 4};
    double y[2] = {1, 3};
    double z[2] = {0, 8};
    
    int area2 = masc::polygon::Area(x, y, z);
    int areaSign = masc::polygon::AreaSign(x, y, z);
    
    cout << "Area = " << area2 << ", AreaSign = " << areaSign << endl;
    
  double a = point1[0] - point0[0];
  double b = point2[1] - point0[1];
  double c = point2[0] - point0[0];
  double d = point1[1] - point0[1];

  double area = (a * b) - (c * d);
  bool result = area > 0;
  
  cout << "isLeft calculation for: " << point0 << " ==> " << point1 << " ==> " << point2 
    << " is: " << result << " (" << area << ")" << endl;
  
  return result;
}

bool isLeft(ply_vertex* p0, ply_vertex* p1, ply_vertex* p2) {
  return isLeft(p0->getPos(), p1->getPos(), p2->getPos());
  
}

//
//s and e are the start and the end vertices of the polygon
//e mush be reachable from s
//
void hull2d(ply_vertex * startVertex, ply_vertex * endVertex, list<ply_vertex*>& hull)
{
  ///////////////////////////////////////////////////////////////////////////////
  // Implemets the idea from
  // A. Melkman, "On-line construction of the convex hull of a simple polygon",
  // Info. Proc. Letters 25, 11-12 (1987)
  //
  ///////////////////////////////////////////////////////////////////////////////
  ply_vertex* p0 = startVertex;
  ply_vertex* p1 = p0->getNext();
  ply_vertex* p2 = p1->getNext();
  
  bool vertex2OnLeft = isLeft(p0, p1, p2);
  vector<ply_vertex*> convexHullDeque ({p2});
  
  if(vertex2OnLeft) {
    convexHullDeque.push_back(p0);
    convexHullDeque.push_back(p1);
  }
  else {
    convexHullDeque.push_back(p0);
    convexHullDeque.push_back(p1);
  }
  
  convexHullDeque.push_back(p2);
  
  ply_vertex* current = p2;

  while(current != endVertex) {
    // Part 1.
    ply_vertex* bottom = convexHullDeque.at(0);
    ply_vertex* bottomNext = convexHullDeque.at(1);
    
    int convexHullSize = convexHullDeque.size();
    ply_vertex* top = convexHullDeque.at(convexHullSize - 1);
    ply_vertex* topPrev = convexHullDeque.at(convexHullSize - 2);
    
    bool part1Left = isLeft(bottom, bottomNext, current);
    bool part2Left = isLeft(topPrev, top, current);
    
    if(!part1Left || !part2Left) {
       
      while(!part1Left) {
        convexHullDeque.erase(convexHullDeque.begin());
      
        bottom = convexHullDeque.at(0);
        bottomNext = convexHullDeque.at(1);
        part1Left = isLeft(bottom, bottomNext, current);
      }
    
      convexHullDeque.insert(convexHullDeque.begin(), current);
    
      // Part 2
      while(!part2Left) {
        convexHullDeque.pop_back();
       
        convexHullSize = convexHullDeque.size();
        top = convexHullDeque.at(convexHullSize - 1);
        topPrev = convexHullDeque.at(convexHullSize - 2);
        part2Left = isLeft(topPrev, top, current);
      }
      
      convexHullDeque.push_back(current);
    }

    current = current->getNext();
  }
  
  hull.clear();
  copy(convexHullDeque.begin(), convexHullDeque.end(), back_inserter(hull));
}

}//end namespace polygon
}//end namespace masc
