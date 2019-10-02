#pragma once

#include "Point.h"
#include "polygon.h"
#include "Matrix.h"
#include <ostream>

#define DEBUG 0 //to enable debugging output, change to 1

namespace masc {
namespace polygon {

  //oritented bounding box (obb)
  struct obb
  {
      obb(){ width=height=FLT_MAX; }
      mathtool::Point2d corners[4];
      float width, height;
  };

  //the base class for all bounding box problems
  class bbox2d_problem
  {
    public:
      //return true if the problem is solved by the given box
      virtual bool solved(const obb& box)=0;
      const obb & getSolution() const { return m_solution; }

    protected:
      obb m_solution;
  };

  //You main task is to implement this class.
  class bbox2d
  {
  public:

      //initialize with a given polygon
      bbox2d(const c_polygon & poly);

      //build an obb of the provided polygon that solved the given problem.
      //return: obb that solves the given problem.
      obb build(bbox2d_problem & problem);

  private:

      // Calculate the parallel and normal vectors at the vertex of a polygon
      void calculateVectorsForVertex(const int startVertexIndex, mathtool::Vector2d& v, mathtool::Vector2d& n);
      
      // Calculate the inverse matrix that will allow us to compute extreme points
      mathtool::Matrix2x2 calculateInverseMatrix(const mathtool::Vector2d& v, const mathtool::Vector2d& n);
      
      // Calculate the extreme points given a point and the inverse of the orthonormal (v n) matrix.
      void calculateExtremePoints(const mathtool::Matrix2x2 inverseMatrix, const mathtool::Point2d& startPoint, int (&extremePointVertexIndices)[4]);
      
      // Calculate the intersection point of 2 point/direction pairs.
      mathtool::Point2d calculateIntersection(const mathtool::Point2d point0, const mathtool::Vector2d dir0, 
                                                const mathtool::Point2d point1, const mathtool::Vector2d dir1,
                                                double (&scaleParameters)[2]);
      
      //return the index of the smallest angle
      int findAngles(int e[4], float a[4], const mathtool::Vector2d& v, const mathtool::Vector2d& n);

      //create create OBB
      obb createOBB(int e[4],const mathtool::Vector2d& v, const mathtool::Vector2d& n);

      //data
      vector<mathtool::Point2d> m_chull; //convex hull of the input poly
  };

  //!!
  //READ BUT DO NOT CHANGE ANYTHING BELOW
  //we now define problems of finding various boudning boxes
  //!!

  //the problem of finding the minimum area bounding box
  class min_area_bbox : public bbox2d_problem
  {
    public:
      min_area_bbox(){ m_min_area=FLT_MAX; }
      bool solved(const obb& box);
    private:
      float m_min_area;
  };

  //the problem of finding the minimum boundary bounding box
  class min_perimeter_bbox : public bbox2d_problem
  {
    public:
      min_perimeter_bbox(){ m_min_peri=FLT_MAX; }
      bool solved(const obb& box);
    private:
      float m_min_peri;
  };

  //the problem of finding a bounding box that can fit into another box
  class contained_bbox : public bbox2d_problem
  {
    public:
      contained_bbox(float width, float height){m_width=width; m_height=height;}
      bool solved(const obb& box);
    private:
      float m_width, m_height;
  };

  //stream out the box
  std::ostream & operator<<(std::ostream& out, const obb& box);

  //save to svg file
  void saveSVG(string svg_filename, c_ply& ply, const obb& box);
  void saveSVG(string svg_filename, c_ply& ply);

}//end namespace polygon
}//end namespace masc
