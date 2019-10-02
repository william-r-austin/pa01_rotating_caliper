#include "bbox2d.h"
#include "chull.h"
#include "simple_svg_1.0.0.hpp"
#include "Matrix.h"

namespace masc {
namespace polygon {

//initialize with a given polygon
bbox2d::bbox2d(const c_polygon & poly)
{
  const c_ply& ply=poly.front();
  list<ply_vertex*> hull;
  hull2d(ply.getHead(), ply.getHead()->getPre(), hull);

  for(ply_vertex* v:hull)
  {
    auto pos=v->getPos();
    this->m_chull.push_back(pos);
  }//end for
}

void bbox2d::calculateVectorsForVertex(const int startVertexIndex, mathtool::Vector2d& v, mathtool::Vector2d& n) 
{
    int convexHullSize = m_chull.size();
  
    Point2d startPoint = m_chull[startVertexIndex];
    Point2d endPoint = m_chull[(startVertexIndex + 1) % convexHullSize];
  
    Point2d vAsPoint2d = endPoint - startPoint;

    v.set(vAsPoint2d[0], vAsPoint2d[1]);
    n.set(-1 * vAsPoint2d[1], vAsPoint2d[0]); // N points to the interior of the polygon.
}

mathtool::Matrix2x2 bbox2d::calculateInverseMatrix(const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
    mathtool::Matrix2x2 a(v[0], n[0], v[1], n[1]);
    mathtool::Matrix2x2 aInverse = a.inv();
    return aInverse;
}

void bbox2d::calculateExtremePoints(const mathtool::Matrix2x2 inverseMatrix, const mathtool::Point2d& startPoint, int (&extremePointVertexIndices)[4]) 
{
    int convexHullSize = m_chull.size();
    Vector2d startPointAsVector(startPoint[0], startPoint[1]);
    bool firstPoint = true;
    double caliperValues[4];
    
    for(int i = 0; i < convexHullSize; i++) 
    {
        Point2d iPoint = m_chull[i];
        Vector2d iVector = Vector2d(iPoint[0], iPoint[1]);
        Vector2d translated = iVector - startPointAsVector;
        
        Vector2d componentsVector = inverseMatrix * translated;
        cout << "componentsVector = " << componentsVector << endl;
        double vComponent = componentsVector[0];
        double nComponent = componentsVector[1];
      
        cout << "build(), point index = " << i << ", point = " << iPoint << ", Parallel (v) component = " << vComponent 
            << ", Normal (n) component = " << nComponent << endl;
        
        if(firstPoint) 
        {
            for(int k = 0; k < 3; k++) 
            {
                extremePointVertexIndices[k] = i;
                if(k % 2 == 0)
                {
                    caliperValues[k] = nComponent;
                }
                else 
                {
                    caliperValues[k] = vComponent;
                }
            }
          
            firstPoint = false;
        }
        else 
        {
            // Instead of < and >, use !>  and !<
            if(!(nComponent > caliperValues[0])) 
            {
                extremePointVertexIndices[0] = i;
                caliperValues[0] = nComponent;
            }
          
            if(!(vComponent < caliperValues[1]))
            {
                extremePointVertexIndices[1] = i;
                caliperValues[1] = vComponent;
            }
          
            if(!(nComponent < caliperValues[2])) 
            {
                extremePointVertexIndices[2] = i;
                caliperValues[2] = nComponent;
            }
          
            if(!(vComponent > caliperValues[3])) 
            {
                extremePointVertexIndices[3] = i;
                caliperValues[3] = vComponent;
            }
        }
    }
}

obb bbox2d::build(bbox2d_problem & problem)
{
    mathtool::Vector2d v,n; //the calipers, v & n are perpendicular
    int e[4]; //vertex indices of extreme points
    float a[4]; //angles between the calipers and the polygon edges
  
    //double caliperValues[4];
    //double bCaliperMinMax[2];

    // 1. initialize v so it is parallel to an edge and then determine n
    int startingVertexIndex = 0;
    calculateVectorsForVertex(startingVertexIndex, v, n);
    /*int convexHullSize = m_chull.size();
  
  Point2d startPoint = m_chull[0];
  Point2d endPoint = m_chull[1];
  
  Point2d vAsPoint2d = endPoint - startPoint;
  v.set(vAsPoint2d[0], vAsPoint2d[1]);
  //v.normalize();
  n.set(-1 * vAsPoint2d[1], vAsPoint2d[0]); // N points to the interior of the polygon.
  //n.normalize(); 
  cout << "build(), convexHullSize = " << convexHullSize << endl;
  cout << "build(), v = " << v << ", n = " << n << endl;
  
  Vector2d startPointAsVector(startPoint[0], startPoint[1]);
*/
  
    // 2. init extreme points e[4] using v & n, compute angles a[4]
    mathtool::Matrix2x2 inverseMatrix = calculateInverseMatrix(v, n);
    /*
  Matrix2x2 A(v[0], n[0], v[1], n[1]);
  //Matrix2x2 B(n[0], -1 * v[0], n[1], -1 * v[1]);
  Matrix2x2 AInverse = A.inv();
  //Matrix2x2 BInverse = B.inv();
  
  cout << "build(), A = " << endl;
  A.print();
  
  cout << "build(), A-Inverse = " << endl;
  AInverse.print();
  cout << "(0, 0): " << AInverse[0][0] << endl;
  cout << "(0, 1): " << AInverse[0][1] << endl;
  cout << "(1, 0): " << AInverse[1][0] << endl;
  cout << "(1, 1): " << AInverse[1][1] << endl;
  */
    
    calculateExtremePoints(inverseMatrix, m_chull[startingVertexIndex], e);
    
    int minAngleIndex = findAngles(e, a, v, n);
    
    /*
  bool firstPoint = true;
  
  for(int i = 0; i < convexHullSize; i++) {
      
      Point2d iPoint = m_chull[i];
      Vector2d iVector = Vector2d(iPoint[0], iPoint[1]);
      
      Vector2d translated = iVector - startPointAsVector;
      
      
      Vector2d componentsVector = AInverse * translated;
      cout << "componentsVector = " << componentsVector << endl;
      double vComponent = componentsVector[0];
      double nComponent = componentsVector[1];
      
      //Vector2d nVector = BInverse * iVector;
      //cout << "nVector = " << nVector << endl;
            
      cout << "build(), point index = " << i << ", point = " << iPoint << ", Parallel (v) component = " << vComponent << ", Normal (n) component = " << nComponent << endl;
      
      if(firstPoint) {
          for(int k = 0; k < 3; k++) {
              e[k] = i;
              if(k % 2 == 0) {
                  caliperValues[k] = nComponent;
              }
              else {
                  caliperValues[k] = vComponent;
              }
          }
          
          firstPoint = false;
      }
      else {
          // Instead of < and >, use !>  and !<
          if(!(nComponent > caliperValues[0])) {
              e[0] = i;
              caliperValues[0] = nComponent;
          }
          
          if(!(vComponent < caliperValues[1])) {
              e[1] = i;
              caliperValues[1] = vComponent;
          }
          
          if(!(nComponent < caliperValues[2])) {
              e[2] = i;
              caliperValues[2] = nComponent;
          }
          
          if(!(vComponent > caliperValues[3])) {
              e[3] = i;
              caliperValues[3] = vComponent;
          }
      }
  }
  
  
  */
    /*
  for(int k = 0; k < 4; k++) {
      Vector2d boxLine;
      if(k == 0) {
          boxLine.set(v);
      }
      else if(k == 1) {
          boxLine.set(n);
      }
      else if(k == 2) {
          boxLine.set(v * -1);
      }
      else if(k == 3) {
          boxLine.set(n * -1);
      }
      
      Vector2d polyLine;
      int extremeIndex = e[k];
      Point2d& extremeStartPoint = m_chull[extremeIndex];
          
      
      Point2d& extremeEndPoint = m_chull[(extremeIndex + 1) % convexHullSize];
      Point2d polyLineAsPoint = extremeEndPoint - extremeStartPoint;
      polyLine.set(polyLineAsPoint[0], polyLineAsPoint[1]);
      
      cout << "build(), Calculated extreme point " << k << ", CH Line = " << extremeIndex << ", (" << extremeStartPoint << ") --> (" << extremeEndPoint << ")" << endl;
      
      double dotProduct = polyLine * boxLine;
      double magnitudeProduct = polyLine.norm() * boxLine.norm();
      a[k] = (float) (dotProduct / magnitudeProduct);
  }
*/
    
    obb orientedBoundingBox = createOBB(e, v, n);
    problem.solved(orientedBoundingBox);
    
  // 3. iteratively update extreme points
  for(int i=0;i<m_chull.size();i++)
  {
    //3.1 create a box from v,n,e[4]
    //3.2 check if this box solve the problem (use problem.solved)
    //3.3 update v,n,e[4],a[4]
  }

  return problem.getSolution(); //done
}

mathtool::Point2d bbox2d::calculateIntersection(const mathtool::Point2d point0, const mathtool::Vector2d dir0, 
                                                const mathtool::Point2d point1, const mathtool::Vector2d dir1,
                                                double (&scaleParameters)[2])
{
    mathtool::Point2d diff = point1 - point0;
    mathtool::Vector2d diffVector = Vector2d(diff[0], diff[1]);
    mathtool::Matrix2x2 vnInverse = calculateInverseMatrix(dir0, dir1);
    mathtool::Vector2d vnComponents = vnInverse * diffVector;
    scaleParameters[0] = vnComponents[0];
    scaleParameters[1] = vnComponents[1] * -1;
  
    mathtool::Vector2d point0AsVector(point0[0], point0[1]);
    mathtool::Vector2d point1AsVector(point1[0], point1[1]);
  
    mathtool::Vector2d intersectionVector = point0AsVector + (dir0 * vnComponents[0]);
    //mathtool::Vector2d nCorner0 = point1AsVector + (dir1 * (-1 * vnComponents[1]));
    
    mathtool::Point2d intersectionPoint(intersectionVector[0], intersectionVector[1]);
    return intersectionPoint;
}

//TODO:
//compute "angles" or some values that can be used to sort angles between
//the caliper and the edges
//
// e: 4 extreme points on the convex hull
// a: the angles to be computed between the calipers and the edges incident to the extreme points
// u, v: the calipers
//
// return a value i=0~4, where a[i] is the smallest
int bbox2d::findAngles(int e[4], float a[4], const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
    int convexHullSize = m_chull.size();
    float maxCosTheta;
    int minAngleIndex;
    
    for(int k = 0; k < 4; k++) {
        Vector2d boxLine;
        if(k == 0) {
            boxLine.set(v);
        }
        else if(k == 1) 
        {
            boxLine.set(n);
        }
        else if(k == 2)
        {
            boxLine.set(v * -1);
        }
        else if(k == 3) {
            boxLine.set(n * -1);
        }
      
        Vector2d polyLine;
        int extremeIndex = e[k];
        Point2d& extremeStartPoint = m_chull[extremeIndex];
          
      
        Point2d& extremeEndPoint = m_chull[(extremeIndex + 1) % convexHullSize];
        Point2d polyLineAsPoint = extremeEndPoint - extremeStartPoint;
        polyLine.set(polyLineAsPoint[0], polyLineAsPoint[1]);
      
        cout << "build(), Calculated extreme point " << k << ", CH Line = " << extremeIndex << ", (" << extremeStartPoint << ") --> (" <<                                   
            extremeEndPoint << ")" << endl;
      
        double dotProduct = polyLine * boxLine;
        double magnitudeProduct = polyLine.norm() * boxLine.norm();
        float cosTheta = (float) (dotProduct / magnitudeProduct);
        a[k] = cosTheta;
        
        if(k == 0)
        {
            maxCosTheta = cosTheta;
            minAngleIndex = k;
        }
        else 
        {
            if(cosTheta > maxCosTheta)
            {
                maxCosTheta = cosTheta;
                minAngleIndex = k;
            }
        }
    }
    
    return minAngleIndex;
}

obb bbox2d::createOBB(int e[4],const mathtool::Vector2d& v, const mathtool::Vector2d& n)
{
  obb box;
  
  // Start with: Corner 0 = ExtremePoint[0] + j * V = ExtremePoint[1] + k * N
  /*int index0 = e[0];
  int index1 = e[1];
  int index
  mathtool::Point2d point0 = m_chull[index0];
  mathtool::Point2d point1 = m_chull[index1];
  mathtool::Point2d */
  
  double scaleParams[2];
  box.corners[0] = calculateIntersection(m_chull[e[0]], v, m_chull[e[1]], n, scaleParams);
  box.corners[1] = calculateIntersection(m_chull[e[1]], n, m_chull[e[2]], v, scaleParams);
  box.corners[2] = calculateIntersection(m_chull[e[2]], v, m_chull[e[3]], n, scaleParams);
  box.corners[3] = calculateIntersection(m_chull[e[3]], n, m_chull[e[0]], v, scaleParams);
  
  mathtool::Point2d widthVecAsPoint = box.corners[3] - box.corners[0];
  mathtool::Point2d heightVecAsPoint = box.corners[1] - box.corners[0];
  
  mathtool::Vector2d widthVec(widthVecAsPoint[0], widthVecAsPoint[1]);
  mathtool::Vector2d heightVec(heightVecAsPoint[0], heightVecAsPoint[1]);
  
  box.width = (float) widthVec.norm();
  box.height = (float) heightVec.norm();
  
  /*
  mathtool::Point2d diff = point1 - point0;
  mathtool::Vector2d diffVector = Vector2d(diff[0], diff[1]);
  
  mathtool::Matrix2x2 vnInverse = calculateInverseMatrix(v, n);
  mathtool::Vector2d vnComponents = vnInverse * diffVector;
  
  mathtool::Vector2d point0AsVector(point0[0], point0[1]);
  mathtool::Vector2d point1AsVector(point1[0], point1[1]);
  
  mathtool::Vector2d vCorner0 = point0AsVector + (v * vnComponents[0]);
  mathtool::Vector2d nCorner0 = point1AsVector + (n * (-1 * vnComponents[1]));
  
  cout << "Corner 0, based on v = " << vCorner0 << endl;
  cout << "Corner 0, based on n = " << nCorner0 << endl;*/
  
  
  

  //TODO: build a box from e, the extreme points, v and n

  return box;
}

//
// DO NOT CHANGE ANYTHING BELOW
//
// 3 different types of bounding box
//


//the problem of finding the minimum area bounding box
bool min_area_bbox::solved(const obb& box)
{
  float area=box.width*box.height;
  if(area<m_min_area){
    m_min_area=area;
    this->m_solution=box;
  }
  return false; //always return false so search continues;
}

//the problem of finding the minimum boundary bounding box
bool min_perimeter_bbox::solved(const obb& box)
{
  float peri=box.width+box.height;
  if(peri<m_min_peri){
    m_min_peri=peri;
    this->m_solution=box;
  }
  return false; //always return false so search continues;
}

//the problem of finding a bounding box that can fit into another box
bool contained_bbox::solved(const obb& box)
{
  if( (box.width<=m_width && box.height <= m_height) ||
      (box.height<=m_width && box.width <= m_height) )
  {
    this->m_solution=box;
    return true;
  }

  return false;
}

//stream out the box
std::ostream & operator<<(std::ostream& out, const obb& box)
{
  out<<"[w="<<box.width<<", h="<<box.height<<"], ("
     <<box.corners[0]<<") ,("<<box.corners[1]<<"), ("
     <<box.corners[2]<<") ,("<<box.corners[3]<<")";
  return out;
}

//
//SVG related methods below. DO NOT change!
//
//function for saving svg file
//
void ply2ply(const masc::polygon::c_ply& ply, svg::Polygon& poly)
{
    auto v=ply.getHead();
    do{
      auto & pos=v->getPos();
      poly << svg::Point(pos[0], pos[1]);
      v=v->getNext();
    }
    while(v!=ply.getHead());
    poly.endBoundary();
}

void box2ply(const masc::polygon::obb& box, svg::Polygon& poly)
{
    poly << svg::Point(box.corners[0][0], box.corners[0][1]);
    poly << svg::Point(box.corners[1][0], box.corners[1][1]);
    poly << svg::Point(box.corners[2][0], box.corners[2][1]);
    poly << svg::Point(box.corners[3][0], box.corners[3][1]);
    poly.endBoundary();
}

void saveSVG(string svg_filename, masc::polygon::c_ply& ply, const masc::polygon::obb& box)
{
    auto R=ply.getRadius();
    auto center=ply.getCenter();

    svg::Dimensions dimensions(R*2.5, R*2.5);
    svg::Document doc(svg_filename, svg::Layout(dimensions, svg::Layout::BottomLeft, 1, svg::Point(-center[0]+R*1.25, -center[1]+R*1.25)));

    //------------------------------------------------------------------
    //draw the external boundary
    svg::Polygon box_bd(svg::Fill(svg::Color::Yellow), svg::Stroke(0.5, svg::Color::Black));
    box2ply(box, box_bd);
    doc << box_bd;
    svg::Polygon poly_bd(svg::Fill(svg::Color::Silver), svg::Stroke(0.5, svg::Color::Black));
    ply2ply(ply, poly_bd);
    doc << poly_bd;

    doc.save();
    cout << "- Saved " << svg_filename << endl;
}


void saveSVG(string svg_filename, masc::polygon::c_ply& ply)
{
    double scale = 2.5;
    double halfScale = scale / 2;
    auto R=ply.getRadius();
    auto center=ply.getCenter();

    svg::Dimensions dimensions(R*scale, R*scale);
    svg::Document doc(svg_filename, svg::Layout(dimensions, svg::Layout::BottomLeft, 1, svg::Point(-center[0]+R*halfScale, -center[1]+R*halfScale)));

    //------------------------------------------------------------------
    //draw the external boundary
    svg::Polygon poly_bd(svg::Fill(svg::Color::Silver), svg::Stroke(0.5, svg::Color::Black));
    ply2ply(ply, poly_bd);
    doc << poly_bd;

    doc.save();
    cout << "- Saved " << svg_filename << endl;
}

}}//end namespaces
