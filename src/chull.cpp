//------------------------------------------------------------------------------
//  Copyright 2007-2019 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "chull.h"

namespace masc{
namespace polygon{

///////////////////////////////////////////////////////////////////////////////
#include <cassert>
using namespace std;

//
//s and e are the start and the end vertices of the polygon
//e mush be reachable from s
//
void hull2d(ply_vertex * s, ply_vertex * e, list<ply_vertex*>& hull )
{
  ///////////////////////////////////////////////////////////////////////////////
  // Implemets the idea from
  // A. Melkman, "On-line construction of the convex hull of a simple polygon",
  // Info. Proc. Letters 25, 11-12 (1987)
  //
  ///////////////////////////////////////////////////////////////////////////////
}

}//end namespace polygon
}//end namespace masc
