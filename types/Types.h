#ifndef CICP_TYPES_H
#define CICP_TYPES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

typedef std::pair<Point, Vector> PointVectorPair;

#endif
