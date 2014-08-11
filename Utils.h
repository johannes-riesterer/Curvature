/* 
 * File:   Utils.h
 * Author: johannes
 *
 * Created on 11. August 2014, 10:17
 */

#ifndef UTILS_H
#define	UTILS_H
#include "defines.h"



namespace geometryUtils {
    
    //controls  resolution for color conversion
    static float kappaMax = 1.0;

    //convert color from RGB to HSV
    myColor HSVtoRGB(int H, double S, double V);

    //compute voronoi area of a vertex
    double computeVoronoiArea(Vertex_handle vertex);

    //compute gaussian curvature at a vertex
    double computeLocalGaussCurvature(Vertex_handle vertex);

    void computeGaussCurvature(Polyhedron* P);



    //subdivision algorithm for CGAL plyhedron datastrucutre
    //------------------------------------------------------------

    void subdivide_create_center_vertex(Polyhedron& P, Facet_iterator f);

    void subdivide_flip_edge(Polyhedron& P, Halfedge_handle e);

    void subdivide(Polyhedron& P);
    
    //Draw polyhedral mesh with color information based on gaussian curvature
    void renderPolyhedron(Polyhedron * pmesh);

}
#endif	/* UTILS_H */

