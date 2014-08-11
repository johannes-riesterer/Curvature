/* 
 * File:   Utils.cpp
 * Author: johannes
 * 
 * Created on 11. August 2014, 10:17
 */

#include "Utils.h"

struct Smooth_old_vertex {

    Point_3 operator()(const Vertex& v) const {
        CGAL_precondition((CGAL::circulator_size(v.vertex_begin()) & 1) == 0);
        std::size_t degree = CGAL::circulator_size(v.vertex_begin()) / 2;
        double alpha = (4.0 - 2.0 * std::cos(2.0 * CGAL_PI / degree)) / 9.0;
        Vector_3 vec = (v.point() - CGAL::ORIGIN) * (1.0 - alpha);
        HV_circulator h = v.vertex_begin();
        do {
            vec = vec + (h->opposite()->vertex()->point() - CGAL::ORIGIN)
                    * alpha / static_cast<double> (degree);
            ++h;
            CGAL_assertion(h != v.vertex_begin()); // even degree guaranteed
            ++h;
        } while (h != v.vertex_begin());
        return (CGAL::ORIGIN + vec);
    }
};

myColor geometryUtils::HSVtoRGB(int H, double S, double V) {
    //enforce assumptions for HSV

    if (H > 300) {
        H = 300;
    }
    if (H < 0) {
        H = 0;
    }
    if (S > 1.0) {
        S = 1.0;
    }
    if (S < 0) {
        S = 0.0;
    }
    if (V > 1) {
        V = 1.0;
    }
    if (V < 0) {
        V = 0.0;
    }

    double R;
    double G;
    double B;

    double C = V*S;
    double X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    double m = V - C;

    if (H <= 60) {
        R = 1;
        G = 1;
        B = 0;
    }
    /*    if(H <60 && H>30) {
            R = C;
            G = X;
            B = 0;
        }
     */
    if (H < 120 && H >= 60) {
        R = X;
        G = C;
        B = 0;
    }

    if (H < 180 && H >= 120) {
        R = 0;
        G = C;
        B = X;
    }


    if (H < 240 && H >= 180) {
        R = 0;
        G = X;
        B = C;
    }


    if (H < 300 && H >= 240) {
        R = X;
        G = 0;
        B = C;
    }


    if (H <= 360 && H >= 300) {
        R = C;
        G = 0;
        B = X;
    }




    myColor color;
    color.R = R + m;
    color.G = G + m;
    color.B = B + m;
    return color;
};

double geometryUtils::computeVoronoiArea(Vertex_handle vertex) {
    double voronoiArea = 0.0;
    Vertex_circulator j;

    j = vertex->vertex_begin();

    do {
        Point_3 p11 = j->vertex()->point();
        Point_3 p12 = j->next()->vertex()->point();
        Point_3 p13 = j->next()->next()->vertex()->point();
        Vector_3 v11 = p13 - p12;
        Vector_3 v12 = p11 - p12;
        v11 = v11 / sqrt(CGAL::to_double(v11.squared_length()));
        v12 = v12 / sqrt(CGAL::to_double(v12.squared_length()));
        double alpha = acos(CGAL::to_double(v11 * v12));
        Point_3 p22 = j->opposite()->vertex()->point();
        Point_3 p23 = j->opposite()->next()->vertex()->point();
        Vector_3 v21 = p11 - p23;
        Vector_3 v22 = p22 - p23;
        v21 = v21 / sqrt(CGAL::to_double(v21.squared_length()));
        v22 = v22 / sqrt(CGAL::to_double(v22.squared_length()));

        double beta = acos(CGAL::to_double(v21 * v22));
        Vector_3 x = p13 - p11;
        double length = CGAL::to_double(x.squared_length());

        voronoiArea += (1.0 / 8.0) * (1.0 / tan(alpha) + 1.0 / tan(beta)) * length;

    } while (++j != vertex->vertex_begin());

    return voronoiArea;

};

double geometryUtils::computeLocalGaussCurvature(Vertex_handle vertex) {

    double gaussCurvature = 0.0;
    double vA = computeVoronoiArea(vertex);
    double sumTheta = 0.0;
    Vertex_circulator j;
    j = vertex->vertex_begin();

    do {
        Point_3 p1 = j->vertex()->point();
        Point_3 p2 = j->prev()->vertex()->point();
        Point_3 p3 = j->next()->vertex()->point();

        Vector_3 v1 = p2 - p1;
        Vector_3 v2 = p3 - p1;

        v1 = v1 / sqrt(CGAL::to_double(v1.squared_length()));
        v2 = v2 / sqrt(CGAL::to_double(v2.squared_length()));

        sumTheta += acos(CGAL::to_double(v1 * v2));
    } while (++j != vertex->vertex_begin());

    gaussCurvature = (2 * 3.1415926 - sumTheta) / vA;

    return gaussCurvature;
};

void geometryUtils::computeGaussCurvature(Polyhedron* P) {
    for (Facet_iterator i = P->facets_begin(); i != P->facets_end(); i++) {

        int e = 0;
        Halfedge_around_facet_circulator edge = i->facet_begin();
        do {
            i->kappa[e] = computeLocalGaussCurvature(edge->vertex());
            int H = floor(i->kappa[e] * geometryUtils::kappaMax + 180);

            //            std::cout << H;
            //            std::cout << " ";

            i->color[e] = HSVtoRGB(H, 1.0, 1.0);

            e++;
        } while (++edge != i->facet_begin());
        //        std::cout << edge->kappa;
        //        std::cout << " ";
    }
};

void geometryUtils::subdivide_create_center_vertex(Polyhedron& P, Facet_iterator f) {
    Vector_3 vec(0.0, 0.0, 0.0);
    std::size_t order = 0;
    HF_circulator h = f->facet_begin();
    do {
        vec = vec + (h->vertex()->point() - CGAL::ORIGIN);
        ++order;
    } while (++h != f->facet_begin());
    CGAL_assertion(order >= 3); // guaranteed by definition of polyhedron
    Point_3 center = CGAL::ORIGIN + (vec / static_cast<double> (order));
    Halfedge_handle new_center = P.create_center_vertex(f->halfedge());
    new_center->vertex()->point() = center;
}

void geometryUtils::subdivide_flip_edge(Polyhedron& P, Halfedge_handle e) {
    Halfedge_handle h = e->next();
    P.join_facet(e);
    P.split_facet(h, h->next()->next());
}

void geometryUtils::subdivide(Polyhedron& P) {
    if (P.size_of_facets() == 0)
        return;
    // We use that new vertices/halfedges/facets are appended at the end.
    std::size_t nv = P.size_of_vertices();
    Vertex_iterator last_v = P.vertices_end();
    --last_v; // the last of the old vertices
    Edge_iterator last_e = P.edges_end();
    --last_e; // the last of the old edges
    Facet_iterator last_f = P.facets_end();
    --last_f; // the last of the old facets
    Facet_iterator f = P.facets_begin(); // create new center vertices
    do {
        geometryUtils::subdivide_create_center_vertex(P, f);
    } while (f++ != last_f);
    std::vector<Point_3> pts; // smooth the old vertices
    pts.reserve(nv); // get intermediate space for the new points
    ++last_v; // make it the past-the-end position again
    std::transform(P.vertices_begin(), last_v, std::back_inserter(pts),
            Smooth_old_vertex());
    std::copy(pts.begin(), pts.end(), P.points_begin());
    Edge_iterator e = P.edges_begin(); // flip the old edges
    ++last_e; // make it the past-the-end position again
    while (e != last_e) {
        Halfedge_handle h = e;
        ++e; // careful, incr. before flip since flip destroys current edge
        geometryUtils::subdivide_flip_edge(P, h);
    };
    CGAL_postcondition(P.is_valid());
};

void geometryUtils::renderPolyhedron(Polyhedron * pmesh) {

    glBegin(GL_TRIANGLES);
    for (Facet_iterator i = pmesh->facets_begin(); i != pmesh->facets_end(); i++) {


        Halfedge_around_facet_circulator j = i->facet_begin();
        glColor3d(i->color[0].R, i->color[0].G, i->color[0].B);
        glVertex3d(CGAL::to_double(j->vertex()->point().x()), CGAL::to_double(j->vertex()->point().y()), CGAL::to_double(j->vertex()->point().z()));

        glColor3d(i->color[1].R, i->color[1].G, i->color[1].B);
        glVertex3d(CGAL::to_double(j->next()->vertex()->point().x()), CGAL::to_double(j->next()->vertex()->point().y()), CGAL::to_double(j->next()->vertex()->point().z()));

        glColor3d(i->color[2].R, i->color[2].G, i->color[2].B);
        glVertex3d(CGAL::to_double(j->next()->next()->vertex()->point().x()), CGAL::to_double(j->next()->next()->vertex()->point().y()), CGAL::to_double(j->next()->next()->vertex()->point().z()));
    }
    glEnd();
}