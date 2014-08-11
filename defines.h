/* 
 * File:   defines.h
 * Author: johannes
 *
 * Created on 1. April 2014, 09:25
 */

#ifndef DEFINES_H
#define	DEFINES_H

#include <iostream>
#include <list>

#include <CGAL/Polyhedron_3.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Gmpz.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/IO/Color.h>
#include <GL/glew.h>
#include <GL/glut.h>

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Simple_cartesian<float> K;

typedef K::FT FT;
typedef K::Point_3 Point_3;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 Triangle;
typedef K::Vector_3 Vector_3;
typedef K:: Aff_transformation_3  Aff_transformation_3;


struct myColor {
    double R;
    double G;
    double B;
};

// A face type with a three color member variables.
template <class Refs>
struct My_face : public CGAL::HalfedgeDS_face_base<Refs> {
    myColor color[3];
    double kappa[3];
};

template <class Refs>
struct My_halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {
    myColor color;
    int kappa;
    double orig[3];
};


// An items type using my face.
struct My_items : public CGAL::Polyhedron_items_3 {
    template <class Refs, class Traits>
    struct Face_wrapper {
        typedef My_face<Refs> Face;
    };
    template <class Refs, class Traits>
    struct Halfedge_wrapper {
        typedef My_halfedge<Refs> Halfedge;
    };
};

typedef CGAL::Polyhedron_3<K, My_items> Polyhedron;



//typedef Polyhedron::Aff_transformation_3 Aff_transformation_3;

typedef typename Polyhedron::Vertex Vertex;
typedef typename Polyhedron::Face_handle Face_handle;
typedef typename Polyhedron::Vertex_handle Vertex_handle;
typedef typename Polyhedron::Halfedge_handle Halfedge_handle;
typedef typename Polyhedron::Halfedge_around_vertex_circulator Vertex_circulator;
typedef Polyhedron::Vertex_iterator Vertex_iterator;
typedef Polyhedron::Facet_iterator Facet_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_around_facet_circulator;




typedef Polyhedron::Vertex                                   Vertex;
typedef Polyhedron::Vertex_iterator                          Vertex_iterator;
typedef Polyhedron::Halfedge_handle                          Halfedge_handle;
typedef Polyhedron::Edge_iterator                            Edge_iterator;
typedef Polyhedron::Facet_iterator                           Facet_iterator;
typedef Polyhedron::Halfedge_around_vertex_const_circulator  HV_circulator;
typedef Polyhedron::Halfedge_around_facet_circulator         HF_circulator;

#endif	/* DEFINES_H */

