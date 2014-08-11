/* 
 * File:   main.cpp
 * Author: johannes
 *
 * Created on 22. April 2014, 10:25
 */
#include "defines.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <CGAL/Gmpz.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>



#include <fstream>


#include "Utils.h"

#define window_width  1280              
#define window_height 1024

Polyhedron * P;
float dt = 0.0;
float angle = 0.0;


using namespace geometryUtils;

void keyboard(unsigned char key, int mousePositionX, int mousePositionY) {

}

void GL_Setup(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glEnable(GL_DEPTH_TEST);
    gluPerspective(45, (float) width / height, .1, 100);

    P = new Polyhedron;
    //path to polyhedra in *.off format
    std::ifstream stream("pig.off");
    //    std::ifstream stream("aircraft_open.off");    
    if (!stream) {
        std::cerr << "Cannot open file!\n";
    }

    stream >> *P;
    if (!stream) {
        std::cerr << "this is not a polyhedron\n";
    }
    std::cout << "\n ";
    std::cout << "\n Size of vertices:  " << P->size_of_vertices();
    std::cout << "\n Size of halfedges:  " << P->size_of_halfedges();
    std::cout << "\n Size of facets:  " << P->size_of_facets();
    std::cout << "\n ";

    
    //subdivide loaded mesh
    subdivide(*P);


    computeGaussCurvature(P);

}

void reshape(int width, int height) {
    glMatrixMode(GL_PROJECTION);
    glViewport(0, 0, width, height);
    glLoadIdentity();
    gluPerspective(45, (float) width / height, .1, 1000);
}



void compute() {
};

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    gluLookAt(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glRotatef(angle, 1.0, 1.0, 0.0);
    
    dt += 0.1;
    renderPolyhedron(P);

    glPopMatrix();
    angle += 0.3;
    glutSwapBuffers();
    glutPostRedisplay();

}

int main(int argc, char** argv) {

    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutCreateWindow("cgalTest");
    GLenum err = glewInit();
    if (GLEW_OK != err) {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    }
    fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    glutDisplayFunc(display);
    glutIdleFunc(compute);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);

    GL_Setup(window_width, window_height);
    glutMainLoop();


    return 0;
}