#pragma once
#include "Mathematics.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>

template<class Items>
class Base
{
public:
    typedef typename CGAL::Polyhedron_3<Kernel,Items> Polyhedron;

        
    Base(){}
    ~Base(){}
    
    std::size_t size_of_faces(){
        return mesh.size_of_facets();
    }
    
    std::size_t size_of_vertices(){
        return mesh.size_of_vertices();
    }
    
    void loadMesh(std::string path){
        std::ifstream in(path);
        if(!(in>>mesh)){
            std::cerr<<"Read mesh fail!\n";
            std::exit(1);
        }
        
        if(!mesh.is_pure_triangle()){
            std::cerr<<"Only support triangle mesh\n";
            std::exit(1);
        }
    }
    
    void saveMesh(std::string path){
        std::ofstream out(path);
        out<<mesh;
    }

    std::vector<std::vector<float>> selectedVerts;
    // void setSelectedVerts(std::vector<std::vector<float>> verts) {
    //     selectedVerts = verts;
    // }

protected:

    Vector3 FaceNormal(typename Polyhedron::Facet_const_handle f);
    Vector3 vertexNormal(typename Polyhedron::Vertex_const_handle v);
    bool checkSubdivide(typename Polyhedron::Facet_const_handle f, std::map<typename Polyhedron::Vertex_handle, Point3> vertexBuffer);

    Polyhedron mesh;
};

//Check if this face is in the vertexBuffer
// template<class Items>
// bool Base<Items>::checkSubdivide(typename Polyhedron::Facet_const_handle f, std::map<typename Polyhedron::Vertex_handle, Point3> vertexBuffer)
// {
//     typename Polyhedron::Halfedge_around_facet_circulator h=f->facet_begin();
//     // check if any of the face's vertices is in the vertexBuffer
//     do{
//         // typename Polyhedron::Halfedge_const_handle oh=h->opposite();
//         // get vertices of the face
//         typename Polyhedron::Vertex_handle v = h->vertex();
//         // check if the vertex is on the vertexBuffer
//         if (vertexBuffer.find(v) != vertexBuffer.end()) {
//             return true;
//         }
//         ++h;
//     }while(h!=f->facet_begin());
//     //Otherwise not subdivide this face
//     return false;
// }

// template<class Items>
// bool Base<Items>::checkSubdivide(typename Polyhedron::Facet_const_handle f, std::vector<std::vector<float>> verts)
// {
//     typename Polyhedron::Halfedge_around_facet_const_circulator h=f->facet_begin();
//     do{
//         // typename Polyhedron::Halfedge_const_handle oh=h->opposite();
//         // get vertices of the face
//         typename Polyhedron::Vertex_const_handle v = h->vertex();
//         // check if the vertex is on the verts vector
//         for (int i = 0; i < verts.size(); i++) {
//             if (v->point().x() == verts[i][0] && v->point().z() == verts[i][2]) {
//                 return true;
//             }
//         }
//         ++h;
//     }while(h!=f->facet_begin());
//     //Otherwise not subdivide this face
//     return false;
// }

template<class Items>
Vector3 Base<Items>::vertexNormal(typename Polyhedron::Vertex_const_handle v)
{
    //Area weighted vertex normal
    typename Polyhedron::Halfedge_around_vertex_const_circulator h=v->vertex_begin();
    Point3 p0=v->point();
    Vector3 Normal(0.0,0.0,0.0);
    do{
        if(!h->is_border()){
            //Access the incident faces normal, weighted by their area
            Point3 p1=h->next()->vertex()->point();
            Point3 p2=h->opposite()->vertex()->point();
            Vector3 e1(p0,p1);
            Vector3 e2(p0,p2);
            
            Normal = Normal + CGAL::cross_product(e1,e2);
        }
        ++h;
    }while(h!=v->vertex_begin());
    
    
    return normalize(Normal);
}

template<class Items>
Vector3 Base<Items>::FaceNormal(typename Polyhedron::Facet_const_handle f)
{
    typename Polyhedron::Halfedge_const_handle h=f->halfedge();
    Point3 p1=h->vertex()->point();
    Point3 p2=h->next()->vertex()->point();
    Point3 p3=h->opposite()->vertex()->point();
    return unitNormal(p1,p2,p3);
}
