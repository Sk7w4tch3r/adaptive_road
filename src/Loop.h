#pragma once
#include "Base.h"
#include <map>
#include <set>
class Loop:public Base<CGAL::Polyhedron_items_3>{
public:
    Loop();
    ~Loop(){}
    
    void setSelectedVerts(const std::vector<std::vector<float>>& verts)
    {
        // loop over all faces and if a face contains a vertex in the selectedVerts, store it in the vertexBuffer
        for(auto f = mesh.facets_begin(); f != mesh.facets_end(); ++f)
        {
            typename Polyhedron::Halfedge_around_facet_circulator h = f->facet_begin();
            do{
                typename Polyhedron::Vertex_handle v = h->vertex();
                for(auto vert : verts)
                {
                    if(v->point().x() == vert[0] && v->point().z() == vert[2])
                    {
                        vertexBuffer.insert(std::make_pair(v, v->point()));
                    }
                }
                ++h;
            }while(h != f->facet_begin());
        }
    }
    
    void adaptive();
    Vector3 FaceNormal(typename Polyhedron::Facet_const_handle f);

private:
    
    double computeBeta(Polyhedron::Vertex_const_handle v, double n)
    {
        return ((0.625-std::pow(0.375+0.25*std::cos(2.0*M_PI/n),2))/n);
    }
    
    Point3 vertexPosition(Polyhedron::Vertex_const_handle v);
    Point3 computeEdgeVertex(Polyhedron::Halfedge_const_handle h);
    void storeVertex(Polyhedron::Facet_handle f);
    std::map<Polyhedron::Vertex_handle, Point3> vertexBuffer; //Vertex buffer
    bool checkSubdivide(typename Polyhedron::Facet_handle f);
};
