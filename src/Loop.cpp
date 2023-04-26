#include "Loop.h"

Loop::Loop(){}


void Loop::storeVertex(Polyhedron::Facet_handle f)
{
    Polyhedron::Halfedge_around_facet_circulator h=f->facet_begin();
    do{
        Polyhedron::Vertex_handle v=h->vertex();
        
        //Buffer new vertex position
        Point3 newPos=vertexPosition(v);
        vertexBuffer.insert(std::pair<Polyhedron::Vertex_handle,Point3>(v,newPos));

        ++h;
    }while(h!=f->facet_begin());
}

bool Loop::checkSubdivide(Polyhedron::Facet_handle f)
{
    Polyhedron::Halfedge_around_facet_circulator h=f->facet_begin();
    // check if any of the face's vertices is in the vertexBuffer
    do{
        // get vertices of the face
        Polyhedron::Vertex_handle v = h->vertex();
        // check if the vertex is on the vertexBuffer
        if (vertexBuffer.find(v) != vertexBuffer.end()) {
            return true;
        }
        ++h;
    }while(h!=f->facet_begin());
    //Otherwise not subdivide this face
    return false;
}


Point3 Loop::computeEdgeVertex(Polyhedron::Halfedge_const_handle h)
{
    if(h->is_border_edge())
        return 0.5*h->vertex()->point()+0.5*h->opposite()->vertex()->point();
    
    return  3.0/8.0*h->vertex()->point()+
            3.0/8.0*h->opposite()->vertex()->point()+
            1.0/8.0*h->next()->vertex()->point()+
            1.0/8.0*h->opposite()->next()->vertex()->point();
}


Point3 Loop::vertexPosition(Polyhedron::Vertex_const_handle v)
{
    //Get the valence
    double n=static_cast<double>(v->vertex_degree());
    
    //Compute coefficient beta
    double beta = computeBeta(v, n);
    
    //Denote if this vertex is on border
    bool onBorder=false;
    
    //Interior vertex mask
    Point3 P1((1-n*beta)*v->point());
    
    //Border vertex mask
    Point3 P2(3.0/4.0*v->point());
    
    Polyhedron::Halfedge_around_vertex_const_circulator h=v->vertex_begin();

    do{
        //Interior vertex mask
        P1 = P1 + beta*h->opposite()->vertex()->point();
        
        //Border vertex mask, set onBorder true
        if(h->is_border_edge()){
            P2 = P2 + (1.0/8.0)*h->opposite()->vertex()->point();
            onBorder=true;
        }
        
        ++h;
    }while(h!=v->vertex_begin());
    
    //If it is boder vertex, return P2, otherwise return P1
    if(onBorder){
        return P2;
    }else{
        return P1;
    }
}


void Loop::adaptive()
{
    std::size_t nv=mesh.size_of_vertices();
    std::size_t nf=mesh.size_of_facets();
    std::size_t ne=mesh.size_of_halfedges();
    
    //Reserve memory
    mesh.reserve(nv+ne, 2*ne+6*nf, 4*nf);
    
    std::vector<std::pair<Polyhedron::Halfedge_handle,Point3>> newEdgeVertices; //Edge vertex buffer
    std::vector<Polyhedron::Halfedge_handle> newHalfedges;//Halfedge buffer, used to split faces

    std::map<Polyhedron::Vertex_handle, Point3> tempBuffer; //Buffer for new vertex position
    
    //For each edge...
    for(Polyhedron::Edge_iterator edgeIter=mesh.edges_begin();
        edgeIter!=mesh.edges_end();++edgeIter)
    {
        //Denote if two incident faces should be subdivided
        bool subdivideF1=false;
        bool subdivideF2=false;
        
        //Get two incident faces
        Polyhedron::Facet_handle f1=edgeIter->facet();
        Polyhedron::Facet_handle f2=edgeIter->opposite()->facet();
        
        
        if(f1!=Polyhedron::Facet_handle()) subdivideF1=checkSubdivide(f1);   

        if(f2!=Polyhedron::Facet_handle()) subdivideF2=checkSubdivide(f2);
        
        
        //If either one or two incident faces should be divided, this edge should be split
        if(subdivideF1 || subdivideF2 )
        {
            //Compute edge position
            Point3 edgeV=computeEdgeVertex(edgeIter);
            //Add this edge and vertex position to buffer
            newEdgeVertices.emplace_back(edgeIter,edgeV);
            //add new vertex to the vertex buffer
            tempBuffer.insert(std::pair<Polyhedron::Vertex_handle,Point3>(edgeIter->vertex(),edgeV));
        }
    }
    
    for (auto ListIter = tempBuffer.begin(); ListIter != tempBuffer.end(); ++ListIter) {
        vertexBuffer.insert(std::pair<Polyhedron::Vertex_handle,Point3>(ListIter->first,ListIter->second));
    }
    tempBuffer.clear();

    
    //For each edge which will be split
    for(auto ListIter=newEdgeVertices.begin();ListIter!=newEdgeVertices.end();++ListIter)
    {
        
        Polyhedron::Halfedge_handle h=ListIter->first;
        //Split halfedge
        mesh.split_edge(h);
        
        //Assign edge vertex position
        h->opposite()->vertex()->point()=ListIter->second;
        
        //Now we have four halfedges, add the two which are not point to the new edge vertex to buffer which will be used to split face
        if(!h->is_border())
            newHalfedges.push_back(h);
        if(!h->prev()->opposite()->is_border())
            newHalfedges.push_back(h->prev()->opposite());
    }

    // add t-junctions
    for(auto ListIter=newHalfedges.begin();ListIter!=newHalfedges.end();++ListIter)
    {
        //split faces
        Polyhedron::Halfedge_handle h=*ListIter;
        assert(h!=Polyhedron::Halfedge_handle());
        mesh.split_facet(h->prev(), h->next());
    }
}

