#include "Loop.h"
#include <unistd.h>
#include <limits>


template<class T>
void process(std::string input, std::string output, T& mesh, const int& level, std::vector<std::vector<float>> verts)
{
    mesh.loadMesh(input);
    mesh.setSelectedVerts(verts);
    
    int Level = 0;

    while(Level < level)
    {
        std::size_t before=mesh.size_of_faces();

        mesh.adaptive();
        ++Level;
    }
    std::cout<<"Maximum faces: "<<mesh.size_of_faces()<<"\n";
    mesh.saveMesh(output);
}

std::vector<std::vector<float>> readVerts(std::string path){
    std::ifstream file(path);
    std::vector<std::vector<float>> verts;
    std::string line;
    while(std::getline(file,line)){
        std::vector<float> vert;
        std::stringstream ss(line);
        std::string token;
        while(std::getline(ss,token,' ')){
            vert.push_back(std::stof(token));
        }
        verts.push_back(vert);
    }
    return verts;
}


int main(int argc,char** argv)
{    
    int level= 4;
    std::string terrainMeshName;
    std::string sketchName;
    std::string input;
    std::string output;
    std::string vertsPath;
    int enableVis = 0;
    
    terrainMeshName = argv[1];
    sketchName = argv[2];
    level = std::stoi(argv[3]);
    enableVis = std::stoi(argv[4]);
    input = "output\\terrain_" + sketchName + ".off";
    output = "output\\subdivided\\terrain_" + sketchName + "_" + std::to_string(level) + ".off";
    vertsPath = "output\\boundary_verts_" + sketchName + ".txt";

    // system call to python code for insertion of vertices
    std::string command = std::string("py insertion\\main.py ") + std::string("--terrain_mesh ") + terrainMeshName + std::string(" --sketch_name ")  + sketchName;
    int result = system(command.c_str());
    if (result != 0)
    {
        std::cout << "Error in python code" << std::endl;
        return 1;
    } else {
        std::vector<std::vector<float>> verts = readVerts(vertsPath);
        Loop loop;  
        process(input, output, loop, level, verts);
    }

    if (enableVis){        
        std::string command = "py insertion\\vis.py --vis_target " + output;
        int result = system(command.c_str());
    }
    return 0;
}
