#include "MeshGraph.h"
#include "BinaryHeap.h"
// For printing
#include <fstream>
#include <iostream>
#include <sstream>

MeshGraph::MeshGraph(const std::vector<Double3>& vertexPositions,
                     const std::vector<IdPair>& edges)
{
        vertices.resize(vertexPositions.size());
        for (int i = 0; i < vertexPositions.size(); i++) {
            vertices[i].id = i;
            vertices[i].position3D = vertexPositions[i];
        }
        
        adjList.resize(vertices.size());
        
        for (std::vector<IdPair>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
            IdPair edge = *it;
            int v0 = edge.vertexId0;
            int v1 = edge.vertexId1;
            adjList[v0].push_back(&vertices[v1]);
            adjList[v1].push_back(&vertices[v0]);
        }
        
    }


double MeshGraph::AverageDistanceBetweenVertices() const
{
    // TODO:
    
        double total_distance = 0;
        int edges = 0;
        for (int u = 0; u < vertices.size(); u++) {
            for (std::list<Vertex*>::const_iterator it = adjList[u].begin(); it != adjList[u].end(); it++) {
                Vertex* v = *it;
                double distance = Double3::Distance(vertices[u].position3D, v->position3D);
                total_distance += distance;
                edges++;
            }
        }
        return total_distance / edges;
    }



double MeshGraph::AverageEdgePerVertex() const
{
    // TODO:
    double vertexCount = TotalVertexCount();
    double edgeCount = TotalEdgeCount();
    double result = edgeCount/vertexCount;
    return result;
}

int MeshGraph::TotalVertexCount() const
{
    // TODO:
    return vertices.size();
}

int MeshGraph::TotalEdgeCount() const
{
    // TODO:
    int edges = 0;
        for (int i = 0; i < vertices.size(); i++) {
            for (std::list<Vertex*>::const_iterator it = adjList[i].begin(); it != adjList[i].end(); it++) {
                edges++;
            }
        }
        return edges/2;
    
}

int MeshGraph::VertexEdgeCount(int vertexId) const
{
    // TODO:
    bool check = false;
    int i = 0;
    for(i=0;i<vertices.size();i++){
        if(vertices[i].id == vertexId){
            check = true;
            break;}
    }
    if(check){
        int vertexEdgeCount = 0;
        for (std::list<Vertex*>::const_iterator it = adjList[i].begin(); it != adjList[i].end(); it++) {
                vertexEdgeCount++;
            }
        return vertexEdgeCount;
    }
    return -1;
    
}

void MeshGraph::ImmediateNeighbours(std::vector<int>& outVertexIds,
                                    int vertexId) const
{
    // TODO:
    outVertexIds.clear();
    bool check = false;
    int i = 0;
    for(i=0;i<vertices.size();i++){
        if(vertices[i].id == vertexId){
            check = true;
            break;}
    }
    
    if(check == false){
        return;}
    
    else{
    for (std::list<Vertex*>::const_iterator it = adjList[i].begin(); it != adjList[i].end(); it++) {
                int tempId = (*it)->id;
                outVertexIds.push_back(tempId);
                
                
            }
            return;
    }
}


void MeshGraph::PaintInBetweenVertex(std::vector<Color>& outputColorAllVertex,
                                     int vertexIdFrom, int vertexIdTo,
                                     const Color& color) const
{    outputColorAllVertex.clear();
    if(0 > vertexIdFrom || vertexIdFrom >= TotalVertexCount()) return;
    if(0 > vertexIdTo || vertexIdTo >= TotalVertexCount())  return;
    
    outputColorAllVertex.resize(TotalVertexCount());
    for(int i=0;i<TotalVertexCount();i++){
        outputColorAllVertex[i].r = 0;
        outputColorAllVertex[i].g = 0;
        outputColorAllVertex[i].b = 0;
    }
    std::vector<double> distances(vertices.size(), INFINITY);
    std::vector<int> previous(vertices.size(), -1);
    std::vector<bool> visited(vertices.size(), false);
    BinaryHeap heap;
    //yeni eklediklerim:
    for(int w=0;w<TotalVertexCount();w++){
        heap.Add(w,INFINITY);
    }
    heap.ChangePriority(vertexIdFrom,0);
   
   
    distances[vertexIdFrom] = 0;
    while (heap.HeapSize() > 0)
    {
       
        int u;
        double w;
        heap.PopHeap(u, w);
        visited[u] = true;
        
        if (u == vertexIdTo)
        {
            break;
        }
        
        std::list<Vertex*> neighbours = adjList[u];
        for (std::list<Vertex*>::iterator it = neighbours.begin(); it != neighbours.end(); ++it)
        {
            if (!visited[(*it)->id])
            {
        
                double newDistance = distances[u] + Double3::Distance(vertices[u].position3D, (*it)->position3D);
                
                if (newDistance < distances[(*it)->id])
                {
                    distances[(*it)->id] = newDistance;
                    previous[(*it)->id] = u;
			        heap.ChangePriority((*it)->id,newDistance);
                }
            }
        }
    }
  
    std::vector<int> path;
    int currentVertex = vertexIdTo;
    while (currentVertex != -1)
    {
        path.push_back(currentVertex);
        currentVertex = previous[currentVertex];
    }
    if(path.size() == 1)
        return;
    // for reversing path:
    std::vector<int> temp;
    int path_size = path.size();
    for(int y=0;y<path_size;y++){
        int var = path.back();
        temp.push_back(var);
        path.pop_back();
    }
    for (std::vector<int>::iterator it = temp.begin(); it != temp.end(); ++it)
    {
       
        outputColorAllVertex[*it].r = color.r;
        outputColorAllVertex[*it].g = color.g;
        outputColorAllVertex[*it].b = color.b;
   
    }
    return;
}



void MeshGraph::PaintInRangeGeodesic(std::vector<Color>& outputColorAllVertex,
                                    int vertexId, const Color& color,
                                    int maxDepth, FilterType type,
                                    double alpha) const
{
    outputColorAllVertex.clear();
   if(0 > vertexId || vertexId >= TotalVertexCount()) return;
   
    int NewWeight = 0;
    
    
    outputColorAllVertex.resize(TotalVertexCount());
    for(int i=0;i<TotalVertexCount();i++){
        outputColorAllVertex[i].r = 0;
        outputColorAllVertex[i].g = 0;
        outputColorAllVertex[i].b = 0;
    }
    std::vector<double> distances(vertices.size(), INFINITY);
    std::vector<bool> visited(vertices.size(), false);
    std::vector<double> real_distances(vertices.size(), INFINITY);
    BinaryHeap heap;
    heap.Add(vertexId,NewWeight++);
    visited[vertexId] = true;
    distances[vertexId] = 0;
    real_distances[vertexId] = 0;
    while(heap.HeapSize()>0){
        int u;
        double w;
        heap.PopHeap(u,w);
        outputColorAllVertex[u].r = color.r; 
        outputColorAllVertex[u].g = color.g; 
        outputColorAllVertex[u].b = color.b;
        
        if(type == FILTER_GAUSSIAN){
            double constant;
            double distance = real_distances[u];
            double distancesq = distance*distance;
            constant = std::exp(-distancesq/(alpha*alpha));
            outputColorAllVertex[u].r *= constant; 
            outputColorAllVertex[u].g *= constant; 
            outputColorAllVertex[u].b *= constant;
        }
        
        else if(type == FILTER_BOX){
            double constant;
            double distance = real_distances[u];
            if(-alpha<=distance && distance<=alpha)
                constant = 1;
            else
                constant = 0;
            outputColorAllVertex[u].r *= constant; 
            outputColorAllVertex[u].g *= constant; 
            outputColorAllVertex[u].b *= constant;
        }
        
        if (distances[u] < maxDepth) {
            for (std::list<Vertex*>::const_iterator it = adjList[u].begin(); it != adjList[u].end(); it++) {
                if (!visited[(*it)->id]) {
                    heap.Add((*it)->id, NewWeight++);
                    visited[(*it)->id] = true;
                    
                    distances[(*it)->id] = distances[u] + 1;
                    if(real_distances[(*it)->id] == INFINITY){
                        real_distances[(*it)->id] = real_distances[u] + Double3::Distance(vertices[u].position3D, (*it)->position3D);
                    }
                }
            }
        }
        
    }
}


void MeshGraph::PaintInRangeEuclidian(std::vector<Color>& outputColorAllVertex,
                                      int vertexId, const Color& color,
                                      int maxDepth, FilterType type,
                                      double alpha) const
{
    outputColorAllVertex.clear();
    // TODO:
    if(0 > vertexId || vertexId >= TotalVertexCount()) return;
    
    
    outputColorAllVertex.resize(TotalVertexCount());
    for(int i=0;i<TotalVertexCount();i++){
        outputColorAllVertex[i].r = 0;
        outputColorAllVertex[i].g = 0;
        outputColorAllVertex[i].b = 0;
    }
    std::vector<double> distances(vertices.size(), INFINITY);
    std::vector<bool> visited(vertices.size(), false);
    BinaryHeap heap;
    heap.Add(vertexId,0);
    visited[vertexId] = true;
    distances[vertexId] = 0;
    while(heap.HeapSize()>0){
        int u;
        double w;
        heap.PopHeap(u,w);
        outputColorAllVertex[u].r = color.r; 
        outputColorAllVertex[u].g = color.g; 
        outputColorAllVertex[u].b = color.b;
        if(type == FILTER_GAUSSIAN){
            double constant;
            double distance = Double3::Distance(vertices[vertexId].position3D,vertices[u].position3D);
            double distancesq = distance*distance;
            constant = std::exp(-distancesq/(alpha*alpha));
            outputColorAllVertex[u].r *= constant; 
            outputColorAllVertex[u].g *= constant; 
            outputColorAllVertex[u].b *= constant;
        }
        else if(type == FILTER_BOX){
            double constant;
            double distance = Double3::Distance(vertices[vertexId].position3D,vertices[u].position3D);
            if(-alpha<=distance && distance<=alpha)
                constant = 1;
            else
                constant = 0;
            outputColorAllVertex[u].r *= constant; 
            outputColorAllVertex[u].g *= constant; 
            outputColorAllVertex[u].b *= constant;
        }
        if (distances[u] < maxDepth) {
            for (std::list<Vertex*>::const_iterator it = adjList[u].begin(); it != adjList[u].end(); it++) {
                if (!visited[(*it)->id]) {
                    heap.Add((*it)->id, distances[u] + 1);
                    visited[(*it)->id] = true;
                    distances[(*it)->id] = distances[u] + 1;
                }
            }
        }
        
    }
}

void MeshGraph::WriteColorToFile(const std::vector<Color>& colors,
                                 const std::string& fileName)
{
    // IMPLEMENTED
    std::stringstream s;
    for(int i = 0; i < static_cast<int>(colors.size()); i++)
    {
        int r = static_cast<int>(colors[i].r);
        int g = static_cast<int>(colors[i].g);
        int b = static_cast<int>(colors[i].b);

        s << r << ", " << g << ", " << b << "\n";
    }
    std::ofstream f(fileName.c_str());
    f << s.str();
}

void MeshGraph::PrintColorToStdOut(const std::vector<Color>& colors)
{
    // IMPLEMENTED
    for(int i = 0; i < static_cast<int>(colors.size()); i++)
    {
        std::cout << static_cast<int>(colors[i].r) << ", "
                  << static_cast<int>(colors[i].g) << ", "
                  << static_cast<int>(colors[i].b) << "\n";
    }
}