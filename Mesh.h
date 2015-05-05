#ifndef MESH_H
#define MESH_H

#include "macros.h"
#include <stdio.h>
#include <stack>
#include <unordered_set>
#include <string>
#include <vector>

enum Method{ WALKING, CONFLICT_GRAPH };

typedef long long unsigned llu;

#define MAX_UINT 0xffffffff

inline int orient(const vec3& a, const vec3& b, const vec3& c)
{
    vec3 da = a - c;
    vec3 db = b - c;
    real det = da.x * db.y - db.x * da.y;
    return det == 0 ? 0 : int(det > 0)*2-1;
}

inline int inCircle(const vec3& a, const vec3& b, const vec3& c, const vec3& d)
{
    vec3 da, db, dc;
    real det;

    if(orient(a, b, c) == 0)
        return 1;

    if(d.z == 0)
        return -1;
    if(b.z == 0)
        return -1;

    glm::dmat3 mat;
    da = a - d;
    db = b - d;
    dc = c - d;
    mat[0][0] = da.x;
    mat[1][0] = da.y;
    mat[2][0] = da.x*da.x+da.y*da.y;
    mat[0][1] = db.x;
    mat[1][1] = db.y;
    mat[2][1] = db.x*db.x+db.y*db.y;
    mat[0][2] = dc.x;
    mat[1][2] = dc.y;
    mat[2][2] = dc.x*dc.x+dc.y*dc.y;
    det = glm::determinant(mat);
    return det == 0 ? 0 : int(det > 0)*2-1;
}

class Mesh
{
public:
    struct Vertex;
    struct HalfEdge;
    struct Triangle;
    struct Vertex
    {
        unsigned confTriIndex;
        vec3 pos;
    };
    struct Triangle
    {
        unsigned eIndices[3];
        std::unordered_set<unsigned> confVertIndices;
        Triangle()
        {
            eIndices[0] = eIndices[1] = eIndices[2] = MAX_UINT;
        }
    };
    struct HalfEdge
    {
        unsigned vIndices[2];
        unsigned tIndex;
        unsigned nextIndex, prevIndex, neighborIndex;
        HalfEdge()
        {
            vIndices[0] = vIndices[1] = MAX_UINT;
            tIndex = nextIndex = prevIndex = neighborIndex = MAX_UINT;
        }
        llu getHashValue() const
        {
            return (llu(vIndices[0]) << 32) + vIndices[1];
        }
    };
    Mesh();
    void setMethod(Method method) { this->method = method; }
    std::vector<vec3> getTriangles() const;
    std::vector<vec3> loadNodes(const char* nodePath);
    void delaunay(const std::vector<vec3>& v);
    void outputNodes(const char* nodePath);
    void outputElements(const char* elePath);
    void setRandomizeFlag(bool r) { randomize = r; }
    void setSeed(unsigned s) { seed = s; }
protected:
    unsigned seed;
    bool randomize;
    Method method;
    bool isDelaunay(unsigned eIndex) const;
    void flipEdge(unsigned eIndex);
    real intersectRatio(unsigned eIndex, const vec3& searchPos, const vec3& pos) const;
    void correctEdges(std::stack<unsigned>& suspEdgeIndices, std::unordered_set<llu>& checked);
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    std::vector<HalfEdge> halfEdges;
    vec3 getMidPoint(unsigned eIndex) const;
    bool inTriangle(unsigned triIndex, const vec3& pos) const;
    unsigned locateTriangleWalking(unsigned startEdgeIndex, unsigned vIndex) const;
    unsigned locateTriangle(unsigned vIndex) const;// returns triangle index
    void updateConfGraph(unsigned vIndex, unsigned tIndex);
    void clearAll();
    void buildGhostTriangle(const std::vector<vec3>& vList);
    void destroyGhostTriangle();
    void insertVertex(unsigned vIndex);
    void connectEdges(unsigned prevIndex, unsigned nextIndex);
    void pairEdges(unsigned eIndex0, unsigned eIndex1);
    void fillTriangle(unsigned tIndex, unsigned eIndex0, unsigned eIndex1, unsigned eIndex2);
};

#endif // MESH_H
