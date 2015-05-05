#include "Mesh.h"

Mesh::Mesh()
{
    method = CONFLICT_GRAPH;
    randomize = true;
    seed = 0;
}

std::vector<vec3> Mesh::getTriangles() const
{
    std::vector<vec3> vList;
    for(auto it=triangles.begin(); it!=triangles.end(); it++)
    {
        vList.push_back(vertices[halfEdges[it->eIndices[0]].vIndices[0]].pos);
        vList.push_back(vertices[halfEdges[it->eIndices[1]].vIndices[0]].pos);
        vList.push_back(vertices[halfEdges[it->eIndices[2]].vIndices[0]].pos);
    }
    return vList;
}

void Mesh::clearAll()
{
    vertices.clear();
    triangles.clear();
    halfEdges.clear();
}

void Mesh::buildGhostTriangle(const std::vector<vec3>& vList)
{
    vec3 vPos[3];
    vec3 centroid(0, 0, 0);
    real radius = 0;
    for(auto it = vList.begin(); it != vList.end(); it++)
    {
        centroid += *it;
    }
    centroid /= real(vList.size());
    centroid.z = 1;

    for(auto it = vList.begin(); it != vList.end(); it++)
    {
        real dist = glm::length(*it - centroid);
        if(dist > radius)
            radius = dist;
    }

    centroid.z = 0;

    vPos[0] = centroid + vec3(-radius*2, -radius, 0);
    vPos[1] = centroid + vec3(radius*2, -radius, 0);
    vPos[2] = centroid + vec3(0, radius*2, 0);

    Vertex v;
    Triangle t;
    HalfEdge e;

    v.pos = vPos[0];
    vertices.push_back(v);

    v.pos = vPos[1];
    vertices.push_back(v);

    v.pos = vPos[2];
    vertices.push_back(v);

    halfEdges.push_back(e);
    halfEdges.push_back(e);
    halfEdges.push_back(e);

    for(int i=0; i<3; i++)
    {
        int j = (i+1)%3;
        halfEdges[i].vIndices[0] = i;
        halfEdges[i].vIndices[1] = j;
        halfEdges[i].nextIndex = j;
        halfEdges[j].prevIndex = i;
        t.eIndices[i] = i;
    }
    triangles.push_back(t);
    for(int i=0; i<3; i++)
    {
        halfEdges[i].tIndex = 0;
    }
}

template<typename T> std::vector<unsigned> lazyDeletion(std::vector<T>& v, std::vector<bool> delMap)
{
    std::vector<unsigned> indicesMap(v.size());
    unsigned k=0;
    for(unsigned i=0; i<v.size(); i++)
    {
        v[k] = v[i];
        indicesMap[i] = k;
        if(!delMap[i])
            k++;
    }
    v.resize(k);
    return indicesMap;
}

void Mesh::destroyGhostTriangle()
{
    std::vector<bool> ghostTriMap(triangles.size(), false);
    std::vector<bool> ghostEdgeMap(halfEdges.size(), false);
    for(unsigned i=0; i<triangles.size(); i++)
    {
        for(unsigned k=0; k<3; k++)
        {
            unsigned &eIndex = triangles[i].eIndices[k];
            HalfEdge &e = halfEdges[eIndex];
            if(e.vIndices[0] < 3 || e.vIndices[1] < 3)
            {
                ghostEdgeMap[eIndex] = true;
                ghostTriMap[i] = true;
            }
        }
    }
    std::vector<unsigned> indicesMap = lazyDeletion(halfEdges, ghostEdgeMap);
    lazyDeletion(triangles, ghostTriMap);
    for(unsigned i=0; i<triangles.size(); i++)
    {
        for(unsigned k=0; k<3; k++)
        {
            unsigned &eIndex = triangles[i].eIndices[k];
            eIndex = indicesMap[eIndex];
            HalfEdge &e = halfEdges[eIndex];
            e.vIndices[0] -= 3;
            e.vIndices[1] -= 3;
        }
    }

    for(unsigned i=3; i<vertices.size(); i++)
        vertices[i-3] = vertices[i];
    vertices.resize(vertices.size()-3);
}

bool Mesh::inTriangle(unsigned triIndex, const vec3& pos) const
{
    bool projCond = false;
    for(int i=0; i<3; i++)
    {
        vec3 v[2];
        const HalfEdge& e = halfEdges[triangles[triIndex].eIndices[i]];
        v[0] = vertices[e.vIndices[0]].pos;
        v[1] = vertices[e.vIndices[1]].pos;
        if(orient(v[0], v[1], pos)<0)
            return false;
        real projDist = glm::dot(pos-v[0], glm::normalize(v[1]-v[0]));
        if(projDist >= 0 && projDist < glm::length(v[1] - v[0]))
            projCond = true;
    }
    return projCond;
}

real Mesh::intersectRatio(unsigned eIndex, const vec3& searchPos, const vec3& pos) const
{
    const vec3& v0 = vertices[halfEdges[eIndex].vIndices[0]].pos;
    const vec3& v1 = vertices[halfEdges[eIndex].vIndices[1]].pos;
    vec3 dv = v1 - v0;
    vec3 dp = -pos + searchPos;
    vec3 rhs = searchPos - v0;
    real down = dv.x * dp.y - dv.y * dp.x;
    real up = rhs.x * dp.y - rhs.y * dp.x;
    if(down == 0)
        return -1;
    return up / down;
}

vec3 Mesh::getMidPoint(unsigned eIndex) const
{
    const vec3& v0 = vertices[halfEdges[eIndex].vIndices[0]].pos;
    const vec3& v1 = vertices[halfEdges[eIndex].vIndices[1]].pos;
    return (v0+v1)*0.5;
}

unsigned Mesh::locateTriangleWalking(unsigned startEdgeIndex, unsigned vIndex) const
{
    const vec3& pos = vertices[vIndex].pos;
    unsigned eIndex = startEdgeIndex;
    vec3 searchPos = getMidPoint(eIndex);
    while(!inTriangle(halfEdges[eIndex].tIndex, pos))
    {
        unsigned exitIndex = halfEdges[eIndex].nextIndex;
        real r = intersectRatio(exitIndex, searchPos, pos);
        if(r < 0 || r > 1)
        {
            exitIndex = halfEdges[exitIndex].nextIndex;
            r = intersectRatio(exitIndex, searchPos, pos);
        }

        const vec3& v0 = vertices[halfEdges[exitIndex].vIndices[0]].pos;
        const vec3& v1 = vertices[halfEdges[exitIndex].vIndices[1]].pos;
        searchPos = v0+r*(v1-v0);
        eIndex = halfEdges[exitIndex].neighborIndex;
    }
    return halfEdges[eIndex].tIndex;
}

unsigned Mesh::locateTriangle(unsigned vIndex) const
{
    if(method == CONFLICT_GRAPH)
        return vertices[vIndex].confTriIndex;
    return locateTriangleWalking(0, vIndex);
}

void Mesh::connectEdges(unsigned prevIndex, unsigned nextIndex)
{
    halfEdges[prevIndex].nextIndex = nextIndex;
    halfEdges[nextIndex].prevIndex = prevIndex;
}

void Mesh::pairEdges(unsigned eIndex0, unsigned eIndex1)
{
    halfEdges[eIndex0].neighborIndex = eIndex1;
    halfEdges[eIndex1].neighborIndex = eIndex0;
}

void Mesh::fillTriangle(unsigned tIndex, unsigned eIndex0, unsigned eIndex1, unsigned eIndex2)
{
    Triangle &t = triangles[tIndex];
    t.eIndices[0] = eIndex0;
    t.eIndices[1] = eIndex1;
    t.eIndices[2] = eIndex2;

    connectEdges(eIndex0, eIndex1);
    connectEdges(eIndex1, eIndex2);
    connectEdges(eIndex2, eIndex0);
    halfEdges[eIndex0].tIndex = tIndex;
    halfEdges[eIndex1].tIndex = tIndex;
    halfEdges[eIndex2].tIndex = tIndex;
}

bool Mesh::isDelaunay(unsigned eIndex) const
{
    const HalfEdge &e = halfEdges[eIndex];
    const HalfEdge &next = halfEdges[e.nextIndex];
    const HalfEdge &neighbor_next = halfEdges[halfEdges[e.neighborIndex].nextIndex];
    vec3 v[4];
    v[0] = vertices[e.vIndices[0]].pos;
    v[1] = vertices[e.vIndices[1]].pos;
    v[2] = vertices[next.vIndices[1]].pos;
    v[3] = vertices[neighbor_next.vIndices[1]].pos;
    int ori123 = orient(v[1], v[2], v[3]);
    int ori203 = orient(v[2], v[0], v[3]);
    if(ori123<=0 || ori203<=0)
        return true;

    if(v[0].z == 0 && (v[2].z==0 || v[3].z == 0))
        return false;

    if(v[1].z == 0 && (v[2].z==0 || v[3].z == 0))
        return false;

    if(v[0].z == 0)
    {
        return ori123<=0;
    }
    if(v[1].z == 0)
    {
        return ori203<=0;
    }
    if(inCircle(v[0], v[1], v[2], v[3])>=0)
        return false;
    return inCircle(v[0], v[3], v[1], v[2])<0;
}

void Mesh::correctEdges(std::stack<unsigned>& suspEdgeIndices, std::unordered_set<llu>& checked)
{
    while(suspEdgeIndices.size())
    {
        unsigned eIndex = suspEdgeIndices.top();
        suspEdgeIndices.pop();
        const HalfEdge &e = halfEdges[eIndex];
        if(checked.find(e.getHashValue()) != checked.end()) // Skip edges that are already checked.
            continue;
        checked.insert(e.getHashValue());
        if(e.neighborIndex == MAX_UINT) // No neighbor, must be Delaunay.
            continue;
        checked.insert(halfEdges[e.neighborIndex].getHashValue());
        if(isDelaunay(eIndex))
            continue;
        flipEdge(eIndex);
        checked.insert(e.getHashValue());
        checked.insert(halfEdges[e.neighborIndex].getHashValue());
        suspEdgeIndices.push(e.nextIndex);
        suspEdgeIndices.push(e.prevIndex);
        suspEdgeIndices.push(halfEdges[e.neighborIndex].nextIndex);
        suspEdgeIndices.push(halfEdges[e.neighborIndex].prevIndex);
    }
}

void Mesh::updateConfGraph(unsigned vIndex, unsigned tIndex)
{
    unsigned startEdgeIndex = MAX_UINT;
    for(unsigned i=0; i<3; i++)
    {
        unsigned ei = triangles[tIndex].eIndices[i];
        if(halfEdges[ei].vIndices[0] == vIndex)
            startEdgeIndex = ei;
    }

    unsigned eIndex = startEdgeIndex;

    std::unordered_set<unsigned> confVertIndices;
    do
    {
        Triangle &t = triangles[halfEdges[eIndex].tIndex];
        confVertIndices.insert(t.confVertIndices.begin(), t.confVertIndices.end());

        t.confVertIndices.clear();
        unsigned &neighborIndex = halfEdges[eIndex].neighborIndex;
        eIndex = halfEdges[neighborIndex].nextIndex;
    }
    while(eIndex != startEdgeIndex);

    confVertIndices.erase(vIndex);

    for(auto it = confVertIndices.begin(); it != confVertIndices.end(); it++)
    {
        do
        {
            unsigned triIndex = halfEdges[eIndex].tIndex;
            if(inTriangle(triIndex, vertices[*it].pos))
            {
                vertices[*it].confTriIndex = triIndex;
                triangles[triIndex].confVertIndices.insert(*it);
                eIndex = startEdgeIndex;
                break;
            }
            unsigned &neighborIndex = halfEdges[eIndex].neighborIndex;
            eIndex = halfEdges[neighborIndex].nextIndex;
        }
        while(eIndex != startEdgeIndex);
    }
}

void Mesh::insertVertex(unsigned vIndex)
{
    std::unordered_set<llu> checked; // set of checked edges (Hash Values)
    std::stack<unsigned> suspEdgeIndices; // stack of suspicious edges
    HalfEdge e;

    unsigned oldEdgeIndices[3];

    unsigned eIndexList[3][2];

    unsigned oldTriIndex = locateTriangle(vIndex);

    for(int i=0; i<3; i++)
    {
        unsigned eIndex = triangles[oldTriIndex].eIndices[i];
        HalfEdge &oldEdge = halfEdges[eIndex];
        suspEdgeIndices.push(eIndex); // Edges of the old triangle are suspicious.

        oldEdgeIndices[i] = eIndex;

        // new edge
        e.vIndices[0] = oldEdge.vIndices[1];
        e.vIndices[1] = vIndex;
        halfEdges.push_back(e);
        eIndexList[i][0] = unsigned(halfEdges.size())-1;

        // new edge
        e.vIndices[1] = oldEdge.vIndices[1];
        e.vIndices[0] = vIndex;
        halfEdges.push_back(e);
        eIndexList[i][1] = unsigned(halfEdges.size())-1;
    }

    for(int i=0; i<3; i++)
    {
        unsigned eIndex = oldEdgeIndices[i];
        int prev_i = (i+2)%3;

        unsigned tIndex = unsigned(triangles.size());
        if(i > 0)
            triangles.push_back(Triangle());
        else
            tIndex = oldTriIndex;

        fillTriangle(tIndex, eIndex, eIndexList[i][0], eIndexList[prev_i][1]);
        pairEdges(eIndexList[i][0], eIndexList[i][1]);
        checked.insert(halfEdges[eIndexList[i][0]].getHashValue());
        checked.insert(halfEdges[eIndexList[i][1]].getHashValue());
    }

    correctEdges(suspEdgeIndices, checked);

    if(method == CONFLICT_GRAPH) // redistribute
    {
        updateConfGraph(vIndex, oldTriIndex);
    }
}

void Mesh::flipEdge(unsigned eIndex)
{
    HalfEdge &e = halfEdges[eIndex];
    unsigned eIndexList[5];
    eIndexList[0] = e.nextIndex;
    eIndexList[1] = e.prevIndex;
    eIndexList[2] = e.neighborIndex;
    eIndexList[3] = halfEdges[e.neighborIndex].nextIndex;
    eIndexList[4] = halfEdges[e.neighborIndex].prevIndex;

    e.vIndices[0] = halfEdges[eIndexList[3]].vIndices[1];
    e.vIndices[1] = halfEdges[eIndexList[1]].vIndices[0];
    halfEdges[eIndexList[2]].vIndices[0] = e.vIndices[1];
    halfEdges[eIndexList[2]].vIndices[1] = e.vIndices[0];
    fillTriangle(e.tIndex, eIndex, eIndexList[1], eIndexList[3]);
    fillTriangle(halfEdges[e.neighborIndex].tIndex, eIndexList[2], eIndexList[4], eIndexList[0]);
}

std::vector<vec3> Mesh::loadNodes(const char* nodePath)
{
    glm::ivec4 h;
    std::vector<vec3> vList;
    FILE* file = fopen(nodePath, "r");
    if(fscanf(file, "%d %d %d %d", &h.x, &h.y, &h.z, &h.w) != 4)
    {
        fclose(file);
        return vList;
    }
    int nDims = h.y;
    glm::dvec4 v;
    while(1)
    {
        if(nDims == 2 && fscanf(file, "%d %lf %lf", &h.x, &v.x, &v.y) != 3)
            break;
        if(nDims == 3 && fscanf(file, "%d %lf %lf %lf", &h.x, &v.x, &v.y, &v.z) != 4)
            break;
        vList.push_back(vec3(v.x, v.y, 1));
    }
    fclose(file);
    return vList;
}

void Mesh::outputNodes(const char* nodePath)
{
    glm::ivec4 h;
    std::vector<vec3> vList;
    FILE* file = fopen(nodePath, "w");
    fprintf(file, "%u 2 0 0\n", unsigned(vertices.size()));
    for(unsigned i=0; i<vertices.size(); i++)
    {
        vec3 &v = vertices[i].pos;
        fprintf(file, "%u %lf %lf\n", i+1, v.x, v.y);
    }
    fclose(file);
}

void Mesh::outputElements(const char* elePath)
{
    glm::ivec4 h;
    std::vector<vec3> vList;
    FILE* file = fopen(elePath, "w");
    fprintf(file, "%u 3 0\n", unsigned(triangles.size()));
    for(unsigned i=0; i<triangles.size(); i++)
    {
        unsigned vi[3];
        for(unsigned k=0; k<3; k++)
            vi[k] = halfEdges[triangles[i].eIndices[k]].vIndices[0]+1;
        fprintf(file, "%u %u %u %u\n", i+1, vi[0], vi[1], vi[2]);
    }
    fclose(file);
}

unsigned randu(unsigned& seed)
{
    unsigned m_z = seed >> 16;
    unsigned m_w = seed & 0xffff;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    seed = (m_z << 16) + m_w;
    return seed;
}

void shuffle(std::vector<vec3>& vList, unsigned& seed)
{
    for(unsigned i=0; i<vList.size(); i++)
    {
        std::swap(vList[i], vList[randu(seed)%vList.size()]);
    }
}

void Mesh::delaunay(const std::vector<vec3>& v)
{
    std::vector<vec3> vList = v;

    if(randomize)
        shuffle(vList, seed);

    clearAll();
    buildGhostTriangle(vList);

    for(unsigned i=0; i<vList.size(); i++)
    {
        vertices.push_back(Vertex());
        vertices.back().pos = vList[i];
        if(method == CONFLICT_GRAPH)
        {
            vertices.back().confTriIndex = 0;
            triangles.front().confVertIndices.insert(i+3);
        }
    }

    for(unsigned i=0; i<vList.size(); i++)
    {
        insertVertex(i+3);
        //if(i==2)
        //break;
    }

    destroyGhostTriangle();
}
