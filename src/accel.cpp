/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <chrono>

extern int maxOctreeDepth;


NORI_NAMESPACE_BEGIN

#define USE_OCTREE 1

struct OctPrimitive
{
    const Mesh* mesh;
    std::vector<size_t> triangles;
};

struct OctNode
{
    uint32_t depth;
    BoundingBox3f box;
    uint32_t children;
    
    std::vector<OctPrimitive> primitives;

    bool isValid = false;
};

class Octree
{
public:
    void Build(std::vector<const Mesh*>& meshes, const BoundingBox3f& box, uint32_t maxDepth, uint32_t maxPrimitiveCount);
    void Clean();

    bool RayIntersect(Ray3f &ray, Intersection &its, uint32_t& primitiveIndex, bool shadowRay) const;

protected:
    void AddMesh(OctNode* node, const Mesh* mesh);
    bool AddPrimitive(OctNode* node, const Mesh* mesh, uint32_t index);
    void BornChildren(OctNode* node);

    bool TraverseNode(const OctNode* node, Ray3f& ray, Intersection& its, uint32_t& primitiveIndex, bool shadowRay) const;
    //void TraverseChildren(OctNode* node, )

    bool CheckNode(OctNode* node, uint32_t &primitiveCount);

protected:
    std::vector<const Mesh*> m_meshes;
    std::vector<OctNode> m_nodes;
    uint32_t m_maxDepth = 0;
    uint32_t m_maxPrimitiveCount = 0;
};

std::map<const Mesh*, std::vector<bool>> g_usedPrimitives;

bool Octree::CheckNode(OctNode* node, uint32_t &primitiveCount)
{
    assert(node->children == 0 || node->primitives.empty());
    if(node->children > 0 && !node->primitives.empty())
    {
        std::cout << "[Octree::CheckNode] invalid node. depth: " << node->depth <<
                ", children: " << node->children <<
                ", primitives: " << node->primitives.size() << "\n";
        return false;
    }

    node->isValid = false;

    if(node->primitives.empty())
    {
        if(node->children > 0)
        {
            for(uint32_t childIndex = node->children, maxChild = childIndex + 8; childIndex < maxChild; ++childIndex)
            {
                if(CheckNode(&m_nodes[childIndex], primitiveCount))
                {
                    node->isValid = true;
                }
            }
        }
    }
    else
    {
        node->isValid = true;
        for(auto primitive: node->primitives)
        {
            primitiveCount += primitive.triangles.size();
            auto& triangles = g_usedPrimitives[primitive.mesh];
            for(auto index: primitive.triangles)
            {
                triangles[index] = true;
            }
        }
    }
    
    return node->isValid;
}

void Octree::Build(std::vector<const Mesh*>& meshes, const BoundingBox3f& box, uint32_t maxDepth, uint32_t maxPrimitiveCount)
{
    if(meshes.empty())
    {
        return;
    }

    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    
    Clean();

    m_meshes = meshes;
    m_maxDepth = maxDepth;
    m_maxPrimitiveCount = maxPrimitiveCount;

    m_nodes.reserve(1 + std::pow(2, (3 * (maxDepth + 1))));
    m_nodes.push_back(OctNode{0, box, 0});
    OctNode* root = m_nodes.data();

    for(size_t i = 0; i < m_meshes.size(); ++i)
    {
        AddMesh(root, m_meshes[i]);
    }
    
    //会导致内存重新分配
    m_nodes.shrink_to_fit();
    auto end = high_resolution_clock::now();
    std::cout << "[Octree::Build] cost time of build octree: " << duration_cast<milliseconds>(end - start).count() << "ms.\n";

    g_usedPrimitives.clear();
    uint32_t primitiveCount = 0;
    std::for_each(m_meshes.begin(), m_meshes.end(), [&primitiveCount](const Mesh* mesh)
    {
        primitiveCount += mesh->getTriangleCount();
        g_usedPrimitives.insert(std::make_pair(mesh, std::vector<bool>(mesh->getTriangleCount(), false)));
    });
    
    uint32_t buildPrimitiveCount = 0;
    CheckNode(m_nodes.data(), buildPrimitiveCount);
    std::cout << "[Octree::Build] cost time of check octree: " << duration_cast<milliseconds>(high_resolution_clock::now() - end).count() << "ms.\n";

    std::cout << "[Octree::Build] octree nodes count: " << m_nodes.size() << "\n";
    std::cout << "[Octree::Build] original primitive count: " << primitiveCount << ", build primitive count: " << buildPrimitiveCount << "\n";
    for (auto primitive : g_usedPrimitives)
    {
        for (size_t index = 0; index < primitive.second.size(); ++index)
        {
            if (!primitive.second[index])
            {
                std::cout << "[Octree::Build] missed primitive: " << primitive.first->getName().c_str() << ", " << index << "\n";
            }
        }
    }
}

void Octree::Clean()
{
    m_meshes.clear();
    m_nodes.clear();
    m_maxDepth = 0;
    m_maxPrimitiveCount = 0;
}

bool Octree::TraverseNode(const OctNode* node, Ray3f& ray, Intersection& its, uint32_t& primitiveIndex, bool shadowRay) const
{
    bool ret = false;

     if (!node->isValid || !node->box.rayIntersect(ray))
     {
         return ret;
     }

    if (node->primitives.empty())
    {
        if(node->children > 0)
        {
            std::pair<uint32_t, float> children[8] = {};
            for (uint32_t childIndex = node->children, maxChild = childIndex + 8; childIndex < maxChild; ++childIndex)
            {
                children[childIndex - node->children] = std::make_pair(childIndex, m_nodes[childIndex].box.distanceTo(ray.o));
            }

            //按距离排序
            std::sort(children, children + 8, [](const auto& l, const auto& r) { return l.second < r.second; });

            for(const auto& child: children)
            {
                // if(child.second >= ray.maxt)
                // {
                //     break;
                // }
                
                if (TraverseNode(&m_nodes[child.first], ray, its, primitiveIndex, shadowRay))
                {
                    // if (shadowRay)
                    // {
                    //     return true;
                    // }
                    //

                    return true;
                }
            }
            
             //for (uint32_t childIndex = node->children, maxChild = node->children + 8; childIndex < maxChild; ++childIndex)
             //{
             //    if (TraverseNode(&m_nodes[childIndex], ray, its, primitiveIndex, shadowRay))
             //    {
             //        if (shadowRay)
             //        {
             //            return true;
             //        }
            
             //        ret = true;
             //    }
             //}
        }
    }
    else
    {
        for(auto& primitive: node->primitives)
        {
            if(primitive.mesh >= 0 && !primitive.triangles.empty())
            {
                for (auto index : primitive.triangles)
                {
                    float u, v, t;
                    if (primitive.mesh->rayIntersect(index, ray, u, v, t))
                    {
                        if (shadowRay)
                        {
                            return true;
                        }

                        ray.maxt = its.t = t;
                        its.uv = Point2f(u, v);
                        its.mesh = primitive.mesh;
                        primitiveIndex = index;

                        ret = true;
                    }
                }
            }
        }
    }

    return ret;
}

bool Octree::RayIntersect(Ray3f &ray, Intersection &its, uint32_t& primitiveIndex, bool shadowRay) const
{
    if(m_nodes.empty())
    {
        return false;
    }
    
    return TraverseNode(m_nodes.data(), ray, its, primitiveIndex, shadowRay);
}

void Octree::AddMesh(OctNode* node, const Mesh* mesh)
{
    uint32_t primitiveCount = mesh->getTriangleCount();
    for(uint32_t index = 0; index < primitiveCount; ++index)
    {
        if(!AddPrimitive(node, mesh, index))
        {
            std::cout << "[Octree::Build] failed to add primitive. mesh:" << mesh->getName().c_str() << ", primitive: " << index << "\n";
        }
    }
}

bool Octree::AddPrimitive(OctNode* node, const Mesh* mesh, uint32_t index)
{
    auto AddPrimitive2Childen = [&](const Mesh* curMesh, uint32_t triangleIndex)
    {
        bool bAdded = false;

        for (uint32_t childIndex = node->children, maxChild = node->children + 8; childIndex < maxChild; ++childIndex)
        {
            if(AddPrimitive(&m_nodes[childIndex], curMesh, triangleIndex))
            {
                bAdded = true;
            }
        }
        
        assert(bAdded);
        if(!bAdded)
        {
            return false;
        }

        return true;
    };
    
    if(!node->box.overlaps(mesh->getBoundingBox(index)))
    {
        return false;
    }

    if(node->children > 0)
    {
        return AddPrimitive2Childen(mesh, index);
    }
    else
    {
        if(node->primitives.empty() || node->primitives.back().mesh != mesh)
        {
            node->primitives.push_back(OctPrimitive{mesh});
        }
        node->primitives.back().triangles.push_back(index);
    }

    size_t primitiveCount = 0;
    for(auto primitive: node->primitives)
    {
        primitiveCount += primitive.triangles.size();
    }

    if(primitiveCount >= m_maxPrimitiveCount && node->depth + 1 < m_maxDepth)
    {
        BornChildren(node);
        assert(node->children > 0);

        for (auto primitive : node->primitives)
        {
            for(auto triangleIndex: primitive.triangles)
            {
                AddPrimitive2Childen(primitive.mesh, triangleIndex);
            }
        }
        
        node->primitives.clear();
    }
    
    return true;
}

void Octree::BornChildren(OctNode* node)
{
    uint32_t depth = node->depth + 1;
    auto center = node->box.getCenter();
    uint32_t childIndex = m_nodes.size();
    node->children = childIndex;
    for(int i = 0; i < 8; ++i, ++childIndex)
    {
        auto corner = node->box.getCorner(i);
        Point3f minPoint = Point3f(std::min(corner.x(), center.x()),
                                std::min(corner.y(), center.y()),
                                std::min(corner.z(), center.z()));

        Point3f maxPoint = Point3f(std::max(corner.x(), center.x()),
                        std::max(corner.y(), center.y()),
                        std::max(corner.z(), center.z()));

        m_nodes.push_back(OctNode{depth, BoundingBox3f(minPoint, maxPoint), 0});
    }
}

std::shared_ptr<Octree> g_octree;

void Accel::addMesh(Mesh *mesh)
{
    if(nullptr == mesh)
    {
        return;
    }
    
    m_meshes.push_back(mesh);
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build()
{
#if USE_OCTREE
    if(!g_octree)
    {
        g_octree = std::make_shared<Octree>();
    }

    g_octree->Clean();

    int maxDepth = maxOctreeDepth;

    if(maxDepth <= 0)
    {
#ifdef _DEBUG
    maxDepth = 2;
#else
    maxDepth = 8;
#endif
    }

    g_octree->Build(m_meshes, m_bbox, maxDepth, 16);
#endif
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const
{
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    auto RayIntersectWithMesh = [&](const Mesh* mesh)
    {
        bool foundIntersection = false;
        
        /* Brute force search through all triangles */
        for (uint32_t idx = 0; idx < mesh->getTriangleCount(); ++idx)
        {
            float u, v, t;
            if (mesh->rayIntersect(idx, ray, u, v, t))
            {
                /* An intersection was found! Can terminate
                   immediately if this is a shadow ray query */
                if (shadowRay)
                    return true;
                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = mesh;
                f = idx;
                foundIntersection = true;
            }
        }

        return foundIntersection;
    };
    
    if(g_octree)
    {
        foundIntersection = g_octree->RayIntersect(ray, its, f, shadowRay);
        if(foundIntersection && shadowRay)
        {
            return true;
        }
    }
    else
    {
        for(auto mesh: m_meshes)
        {
            if(RayIntersectWithMesh(mesh))
            {
                foundIntersection = true;
                if(foundIntersection && shadowRay)
                {
                    return true;
                }
            }
        }
    }

    if (foundIntersection)
    {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END

