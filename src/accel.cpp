/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/accel.h>
#include <Eigen/Geometry>

extern int maxOctreeDepth;


NORI_NAMESPACE_BEGIN

#define USE_OCTREE 1

struct OctLeaf
{
    
};

struct OctNode
{
    uint32_t depth;
    BoundingBox3f box;
    uint32_t children;
    
    std::vector<uint32_t> primitives;

    bool isValid = false;
};

class Octree
{
public:
    void Build(const Mesh* mesh, uint32_t maxDepth, uint32_t maxPrimitiveCount);
    void Clean();

    bool RayIntersect(Ray3f &ray, Intersection &its, uint32_t& primitiveIndex, bool shadowRay) const;

protected:
    bool AddPrimitive(OctNode* node, uint32_t index);
    void BornChildren(OctNode* node);

    bool TraverseNode(const OctNode* node, Ray3f& ray, Intersection& its, uint32_t& primitiveIndex, bool shadowRay) const;
    //void TraverseChildren(OctNode* node, )

    bool CheckNode(OctNode* node, uint32_t &primitiveCount);

protected:
    std::vector<OctNode> m_nodes;
    uint32_t m_maxDepth = 0;
    uint32_t m_maxPrimitiveCount = 0;

    const Mesh* m_mesh = nullptr;
};

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
        primitiveCount += node->primitives.size();
    }
    
    return node->isValid;
}

void Octree::Build(const Mesh* mesh, uint32_t maxDepth, uint32_t maxPrimitiveCount)
{
    if(!mesh)
    {
        return;
    }

    Clean();

    m_mesh = mesh;
    m_maxDepth = maxDepth;
    m_maxPrimitiveCount = maxPrimitiveCount;

    m_nodes.reserve(1 + std::pow(2, (3 * (maxDepth + 1))));
    m_nodes.push_back(OctNode{0, m_mesh->getBoundingBox(), 0, std::vector<uint32_t>()});
    OctNode* root = &m_nodes[0];
    
    uint32_t primitiveCount = m_mesh->getTriangleCount();
    for(uint32_t index = 0; index < primitiveCount; ++index)
    {
        AddPrimitive(root, index);
    }

    //会导致内存重新分配
    m_nodes.shrink_to_fit();
    
    root = &m_nodes[0];
    uint32_t buildPrimitiveCount = 0;
    CheckNode(root, buildPrimitiveCount);
    std::cout << "[Octree::Build] original primitive count: " << primitiveCount << ", build primitive count: " << buildPrimitiveCount << "\n";
}

void Octree::Clean()
{
    m_nodes.clear();
    m_maxDepth = 0;
    m_maxPrimitiveCount = 0;
    m_mesh = nullptr;
}

bool Octree::TraverseNode(const OctNode* node, Ray3f& ray, Intersection& its, uint32_t& primitiveIndex, bool shadowRay) const
{
    bool ret = false;

    // if (!node->box.rayIntersect(ray))
    // {
    //     return ret;
    // }

    if (node->primitives.empty())
    {
        if(node->children > 0)
        {
            for (uint32_t childIndex = node->children, maxChild = childIndex + 8; childIndex < maxChild; ++childIndex)
            {
                if (TraverseNode(&m_nodes[childIndex], ray, its, primitiveIndex, shadowRay))
                {
                    if (shadowRay)
                    {
                        return true;
                    }

                    ret = true;
                }
            }
        }
    }
    else
    {
        for (auto index : node->primitives)
        {
            float u, v, t;
            if (m_mesh->rayIntersect(index, ray, u, v, t))
            {
                if (shadowRay)
                {
                    return true;
                }

                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_mesh;
                primitiveIndex = index;

                ret = true;
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
    
    return TraverseNode(&m_nodes[0], ray, its, primitiveIndex, shadowRay);
}

bool Octree::AddPrimitive(OctNode* node, uint32_t index)
{
    if(!node->box.overlaps(m_mesh->getBoundingBox(index)))
    {
        return false;
    }
    
    do
    {
        if(node->children > 0)
        {
            bool ret = false;
            for(uint32_t childIndex = node->children, maxChild = node->children + 8; childIndex < maxChild; ++childIndex)
            {
                if(AddPrimitive(&m_nodes[childIndex], index) > 0)
                {
                    ret = true;
                }
            }

            assert(ret);
            if(!ret)
            {
                std::cout << "[Octree::AddPrimitive] failed to add primitive: " << index << "\n";
            }
            return ret;
        }

		if (node->depth + 1 < m_maxDepth && node->primitives.size() + 1 >= m_maxPrimitiveCount)
		{
			BornChildren(node);

			for (auto primitiveIndex : node->primitives)
			{
				for (uint32_t childIndex = node->children, maxChild = node->children + 8; childIndex < maxChild; ++childIndex)
				{
					AddPrimitive(&m_nodes[childIndex], index);
				}
			}

            node->primitives.clear();
		}
        else
        {
            node->primitives.push_back(index);
            return true;
        }
    }
    while (true);
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

        m_nodes.push_back(OctNode{depth, BoundingBox3f(minPoint, maxPoint), 0, std::vector<uint32_t>()});
    }
}

std::shared_ptr<Octree> g_octree;

void Accel::addMesh(Mesh *mesh)
{
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build()
{
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

    g_octree->Build(m_mesh, maxDepth, 16);
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const
{
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

#if USE_OCTREE
    if(g_octree)
    {
        foundIntersection = g_octree->RayIntersect(ray, its, f, shadowRay);
        if(shadowRay)
        {
            return true;
        }
    }
#else
    /* Brute force search through all triangles */
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }
#endif

    if (foundIntersection) {
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

