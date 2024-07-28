#include <nori/integrator.h>
#include <nori/mesh.h>
#include <nori/scene.h>

static const std::string s_IntegratorName = "Whitted";

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList& props)
    {
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const
    {
        return s_IntegratorName;
    }

protected:
    std::string m_myProperty;
};

NORI_REGISTER_CLASS(WhittedIntegrator, s_IntegratorName);
NORI_NAMESPACE_END