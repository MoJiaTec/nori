#include <nori/integrator.h>
#include <nori/mesh.h>
#include <nori/scene.h>

static const std::string s_IntegratorName = "simple";

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator 
{
public:
    SimpleIntegrator(const PropertyList& props) 
    {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
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
    Point3f m_position;
    Color3f m_energy;
};

NORI_REGISTER_CLASS(SimpleIntegrator, s_IntegratorName);
NORI_NAMESPACE_END