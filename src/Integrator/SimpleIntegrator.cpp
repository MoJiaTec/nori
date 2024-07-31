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
        const Color3f backColor(0.0f);
        
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return backColor;
        }

        auto p = m_position;
        auto x = its.shFrame.toWorld(its.p);
        auto dir = p - x;
        Ray3f wi(x, dir.normalized(), Epsilon, dir.norm());
        wi.o += dir * 0.00001f;
        if(scene->rayIntersect(wi))
        {
            return backColor;
        }

        auto cosTheta = its.shFrame.cosTheta(its.toLocal(wi.d)) * 0.5f + 0.5f;
        //auto cosTheta = std::max(0.0f, its.shFrame.cosTheta(its.shFrame.toLocal(wi.d)));
        auto lo = m_energy * cosTheta / ((4 * M_PI * M_PI) * dir.squaredNorm());
        return lo;
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