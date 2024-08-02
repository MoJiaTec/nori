#include <nori/integrator.h>
#include <nori/mesh.h>
#include <nori/scene.h>
#include <nori/warp.h>

static const std::string s_IntegratorName = "whitted";

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList& props)
    {
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        const Color3f backColor(0.0f);
        
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return backColor;
        }

        float aoScale = 1.0f;

        auto rng = sampler->next2D();
        auto wi = Warp::squareToCosineHemisphere(rng);
        wi = its.shFrame.toWorld(wi).normalized();
        int visiblity = 0;
        if (!scene->rayIntersect(Ray3f(its.p + wi * aoScale, wi)))
        {
            visiblity = 1;
        }
        
        return Color3f(float(visiblity));
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