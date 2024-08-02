#include <nori/emitter.h>

static const std::string s_ClassName = "area";

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter
{
public:
    AreaLight(const PropertyList& props)
    {
        m_radiance = props.getColor("radiance");
    }
    
    std::string toString() const 
    {
        return s_ClassName;
    }

protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaLight, s_ClassName);
NORI_NAMESPACE_END