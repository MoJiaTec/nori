#include <nori/emitter.h>
#include <nori/mesh.h>

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

    void setParent(NoriObject *parent) override
    {
        m_mesh = static_cast<Mesh*>(parent);
    }

protected:
    Color3f m_radiance;
    const Mesh* m_mesh;
};

NORI_REGISTER_CLASS(AreaLight, s_ClassName);
NORI_NAMESPACE_END