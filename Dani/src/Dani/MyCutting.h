#include <Dani/config.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <SofaBaseCollision/ContactListener.h>
#include <sofa/core/ObjectFactory.h>

class EasyCollision : public sofa::core::collision::ContactListener 
{
public:
    SOFA_CLASS(EasyCollision, sofa::core::collision::ContactListener);

protected:
    EasyCollision();
    virtual ~EasyCollision() override;
    
    EasyCollision(sofa::core::CollisionModel* collModel1 = NULL, sofa::core::CollisionModel* collModel2 = NULL): ContactListener(collModel1, collModel2) { }

    virtual void beginContact(const sofa::helper::vector<const sofa::helper::vector<sofa::core::collision::DetectionOutput>*>& contacts)
    {

        for (const sofa::helper::vector<sofa::core::collision::DetectionOutput>* contact : contacts) 
        {
            for (const sofa::core::collision::DetectionOutput& detection_output : *contact)
            {
                std::cout << "Object '" << detection_output.elem.first.model->getName() << "' collided with object '" << detection_output.elem.second.model->getName() << "' at positions (" << detection_output.point[0] << ") and (" <<  detection_output.point[1] << ")\n";
            }
        }

    }
};

int EasyCollisionClass = sofa::core::RegisterObject("Simple collision component").add<EasyCollision>();

SOFA_DECL_CLASS(EasyCollision)