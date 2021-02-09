#include <Dani/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <SofaBaseCollision/ContactListener.h>
#include <sofa/core/ObjectFactory.h>
#include <fstream>
#include <string>
#include <cmath>

int a2 = 0;
int d_ID2 = 2;
class EasyCollision2 : public sofa::core::collision::ContactListener
{
public:
SOFA_CLASS(EasyCollision2, sofa::core::collision::ContactListener);

    EasyCollision2();
    virtual ~EasyCollision2() override;

protected:
   
    EasyCollision2( sofa::core::CollisionModel* collModel1 = NULL, sofa::core::CollisionModel* collModel2 = NULL) : ContactListener( collModel1, collModel2) { }
    virtual void beginContact(const sofa::helper::vector<const sofa::helper::vector<sofa::core::collision::DetectionOutput>*>& contacts) 
    {
        for (const sofa::helper::vector<sofa::core::collision::DetectionOutput>* contact : contacts) 
        {
            for (const sofa::core::collision::DetectionOutput& detection_output : *contact) 
            {
                if (a2 == 0)
                {
                        std::cout << getTime() << "ID:1  Object '" << detection_output.elem.first.model->getName() << "' collided with object '" <<
                        detection_output.elem.second.model->getName() << "' at positions (" << detection_output.point[0] << ") and (" <<
                        detection_output.point[1] << ")\n";
                        std::ofstream fout( ( "2/alabarda" + std::to_string(d_ID2) + "f.txt").c_str() );
                        
                        fout << getTime() << " BANG" << std::endl;
                        a2 = 1;
                }
            }
        }
    }
};
