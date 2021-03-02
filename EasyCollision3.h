#include <Dani/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <SofaBaseCollision/ContactListener.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ArgumentParser.h>
#include <SofaSimulationCommon/common.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationCommon/init.h>
#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>


#include <sofa/helper/BackTrace.h>
#include <fstream>
#include <string>
#include <cmath>

#include <Windows.h>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <conio.h>


#include <sstream>
using std::ostringstream;

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Node.inl>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/AnimateVisitor.h>
#include <sofa/simulation/DeactivatedNodeVisitor.h>
#include <sofa/simulation/InitVisitor.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/VisualVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/Factory.inl>
#include <sofa/helper/cast.h>
#include <iostream>







int a3 = 0;





class EasyCollision3 : public sofa::core::collision::ContactListener






{
public:
    
    
    

    SOFA_CLASS(EasyCollision3, sofa::core::collision::ContactListener);
    EasyCollision3();
    virtual ~EasyCollision3() override;




protected:

   


    EasyCollision3(sofa::core::CollisionModel* collModel1 = NULL, sofa::core::CollisionModel* collModel2 = NULL) : ContactListener(collModel1, collModel2) { }
   
    
    
    
    virtual void beginContact(const sofa::helper::vector<const sofa::helper::vector<sofa::core::collision::DetectionOutput>*>& contacts)
    {
        for (const sofa::helper::vector<sofa::core::collision::DetectionOutput>* contact : contacts)
        {
            for (const sofa::core::collision::DetectionOutput& detection_output : *contact)
            {


                
                int d_ID1 = 3;
                CHAR sysTimeStr1[13] = {};
                SYSTEMTIME systemTime1;



                DCB dcb;




                if (a3 == 0)
                {
              
                    GetLocalTime(&systemTime1);
                    sprintf_s(sysTimeStr1,
                        "%u:%u:%u:%u",
                        systemTime1.wHour,
                        systemTime1.wMinute,
                        systemTime1.wSecond,
                        systemTime1.wMilliseconds);
                    std::cout << getTime() << "ID:1  Object '" << detection_output.elem.first.model->getName() << "' collided with object '" <<
                        detection_output.elem.second.model->getName() << "' at positions (" << detection_output.point[0] << ") and (" <<
                        detection_output.point[1] << ")\n";
                    std::ofstream fout(("behaviors/" + std::to_string(d_ID1) + "/collision" + std::to_string(d_ID1) + ".txt").c_str());
                    fout << sysTimeStr1 << " " << getTime() << std::endl;









                    char device_name[] = "\\\\.\\COM4";
                    HANDLE hComm = CreateFile(
                        L"\\\\.\\COM4",
                        GENERIC_READ | GENERIC_WRITE,
                        0,                              //share device
                        NULL,                              //security
                        OPEN_EXISTING,
                        0,
                        NULL
                    );
                    if (hComm == INVALID_HANDLE_VALUE)
                    {
                        std::cout << "error opening\n";
                    }
                    dcb.DCBlength = sizeof(DCB);
                    GetCommState(hComm, &dcb);

                    dcb.BaudRate = 9600;  // Specify buad rate of communicaiton.
                    dcb.StopBits = 1;  // Specify stopbit of communication.
                    dcb.Parity = 0;      // Specify parity of communication.
                    dcb.ByteSize = 8;  // Specify  byte of size of communication.
                    if (SetCommState(hComm, &dcb) == 0)
                    {
                        std::cout << "Set configuration port has problem.";
                    }
                    BYTE data;
                    data = 0x04;
                    DWORD NumberOfBytesWritten = 0;//DWORD dimensione pari al doppio di un word
                    //DWORD nNumberOfBytesToWrite = strlen(outputData);
                    if (WriteFile(hComm, &data, 1, &NumberOfBytesWritten, NULL) == 0) std::cout << "errore write";
                    if (CloseHandle(hComm) == 0) std::cout << "errore close";/**/







                    a3 = 1;
                }
            }
        }
    }
};