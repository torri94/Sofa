#include <Dani/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <SofaBaseCollision/ContactListener.h>
#include <sofa/core/ObjectFactory.h>
#include <fstream>
#include <string>
#include <cmath>

#include <Windows.h>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <conio.h>


int a1 = 0;
int d_ID1 = 1;

DCB dcb;



class EasyCollision1 : public sofa::core::collision::ContactListener
{
public:
SOFA_CLASS(EasyCollision1, sofa::core::collision::ContactListener);

    EasyCollision1();
    virtual ~EasyCollision1() override;

protected:
   
    EasyCollision1( sofa::core::CollisionModel* collModel1 = NULL, sofa::core::CollisionModel* collModel2 = NULL) : ContactListener( collModel1, collModel2) { }
    virtual void beginContact(const sofa::helper::vector<const sofa::helper::vector<sofa::core::collision::DetectionOutput>*>& contacts) 
    {
        for (const sofa::helper::vector<sofa::core::collision::DetectionOutput>* contact : contacts) 
        {
            for (const sofa::core::collision::DetectionOutput& detection_output : *contact) 
            {
                if (a1 == 0)
                {
                        std::cout << getTime() << "ID:1  Object '" << detection_output.elem.first.model->getName() << "' collided with object '" <<
                        detection_output.elem.second.model->getName() << "' at positions (" << detection_output.point[0] << ") and (" <<
                        detection_output.point[1] << ")\n";
                        std::ofstream fout( ( "1/alabarda" + std::to_string(d_ID1) + "f.txt").c_str() );
                        
                        char device_name[] = "\\\\.\\COM13";
                        //creo handle che mi gestisce la comunicazione seriale. Qui vado a settare i vari parametri come il nome della porta
                        HANDLE hComm = CreateFile(
                            L"\\\\.\\COM13",
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
                        if (SetCommState(hComm, &dcb)==0)
                        {
                           
                            std::cout << "Set configuration port has problem.";

                        }
                        



                        char outputData[]="s";
                        
                        DWORD NumberOfBytesWritten = 0;//DWORD dimensione pari al doppio di un word
                        DWORD nNumberOfBytesToWrite = strlen(outputData);
                        if(WriteFile(hComm, outputData, nNumberOfBytesToWrite, &NumberOfBytesWritten, NULL) ==0) std::cout << "errore write";
                        if(CloseHandle(hComm) == 0 ) std::cout << "errore close";/**/

                        fout << getTime() << " BANG" << std::endl;
                        a1 = 1;
                }
            }
        }
    }
};