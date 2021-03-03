/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include "EvalPointsDistance.h"
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>
#include <iostream>
#include <process.h>
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <cstdlib>
#include <string>
#include <time.h> 
#include <chrono>
#include <thread>
#include <algorithm>
#include <conio.h>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <fstream>
#include <iomanip>

#include <fstream>
int a;

DCB dcb;
CHAR sysTimeStr2[13] = {};
SYSTEMTIME systemTime2;


namespace sofa::component::misc
{

template<class DataTypes>
EvalPointsDistance<DataTypes>::EvalPointsDistance()
    : f_draw( initData(&f_draw, true, "draw", "activate rendering of lines between associated points"))
    , isToPrint( initData(&isToPrint, false, "isToPrint", "suppress somes data before using save as function"))
    , f_filename( initData(&f_filename, "filename", "output file name"))
    , f_period( initData(&f_period, 0.0, "period", "period between outputs"))
    , dist( initData(&dist, "distance", "distances (OUTPUT)"))
    , distMean( initData(&distMean, 1.0, "distMean", "mean distance (OUTPUT)"))
    , distMin( initData(&distMin, 1.0, "distMin", "min distance (OUTPUT)"))
    , distMax( initData(&distMax, 1.0, "distMax", "max distance (OUTPUT)"))
    , distDev( initData(&distDev, 1.0, "distDev", "distance standard deviation (OUTPUT)"))
    , d_ID(initData(&d_ID, 1, "ID", "Simulation identity"))
    , rdistMean( initData(&rdistMean, 1.0, "rdistMean", "mean relative distance (OUTPUT)"))
    , rdistMin( initData(&rdistMin, 1.0, "rdistMin", "min relative distance (OUTPUT)"))
    , rdistMax( initData(&rdistMax, 1.0, "rdistMax", "max relative distance (OUTPUT)"))
    , rdistDev( initData(&rdistDev, 1.0, "rdistDev", "relative distance standard deviation (OUTPUT)"))
    , mstate1(initLink("object1", "Mechanical state 1"))
    , mstate2(initLink("object2", "Mechanical state 2"))
    , outfile(nullptr)
    , lastTime(0)
{


    mstate1.setPath("@./"); // default path: state in the same node
    mstate2.setPath("@./"); // default path: state in the same node
    box1 = sofa::defaulttype::BoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    box2 = sofa::defaulttype::BoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
}

template<class DataTypes>
EvalPointsDistance<DataTypes>::~EvalPointsDistance()
{
    a = 0;
    if (outfile)
        delete outfile;
}


//-------------------------------- init------------------------------------
template<class DataTypes>
void EvalPointsDistance<DataTypes>::init()
{
    if(isToPrint.getValue()==true) dist.setPersistent(false);
    if (!mstate1 )
    {
        mstate1 = dynamic_cast<core::behavior::MechanicalState<DataTypes>*>(this->getContext()->getMechanicalState());
        box1 = mstate1->f_bbox.getValue();
        msg_error()<< " Mechanical State object1 not found, this will be taken in the same context ";
    }
    if (!mstate2)
    {
        mstate2 = dynamic_cast<core::behavior::MechanicalState<DataTypes>*>(this->getContext()->getMechanicalState());
        this->box2 = mstate1->f_bbox.getValue();
        msg_error()<< " Mechanical State object2 not found, this will be taken in the same context ";
    }


    if (!mstate1 || !mstate2)
    {
        msg_error()<< " ERROR Mechanical State object1 and object2 expected ";
        return;
    }

    reinit();
}

//-------------------------------- reinit ----------------------------------
template<class DataTypes>
void EvalPointsDistance<DataTypes>::reinit()
{
    if (outfile)
        delete outfile;
    const std::string& filename = f_filename.getFullPath();
    if (!filename.empty())
    {
        outfile = new std::ofstream(filename.c_str());
        if( !outfile->is_open() )
        {
            msg_error() << "Error creating file "<<filename;
            delete outfile;
            outfile = nullptr;
        }
        else
        {
            (*outfile) << "# name\t\t\ttime\t\tmean\t\tmin\t\tmax\t\tdev\t\tmean(%)\t\tmin(%)\t\tmax(%)\t\tdev(%)" << std::endl;
            msg_info() << "OutputFile " << filename << " created.";
        }
    }
    else
    {
        outfile = nullptr;
    }

    if(f_period.getValue() == 0.0)
    {
        msg_error() << " ERROR period must be different of zero  ";
        return;
    }

    lastTime = -f_period.getValue();
    eval();
}


//-------------------------------- eval ------------------------------------
template<class DataTypes>
SReal EvalPointsDistance<DataTypes>::eval()
{
    if (!mstate1 || !mstate2)
        return 0.0;
    const VecCoord& x0 = mstate1->read(core::ConstVecCoordId::restPosition())->getValue();
    const VecCoord& x1 = mstate1->read(core::ConstVecCoordId::position())->getValue();
    const VecCoord& x2 = mstate2->read(core::ConstVecCoordId::position())->getValue();

    return this->doEval(x1, x2, x0);

}

//-------------------------------- doEval------------------------------------
template<class DataTypes>
SReal EvalPointsDistance<DataTypes>::doEval(const VecCoord& x1, const VecCoord& x2, const VecCoord& /*x0*/)
{
    const int n = (x1.size()<x2.size())?x1.size():x2.size();
    int s1 = x1.size()-n;
    int s2 = x2.size()-n;
    Real dsum = 0.0;
    Real dmin = 0.0;
    Real dmax = 0.0;
    Real d2 = 0.0;
    Real rdsum = 0.0;
    Real rdmin = 0.0;
    Real rdmax = 0.0;
    Real rd2 = 0.0;
    int rn=0;
//    Coord dx0 = x2[s2]-x0[s1];

    const Vec3 minBox = box1.minBBox();
    const Vec3 maxBox = box1.maxBBox();
    Real meanRefSize = (Real)((maxBox[0]-minBox[0])+(maxBox[1]-minBox[1])+(maxBox[2]-minBox[2]))/3.0f;
    helper::vector<Real> &distances = *dist.beginEdit();
    distances.resize(n);
    if (d_ID != 0)

    {

        /* ho 5 passi: 
        1 imposto il bianco, invio un segnale e scrivo
        2 mantengo il bianco per farlo comparire a schermo
        3 imposto nero per poter passare poi a grigio
        4 imposto grigio, invio un segnale e scrivo il tempo
        5 imposto nero e scrivo il tempo
        */
        //BIANCO
        if (a == 0)
        {
           
            

            GetLocalTime(&systemTime2);
            sprintf_s(sysTimeStr2,
                "%u:%u:%u:%u",
                systemTime2.wHour,
                systemTime2.wMinute,
                systemTime2.wSecond,
                systemTime2.wMilliseconds);

            std::ofstream fout(("behaviors/" + std::to_string(d_ID.getValue()) + "/start_time" + std::to_string(d_ID.getValue()) + ".txt").c_str());
            fout << sysTimeStr2 <<" "<< getTime() << std::endl;


            for (int i = 0; i < n; ++i)
            {
                
                Real d = 0;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                //Real d0 = (Real)(x1[s1+i]-x0[s1+i]).norm();
                //Real d0 = (Real)(x2[s2+i]-x0[s1+i]-dx0).norm();
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }
            
            
        }
        if (a == 1)

        {//BIANCO
            //Mettendo delle istruzioni qui mi si blocca
            

            for (int i = 0; i < n; ++i)
            {
                
                Real d = 0;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                //Real d0 = (Real)(x1[s1+i]-x0[s1+i]).norm();
                //Real d0 = (Real)(x2[s2+i]-x0[s1+i]-dx0).norm();
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }


        }


        if (a == 2)

        {//BIANCO
            //Mettendo delle istruzioni qui mi si blocca
           // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            /*
            }*/
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
                std::cout << "Set configuration port has problem.R";
            }
            BYTE data;
            data = 0x01;
            DWORD NumberOfBytesWritten = 0;//DWORD dimensione pari al doppio di un word
            //DWORD nNumberOfBytesToWrite = strlen(outputData);
            if (WriteFile(hComm, &data, 1, &NumberOfBytesWritten, NULL) == 0) std::cout << "errore write";
            if (CloseHandle(hComm) == 0) std::cout << "errore close";


            for (int i = 0; i < n; ++i)
            {

                Real d = 0;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                //Real d0 = (Real)(x1[s1+i]-x0[s1+i]).norm();
                //Real d0 = (Real)(x2[s2+i]-x0[s1+i]-dx0).norm();
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }


        }


        if (a == 3)

        {//NERO
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            for (int i = 0; i < n; ++i)
            {
               // std::this_thread::sleep_for(std::chrono::milliseconds(500));
                Real d = 0.2;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                //Real d0 = (Real)(x1[s1+i]-x0[s1+i]).norm();
                //Real d0 = (Real)(x2[s2+i]-x0[s1+i]-dx0).norm();
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }
            

        }

        if (a == 4)
        {//GRIGIO

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
                std::cout << "Set configuration port has problem.R";
            }
            BYTE data;
            data = 0x02;
            DWORD NumberOfBytesWritten = 0;//DWORD dimensione pari al doppio di un word
            //DWORD nNumberOfBytesToWrite = strlen(outputData);
            if (WriteFile(hComm, &data, 1, &NumberOfBytesWritten, NULL) == 0) std::cout << "errore write";
            if (CloseHandle(hComm) == 0) std::cout << "errore close";

            GetLocalTime(&systemTime2);
          /*  sprintf_s(sysTimeStr1,
                "%u:%u:%u:%u",
                systemTime1.wHour,
                systemTime1.wMinute,
                systemTime1.wSecond,
                systemTime1.wMilliseconds);*/

            std::ofstream fout(("behaviors/" + std::to_string(d_ID.getValue()) + "/end_white"+ std::to_string(d_ID.getValue()) +".txt").c_str());
            fout << sysTimeStr2 <<" "<< getTime() << std::endl;


            for (int i = 0; i < n; ++i)
            {
                
                Real d = 0.1;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }


        }
        if (a == 5)

        {//NERO

            GetLocalTime(&systemTime2);
          /*  sprintf_s(sysTimeStr1,
                "%u:%u:%u:%u",
                systemTime1.wHour,
                systemTime1.wMinute,
                systemTime1.wSecond,
                systemTime1.wMilliseconds);*/

            std::ofstream fout(("behaviors/" + std::to_string(d_ID.getValue()) + "/end_grey" + std::to_string(d_ID.getValue()) +".txt").c_str());
            fout << sysTimeStr2 << " " << getTime() << std::endl;

            for (int i = 0; i < n; ++i)
            {
                Real d = 0.5;
                distances[i] = d;
                dsum += d;
                d2 += d * d;
                if (i == 0 || d < dmin) dmin = d;
                if (i == 0 || d > dmax) dmax = d;
                Real d0 = meanRefSize;
                if (d0 > 1.0e-6)
                {
                    Real rd = d / d0;
                    rdsum += rd;
                    rd2 += rd * rd;
                    if (rn == 0 || rd < rdmin) rdmin = rd;
                    if (rn == 0 || rd > rdmax) rdmax = rd;
                    ++rn;
                }
            }


        }
        a++;
    
    }
    

    


    dist.endEdit();

    Real dmean = (n>0)?dsum/n : (Real)0.0;
    Real ddev = (Real)((n>1)?sqrtf((float)(d2/n - (dsum/n)*(dsum/n))) : 0.0);
    distMean.setValue(dmean);
    distMin.setValue(dmin);
    distMax.setValue(dmax);
    distDev.setValue(ddev);

    Real rdmean = (rn>0)?rdsum/rn : (Real)0.0;
    Real rddev = (Real)((rn>1)?sqrtf((float)(rd2/rn - (rdsum/rn)*(rdsum/rn))) : 0.0);
    rdistMean.setValue(rdmean);
    rdistMin.setValue(rdmin);
    rdistMax.setValue(rdmax);
    rdistDev.setValue(rddev);
 
        return dmean;
}

//-------------------------------- draw ------------------------------------
template<class DataTypes>
void EvalPointsDistance<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!f_draw.getValue())
        return;
    if (!mstate1 || !mstate2)
        return;
    const VecCoord& x1 = mstate1->read(core::ConstVecCoordId::position())->getValue();
    const VecCoord& x2 = mstate2->read(core::ConstVecCoordId::position())->getValue();
    this->doDraw(vparams, x1,x2);
}

//-------------------------------- doDraw------------------------------------
template<class DataTypes>
void EvalPointsDistance<DataTypes>::doDraw(const core::visual::VisualParams* vparams, const VecCoord& x1, const VecCoord& x2)
{
    const int n = (x1.size()<x2.size())?x1.size():x2.size();
    int s1 = x1.size()-n;
    int s2 = x2.size()-n;

    vparams->drawTool()->saveLastState();
    vparams->drawTool()->disableLighting();

    std::vector<sofa::defaulttype::Vector3> vertices;
    sofa::helper::types::RGBAColor color(1, 0.5, 0.5, 1);

    for (int i=0; i<n; ++i)
    {
        sofa::defaulttype::Vector3 v0(x1[s1+i][0],x1[s1+i][1],x1[s1+i][2]);
        sofa::defaulttype::Vector3 v1(x2[s2+i][0],x2[s2+i][1],x2[s2+i][2]);
        vertices.push_back(v0);
        vertices.push_back(v1);
    }
    vparams->drawTool()->drawLines(vertices, 1, color);
    vparams->drawTool()->restoreLastState();
}

//-------------------------------- handleEvent ------------------------------------
template<class DataTypes>
void EvalPointsDistance<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{

    if (!mstate1 || !mstate2)
            return;

    if (simulation::AnimateEndEvent::checkEventType(event))
    {
        double time = getContext()->getTime();
        // write the state using a period
        if (time+getContext()->getDt()/2 >= (lastTime + f_period.getValue()))
        {

            eval();
            if (outfile==nullptr)
            {
                msg_info() << "# name\ttime\tmean\tmin\tmax\tdev\tmean(%)\tmin(%)\tmax(%)\tdev(%)";
                msg_info() << this->getName() << "\t" << time
                     << "\t" << distMean.getValue() << "\t" << distMin.getValue() << "\t" << distMax.getValue() << "\t" << distDev.getValue()
                     << "\t" << 100*rdistMean.getValue() << "\t" << 100*rdistMin.getValue() << "\t" << 100*rdistMax.getValue() << "\t" << 100*rdistDev.getValue();
            }
            else
            {
                (*outfile) << std::setfill(' ') << std::setw(10) << this->getName() << "\t" << std::setw(10) << time
                           << "\t" << std::setw(10) << distMean.getValue() << "\t" << std::setw(10) << distMin.getValue() << "\t" << std::setw(10) << distMax.getValue()
                           << "\t" << std::setw(10) << distDev.getValue()  << "\t" << std::setw(10) << 100*rdistMean.getValue() << "\t" << std::setw(10) << 100*rdistMin.getValue()
                           << "\t" << std::setw(10) << 100*rdistMax.getValue() << "\t" << std::setw(10) << 100*rdistDev.getValue()
                           << std::endl;
            }
            lastTime = time;
        }
    }
}

} // namespace sofa::component::misc
