/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_HAPTICAVATAR_HAPTICAVATARAPI_H
#define SOFA_HAPTICAVATAR_HAPTICAVATARAPI_H

#include <SofaHapticAvatar/config.h>
#include <string>

namespace sofa 
{

namespace component 
{

namespace controller 
{

/**
* HapticAvatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarAPI
{
public:
    HapticAvatarAPI(const std::string& portName);

    virtual ~HapticAvatarAPI();

    bool IsConnected() { return m_connected; }

    void connectDevice();

private:
    //Connection status
    bool m_connected;

    //Serial comm handler
    HANDLE m_hSerial;
    //Get various information about the connection
    COMSTAT m_status;
    //Keep track of last error
    DWORD m_errors;

    std::string m_portName;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HAPTICAVATAR_HAPTICAVATARAPI_H
