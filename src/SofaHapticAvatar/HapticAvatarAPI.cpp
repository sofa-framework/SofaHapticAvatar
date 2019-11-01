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

#include <SofaHapticAvatar/HapticAvatarDefines.h>
#include <SofaHapticAvatar/HapticAvatarAPI.h>
#include <iostream>

namespace sofa
{

namespace component
{

namespace controller
{

HapticAvatarAPI::HapticAvatarAPI(const std::string& portName)
    : m_connected(false)
    , m_portName(portName)
{
    connectDevice();

    if (m_connected == true)
    {
        std::cout << "## Connected!!!!" << std::endl;
    }
    else
    {
        std::cout << "## Not Connected...." << std::endl;
    }
}

void HapticAvatarAPI::connectDevice()
{
    //Try to connect to the given port throuh CreateFile
    m_hSerial = CreateFileA(m_portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    //Check if the connection was successfull
    if (m_hSerial == INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if (GetLastError() == ERROR_FILE_NOT_FOUND) {

            //Print Error if neccessary
            std::cout << "ERROR: Handle was not attached. Reason: " << m_portName << " not available." << std::endl;
        }
        else
        {
            std::cout << "ERROR!!!" << std::endl;
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = { 0 };

        //Try to get the current
        if (!GetCommState(m_hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            std::cout << "failed to get current serial parameters!" << std::endl;
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate = CBR_9600;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;
            //Setting the DTR to Control_Enable ensures that the Arduino is properly
            //reset upon establishing a connection
            dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

            //Set the parameters and check for their proper application
            if (!SetCommState(m_hSerial, &dcbSerialParams))
            {
                std::cout << "ALERT: Could not set Serial Port parameters" << std::endl;
            }
            else
            {
                //If everything went fine we're connected
                m_connected = true;
                //Flush any remaining characters in the buffers
                PurgeComm(m_hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
                //We wait 2s as the arduino board will be reseting
                Sleep(ARDUINO_WAIT_TIME);
            }
        }
    }
}

HapticAvatarAPI::~HapticAvatarAPI()
{
    //Check if we are connected before trying to disconnect
    if (m_connected)
    {
        //We're no longer connected
        m_connected = false;
        //Close the serial handler
        CloseHandle(m_hSerial);
    }
}

} // namespace controller

} // namespace component

} // namespace sofa
