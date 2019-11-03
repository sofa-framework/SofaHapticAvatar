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
#include <SofaHapticAvatar/HapticAvatarDriver.h>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

namespace sofa
{

namespace component
{

namespace controller
{

HapticAvatarDriver::HapticAvatarDriver(const std::string& portName)
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

void HapticAvatarDriver::connectDevice()
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


HapticAvatarDriver::~HapticAvatarDriver()
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


sofa::helper::fixed_array<float, 4> HapticAvatarDriver::getAnglesAndLength()
{
    sofa::helper::fixed_array<float, 4> results;
    char incomingData[INCOMING_DATA_LEN];
    char outgoingData[OUTGOING_DATA_LEN] = "2 \n";
    int outlen = strlen(outgoingData);
    bool write_success = WriteData(outgoingData, outlen);
    if (!write_success) {
        std::cout << "failed_to_send_times" << std::endl;
    }

    // request data
    int res = getData(incomingData, false);

    // parse Data into floats
    // 0: rotation angle
    // 1: Pitch angle
    // 2: insertion length
    // 3: yaw angle
    char* pEnd;
    results[0] = std::strtof(incomingData, &pEnd) * 0.0001;
    for (unsigned int i = 1; i < 4; ++i)
    {
        results[i] = std::strtof(pEnd, &pEnd) * 0.0001;
    }
    
    return results;
}


std::string HapticAvatarDriver::getIdentity()
{
    char incomingData[INCOMING_DATA_LEN];
    char outgoingData[OUTGOING_DATA_LEN] = "1 \n";
    int outlen = strlen(outgoingData);
    bool write_success = WriteData(outgoingData, outlen);
    if (!write_success) {
        std::cout << "failed_to_send_times" << std::endl;
    }
    int res = getData(incomingData, false);
    std::string iden = std::string(incomingData);

    return iden;
}


int HapticAvatarDriver::getData(char *buffer, bool do_flush)
{
    bool response = false;
    int cptSecu = 0;
    int n = 0;
    int que = 0;
    char * pch;
    int num_cr = 0;
    while (!response && cptSecu < 1000)
    {
        n = ReadData(buffer, INCOMING_DATA_LEN, &que, do_flush);        
        if (n > 0) 
        {
            // count the number of /n in the return string
            pch = strchr(buffer, '\n');
            while (pch != NULL)
            {
                num_cr++;
                pch = strchr(pch + 1, 's');
            }
            response = true;
        }

        cptSecu++;
    }

    if (!response) // secu loop reach end
    {
        std::cerr << "Error getData failed for " << buffer << std::endl;
    }

    return n;
}


int HapticAvatarDriver::ReadData(char *buffer, unsigned int nbChar, int *queue, bool do_flush)
{
    //Number of bytes we'll have read
    DWORD bytesRead = 0;
    DWORD junkBytesRead = 0;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(m_hSerial, &m_errors, &m_status);

    *queue = (int)m_status.cbInQue;


    if (do_flush) {
        if (*queue > 0)
            // read away as much as possible, max 512
            ReadFile(m_hSerial, buffer, std::min(unsigned int(*queue), nbChar), &bytesRead, NULL);
    }
    else {
        // regardless of what the queue is, read nbChar bytes, in an attempt to suspend the thread
        ReadFile(m_hSerial, buffer, *queue, &bytesRead, NULL);
        //ClearCommError(this->hSerial, &this->errors, &this->status);
        //*queue = (int)status.cbInQue;
        //if (*queue > 0) {
        //	char junkbuffer[32];
        //	// clear the buffer from any remaining bytes
        //	ReadFile(this->hSerial, junkbuffer, min(*queue, 32), &junkBytesRead, NULL);
        //}

    }
    return bytesRead;

    ////Check if there is something to read
    //if (queue > 0)
    //{
    //	//Try to read the required number of chars, and return the number of read bytes on success
    //	if (ReadFile(this->hSerial, buffer, min(queue, nbChar), &bytesRead, NULL))
    //	{
    //		return bytesRead;
    //	}

    //}

    ////If nothing has been read, or that an error was detected return 0
    //return 0;
}


bool HapticAvatarDriver::writeData(std::string msg)
{
    //writeData
    char outgoingData[OUTGOING_DATA_LEN];
    std::strcpy(outgoingData, msg.c_str());
    int outlen = strlen(outgoingData);
    
    return WriteData(outgoingData, outlen);
}


bool HapticAvatarDriver::WriteData(char *buffer, unsigned int nbChar)
{
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if (!WriteFile(m_hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(m_hSerial, &m_errors, &m_status);

        return false;
    }
    else
        return true;
}



} // namespace controller

} // namespace component

} // namespace sofa
