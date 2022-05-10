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

#include <SofaHapticAvatar/HapticAvatar_DriverBase.h>
#include <sofa/helper/logging/Messaging.h>

namespace sofa::HapticAvatar
{

    using namespace HapticAvatar;

    HapticAvatar_DriverBase::HapticAvatar_DriverBase(const std::string& portName)
        : m_connected(false)
        , m_portName(portName)
    {
        // First try to connect to device
        connectDevice();

        if (!m_connected)
        {
            msg_error("HapticAvatar_DriverBase") << "## Device Not Connected at port: " << m_portName;
        }
    }


    HapticAvatar_DriverBase::~HapticAvatar_DriverBase()
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




    ///////////////////////////////////////////////////////////////
    /////      Public API methods for any communication       /////
    ///////////////////////////////////////////////////////////////

    int HapticAvatar_DriverBase::resetDevice(int mode)
    {
        char incomingData[INCOMING_DATA_LEN];
        std::string arguments = std::to_string(mode);
        if (sendCommandToDevice(0, arguments, incomingData) == false) {  // RESET command is always 0 for Haptic Avatar devices.
            return -1;
        }

        int res = std::atoi(incomingData);
        return res;

        setupNumReturnVals();  // needs to be implemented in each device driver
        setupCmdLists();   // needs to be implemented in each device driver

    }

    bool HapticAvatar_DriverBase::sendCommandToDevice(int commandId, const std::string& arguments, char* result)
    {
        std::string fullCommand = std::to_string(commandId) + " " + arguments + " \n";
        //std::cout << "fullCommand: '" << fullCommand << "'" << std::endl;
        char outgoingData[OUTGOING_DATA_LEN];
        strcpy(outgoingData, fullCommand.c_str());

        unsigned int outlen = (unsigned int)(strlen(outgoingData));
        bool write_success = writeDataImpl(outgoingData, outlen);
        if (!write_success) {
            return false;
        }

        // request data
        if (result != nullptr)
            getDataImpl(result, false);

        return true;
    }


    std::string HapticAvatar_DriverBase::convertSingleData(char* buffer, bool forceRemoveEoL)
    {
        std::string res = std::string(buffer);
        if (forceRemoveEoL)
            res.pop_back();
        else if (res.back() == '\n')
            res.pop_back();

        while (res.back() == ' ' || res.back() == '\n')
        {
            res.pop_back();
        }

        return res;
    }




    ///////////////////////////////////////////////////////////////
    /////      Internal Methods for device communication      /////
    ///////////////////////////////////////////////////////////////

    void HapticAvatar_DriverBase::connectDevice()
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
                msg_error("HapticAvatar_DriverBase") << "Handle was not attached. Reason: " << m_portName << " not available.";
            }
            else
            {
                msg_error("HapticAvatar_DriverBase") << "Unknown error occured!";
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
                msg_warning("HapticAvatar_DriverBase") << "Failed to get current serial parameters!";
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
                    msg_warning("HapticAvatar_DriverBase") << "ALERT: Could not set Serial Port parameters";
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


    int HapticAvatar_DriverBase::getDataImpl(char* buffer, bool do_flush)
    {
        bool response = false;
        int cptSecu = 0;
        int n = 0;
        int que = 0;
        char* pch;
        int num_cr = 0;
        while (!response && cptSecu < 10000)
        {
            n = readDataImpl(buffer, INCOMING_DATA_LEN, &que, do_flush);
            if (n > 0)
            {
                // count the number of \n in the return string
                pch = strchr(buffer, '\n');
                while (pch != NULL)
                {
                    num_cr++;
                    pch = strchr(pch + 1, '\n');
                }
                response = true;
            }

            cptSecu++;
        }

        if (!response) // secu loop reach end
        {
            std::cerr << "## Error getData no message returned. Reach security loop limit: " << cptSecu << std::endl;
            return -1;
        }

        return num_cr;
    }


    int HapticAvatar_DriverBase::readDataImpl(char* buffer, unsigned int nbChar, int* queue, bool do_flush)
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
    }


    bool HapticAvatar_DriverBase::writeDataImpl(char* buffer, unsigned int nbChar)
    {
        DWORD bytesSend;

        //Try to write the buffer on the Serial port
        if (!WriteFile(m_hSerial, (void*)buffer, nbChar, &bytesSend, 0))
        {
            //In case it don't work get comm error and return false
            ClearCommError(m_hSerial, &m_errors, &m_status);

            std::cerr << "Error failed to send command: '" << buffer << "'. Error returned: " << m_errors << std::endl;
            return false;
        }
        else
            return true;
    }


    void HapticAvatar_DriverBase::update()
    {
        if (m_connected) {
            // first, receive the data from the previously sent commands

            updateReceive();

            std::string send_string; 

            // fill the cmd_send_list again, starting with the cmd_always list  based on the subscription
            for (int k = 0; k < device_num_cmds; k++) {
                if (update_cmd_every_nth[k] > 0) {
                    if ((send_counter % update_cmd_every_nth[k]) == 0) {
                        cmd_send_list[cmd_send_list_size++] = k;
                        expected_num_return_vals += num_return_vals[k];
                        send_string += std::to_string(k) + " ";
                    }
                }
            }

            // add the appended commands, if any
            if (cmd_appended_size > 0) {
                for (int k = 0; k < cmd_appended_size; k++) {
                    cmd_send_list[cmd_send_list_size++] = cmd_appended[k];
                    expected_num_return_vals += num_return_vals[cmd_appended[k]];
                }
                send_string += cmd_appended_str;
            }

            // terminate the send string
            send_string += " \n";
 
            if (cmd_send_list_size > 0) {
                // Send the total command string to the device.
                char outgoingData[OUTGOING_DATA_LEN];
                strcpy(outgoingData, send_string.c_str());
                //std::cout << "Call from update 2. Device type " << std::to_string(device_type) << " string length (" << send_string.size() << ") " << send_string << std::endl;
                
                //if (device_type == 2)
                //    std::cout << "Call from update. Device type " << std::to_string(device_type) << " string length (" << send_string.size() << ") " << outgoingData << "END" << std::endl;

                unsigned int outlen = send_string.size(); // (unsigned int)(strlen(outgoingData));
                bool write_success = writeDataImpl(outgoingData, outlen);
                if (!write_success) {
                    msg_warning("HapticAvatar_DriverBase") << "Write to device type " << std::to_string(device_type) << " failed.";
                }
            }

            // Clear the appended list and string
            cmd_appended_str.clear();
            cmd_appended_size = 0;

            send_counter++;
        }
    }

    void HapticAvatar_DriverBase::updateReceive()
    {
        if (expected_num_return_vals > 0) {  // expected_num_return_vals is determined from the previous sent command set.
            getDataImpl(incomingData, false);
            //if (device_type == 1)
                //std::cout << "Received data from device " << std::to_string(device_type) << ": " << incomingData << std::endl;
            parseMessage();
        }
        // now that the previous command is parsed, we can clear it
        expected_num_return_vals = 0;
        cmd_send_list_size = 0;
        //send_string.clear();
    }

    void HapticAvatar_DriverBase::parseMessage()
    {
        // Parse and sort the incoming data into the result table, which is a two dimensional float array
        char* pEnd = incomingData;
        for (int k = 0; k < cmd_send_list_size; k++) {
            for (int i = 0; i < num_return_vals[cmd_send_list[k]]; i++) {
                result_table[cmd_send_list[k]][i] = std::strtof(pEnd, &pEnd) / scale_factor[cmd_send_list[k]];
                //if (device_type == 1)
                    //std::cout << "Port receive data " << std::to_string(result_table[cmd_send_list[k]][i]); 

            }
        }
        //if (device_type == 1)
        //    std::cout << std::endl;

    }

    void HapticAvatar_DriverBase::appendCmd(int cmd, const char* args)
    {
        cmd_appended[cmd_appended_size++] = cmd;
        cmd_appended_str += std::to_string(cmd) + " " + args;
    }

    void HapticAvatar_DriverBase::updateIfUnsubscribed(int cmd)
    {
        if (update_cmd_every_nth[cmd] == 0) {
            appendCmd(cmd, "");
            update(); // Read away any existing return data and request the data with cmd 
            update(); // Read the data from this request. The data ends up in the results_table.
        }
    }


    std::string HapticAvatar_DriverBase::getDeviceType()
    {
        // Use this command only when to determine which type of device you are communicating with.
        char incoming_str[100];
        sendCommandToDevice(1, "", incoming_str);  // GET_DEVICE_TYPE command is number 1 on all Haptic Avatar devices
        return convertSingleData(incoming_str);
    }

    void HapticAvatar_DriverBase::subscribeTo(int cmd, int every_nth)
    {
        update_cmd_every_nth[cmd] = every_nth;
    }

    sofa::type::fixed_array<float, 6> HapticAvatar_DriverBase::getFloat6(int cmd)
    {
        updateIfUnsubscribed(cmd);
        sofa::type::fixed_array<float, 6> results;
        for (unsigned int i = 0; i < results.size(); i++) {
            results[i] = result_table[cmd][i];
        }
        return results;
    }

    sofa::type::fixed_array<float, 4> HapticAvatar_DriverBase::getFloat4(int cmd)
    {
        updateIfUnsubscribed(cmd);
        sofa::type::fixed_array<float, 4> results;
        for (unsigned int i = 0; i < results.size(); i++) {
            results[i] = result_table[cmd][i];
        }
        return results;
    }

    sofa::type::fixed_array<float, 3> HapticAvatar_DriverBase::getFloat3(int cmd)
    {
        updateIfUnsubscribed(cmd);
        sofa::type::fixed_array<float, 3> results;
        for (unsigned int i = 0; i < results.size(); i++) {
            results[i] = result_table[cmd][i];
        }
        return results;
    }

    float HapticAvatar_DriverBase::getFloat(int cmd)
    {
        updateIfUnsubscribed(cmd);
        return result_table[cmd][0];
    }

    float HapticAvatar_DriverBase::getFloat(int cmd, int channel)
    {
        updateIfUnsubscribed(cmd);
        if (channel >= 0 && channel < RESULT_SIZEY)
            return result_table[cmd][channel];
        else
            return 0;
    }


    int HapticAvatar_DriverBase::getInt(int cmd)
    {
        updateIfUnsubscribed(cmd);
        return (int)result_table[cmd][0];
    }
    int HapticAvatar_DriverBase::getInt(int cmd, int channel)
    {
        updateIfUnsubscribed(cmd);
        if (channel >= 0 && channel < RESULT_SIZEY)
            return (int)result_table[cmd][channel];
        else
            return 0;
    }

   sofa::type::fixed_array<int, 4> HapticAvatar_DriverBase::getInt4(int cmd)
    {
        updateIfUnsubscribed(cmd);
        sofa::type::fixed_array<int, 4> results;
        for (unsigned int i = 0; i < results.size(); i++) {
            results[i] = (int)result_table[cmd][i];
        }
        return results;
    }
    sofa::type::fixed_array<int, 6> HapticAvatar_DriverBase::getInt6(int cmd)
    {
        updateIfUnsubscribed(cmd);
        sofa::type::fixed_array<int, 6> results;
        for (unsigned int i = 0; i < results.size(); i++) {
            results[i] = (int)result_table[cmd][i];
        }
        return results;
    }


    void HapticAvatar_DriverBase::appendIntFloat(int cmd, int chan, float value)
    {
        std::string arguments;
        arguments = std::to_string(chan) + " " + 
            std::to_string(int(value * scale_factor[cmd])) + " ";
        appendCmd(cmd, arguments.c_str());
    }
    void HapticAvatar_DriverBase::appendIntFloat(int cmd, int chan, float value1, float value2)
    {
        std::string arguments;
        arguments = std::to_string(chan) + " " + 
            std::to_string(int(value1 * scale_factor[cmd])) + " " + 
            std::to_string(int(value2 * scale_factor[cmd])) + " ";
        appendCmd(cmd, arguments.c_str());
    }
    void HapticAvatar_DriverBase::appendIntFloat(int cmd, int value, sofa::type::fixed_array<float, 3> values)
    {
        std::string arguments;
        arguments = std::to_string(value) + " " +
            std::to_string(int(values[0] * scale_factor[cmd])) + " " +
            std::to_string(int(values[1] * scale_factor[cmd])) + " " +
            std::to_string(int(values[2] * scale_factor[cmd])) + " ";
        appendCmd(cmd, arguments.c_str());
    }

    void HapticAvatar_DriverBase::appendInt(int cmd, int value)
    {
        std::string arguments;
        arguments = std::to_string(value) + " ";
        appendCmd(cmd, arguments.c_str());
    }

    void HapticAvatar_DriverBase::appendInt(int cmd, int value1, int value2)
    {
        std::string arguments;
        arguments = std::to_string(value1) + " " +
            std::to_string(value2) + " ";
        appendCmd(cmd, arguments.c_str());
    }

    void HapticAvatar_DriverBase::appendFloat(int cmd, float value)
    {
        std::string arguments;
        arguments = std::to_string(value) + " ";
        appendCmd(cmd, arguments.c_str());
    }
    void HapticAvatar_DriverBase::appendFloat(int cmd, sofa::type::fixed_array<float, 4> values)
    {
        std::string arguments;
        for (unsigned int i = 0; i < values.size(); i++)
        {
            int value = int(values[i] * scale_factor[cmd]);
            arguments += std::to_string(value) + " ";
        }

        appendCmd(cmd, arguments.c_str());
    }
    void HapticAvatar_DriverBase::appendFloat(int cmd, float f1, float f2, float f3, float f4)
    {
        std::string arguments;
        arguments = std::to_string(int(f1 * scale_factor[cmd])) + " " +
            std::to_string(int(f2 * scale_factor[cmd])) + " " +
            std::to_string(int(f3 * scale_factor[cmd])) + " " +
            std::to_string(int(f4 * scale_factor[cmd])) + " ";
        appendCmd(cmd, arguments.c_str());
    }

    void HapticAvatar_DriverBase::appendFloat(int cmd, sofa::type::fixed_array<float, 6> values)
    {
        std::string arguments;
        for (unsigned int i = 0; i < values.size(); i++)
        {
            int value = int(values[i] * scale_factor[cmd]);
            arguments += std::to_string(value) + " ";
        }

        appendCmd(cmd, arguments.c_str());
    }

}