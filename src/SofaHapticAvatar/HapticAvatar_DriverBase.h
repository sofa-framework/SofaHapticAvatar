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
#pragma once

#include <SofaHapticAvatar/config.h>
//#include <SofaHapticAvatar/HapticAvatar_Defines.h>
#include <sofa/defaulttype/Vec.h>
#include <string>

namespace sofa::HapticAvatar
{

#define OUTGOING_DATA_LEN 1024
#define INCOMING_DATA_LEN 1024
#define NBJOINT 6
#define ARDUINO_WAIT_TIME 2000
#define RESULT_SIZEX  52
#define RESULT_SIZEY  12

    /**
    * HapticAvatar driver
    */
    class SOFA_HAPTICAVATAR_API HapticAvatar_DriverBase
    {
    public:
        HapticAvatar_DriverBase(const std::string& portName);

        virtual ~HapticAvatar_DriverBase();

        bool IsConnected() { return m_connected; }


        /** Reset encoders, motor outputs, calibration flags, collision objects.
        * Command: RESET
        * @param {int} mode: specify what to reset. See doc.
        */
        int resetDevice(int mode);
        virtual int getSerialNumber() = 0;
        std::string getDeviceType();



        /** Generic method which will format the command and send it to the device using HapticAvatar::Cmd and list of arguments given as input. Result will be stored in input char* result if not null.
        * @param {HapticAvatar::Cmd} command: the command enum to be sent.
        * @param {string} arguments: already formatted list of arguments to be sent with the command.
        * @param {char *} result: if not null, response will be asked to device and stored in this char*.
        * @returns {bool} true if command success otherwise false before getting result.
        */
        bool sendCommandToDevice(int commandId, const std::string& arguments, char* result);


        /// Will convert a char* array response into std::string while removing end of line and space at end.
        std::string convertSingleData(char* buffer, bool forceRemoveEoL = false);

        /// Read data from device and send new commands to device
        void update();

  
    protected:
        /// Internal method to connect to device
        void connectDevice();

        /** Internal method to really to get response from the device. Will be looping while calling @sa ReadDataImplLooping with a security of 10k loop.
        * @param {char *} buffer: array to store the response.
        * @param {bool} do_flush: to flush after getting response.
        */
        int getDataImpl(char* buffer, bool do_flush);

        /** Internal low level method to really do the job of getting a response from the device
        * @param {char *} buffer: array to store the response.
        * @param {uint} nbChar: size of the command array
        * @param {int *} queue: queue size to be read.
        * @param {bool} do_flush: to flush after getting response.
        */
        int readDataImpl(char* buffer, unsigned int nbChar, int* queue, bool do_flush);

        /** Internal low level method to really do the job of sending a command to the device.
        * @param {char *} buffer: full command as an array.
        * @param {uint} nbChar: size of the command array
        */
        bool writeDataImpl(char* buffer, unsigned int nbChar);

        void updateReceive();

        int device_num_cmds = 0;  // must be set
        int num_return_vals[RESULT_SIZEX] = { 0 }; // Shall be initialized with the number of return values from each request command
        float scale_factor[RESULT_SIZEX] = { 1.0f }; // Data from devices are sent as integers. This array shall be initialized with the convertion factors back to float.
        float result_table[RESULT_SIZEX][RESULT_SIZEY]; // A table that contains the latest data from a device.
        int update_cmd_every_nth[RESULT_SIZEX] = { 0 };
        
        char incomingData[INCOMING_DATA_LEN];

        int cmd_appended[100];  // A list of commands that is appended based on events in the simulation, such as forces, turning force feedback on/off etc.
        int cmd_appended_size = 0;
        int cmd_appended_num_return_vals = 0;
        std::string cmd_appended_str; 

        int cmd_send_list[100];  // the list of all commands to be sent
        int cmd_send_list_size = 0;
        //std::string send_string;
        int expected_num_return_vals = 0;
        int send_counter = 0; // 

        void subscribeTo(int cmd, int every_nth);
        void parseMessage();
        void appendCmd(int cmd, const char* args);
        void updateIfUnsubscribed(int cmd);

        float getFloat(int cmd);
        float getFloat(int cmd, int channel);
        sofa::type::fixed_array<float, 3> getFloat3(int cmd);
        sofa::type::fixed_array<float, 4> getFloat4(int cmd);
        sofa::type::fixed_array<float, 6> getFloat6(int cmd);

        int getInt(int cmd);
        int getInt(int cmd, int channel);
        sofa::type::fixed_array<int, 4> getInt4(int cmd);
        sofa::type::fixed_array<int, 6> getInt6(int cmd);

        void appendWithChan(int cmd, int chan, float value);
        void appendWithChan(int cmd, int chan, float value1, float value2);
        void appendWithChan(int cmd, int chan, int value);
        void appendInt(int cmd, int value);
        void appendFloat4(int cmd, sofa::type::fixed_array<float, 4> values);
        void appendFloat4(int cmd, float f1, float f2, float f3, float f4);
        void appendFloat6(int cmd, sofa::type::fixed_array<float, 6> values);

        int device_type = 0;

    private:
        //Connection status
        bool m_connected;

        //Serial comm handler
        HANDLE m_hSerial;
        //Get various information about the connection
        COMSTAT m_status;
        //Keep track of last error
        DWORD m_errors;

        // String name of the port (ex: COM3)
        std::string m_portName;
    };
};
