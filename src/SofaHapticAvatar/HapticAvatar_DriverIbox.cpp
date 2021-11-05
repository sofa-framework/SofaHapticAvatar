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

#include <SofaHapticAvatar/HapticAvatar_DriverIbox.h>
#include <sofa/helper/logging/Messaging.h>

namespace sofa::HapticAvatar
{


    ///////////////////////////////////////////////////////////////
    /////       Methods for specific IBOX communication       /////
    ///////////////////////////////////////////////////////////////

    HapticAvatar_DriverIbox::HapticAvatar_DriverIbox(const std::string& portName)
        : HapticAvatar_DriverBase(portName)
    {
        setupNumReturnVals();  // needs to be implemented in each device driver
        setupCmdLists();   // needs to be implemented in each device driver

        device_type = 2;
    }

    void HapticAvatar_DriverIbox::setupNumReturnVals()
    {
        // num_return_vals = new int[CmdIBox::ALWAYS_LAST];

        num_return_vals[(int)CmdIBox::RESET] = 1;
        num_return_vals[(int)CmdIBox::GET_DEVICE_TYPE] = 1;
        num_return_vals[(int)CmdIBox::GET_OPENING_VALUES] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_HANDLE_IDS] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_PEDAL_STATES] = 2;
        num_return_vals[(int)CmdIBox::SET_ALL_FORCES] = 0;
        num_return_vals[(int)CmdIBox::SET_CHAN_FORCE] = 0;
        num_return_vals[(int)CmdIBox::GET_STATUS] = 1;
        num_return_vals[(int)CmdIBox::GET_CALIBRATION_STATUS] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_MOTOR_BOARD_STATUS] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_BATTERY_VOLTAGE] = 1;
        num_return_vals[(int)CmdIBox::GET_BOARD_TEMP] = 1;
        num_return_vals[(int)CmdIBox::SET_MANUAL_PWM] = 0;
        num_return_vals[(int)CmdIBox::SET_POWER_ON_MANUAL] = 0;
        num_return_vals[(int)CmdIBox::SET_FAN_ON_MANUAL] = 0;
        num_return_vals[(int)CmdIBox::GET_LAST_PWM] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::SET_FF_ENABLE] = 0;
        num_return_vals[(int)CmdIBox::GET_CURRENT_DELTA_T] = 1;
        num_return_vals[(int)CmdIBox::SET_LOOP_GAIN] = 0;
        num_return_vals[(int)CmdIBox::GET_OPTO_FORCES] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::SET_ZERO_FORCE] = 0;
        num_return_vals[(int)CmdIBox::GET_POS_VOLTAGES] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_HANDLE_IDS_REAL] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::GET_BUILD_DATE] = 1;
        num_return_vals[(int)CmdIBox::GET_SERIAL_NUM] = 1;
        num_return_vals[(int)CmdIBox::SET_CHARGE_ENABLE] = 0;
        num_return_vals[(int)CmdIBox::SET_HANDLE_LED] = 0;
        num_return_vals[(int)CmdIBox::GET_OPTO_VOLTAGES] = IBOX_NUM_CHANNELS;
        num_return_vals[(int)CmdIBox::SET_TO_CALIBRATE] = 0;
        num_return_vals[(int)CmdIBox::GET_PART_TEMPERATURES] = 11;
        num_return_vals[(int)CmdIBox::SET_MAX_USB_CHARGE_CURRENT] = 0;
        num_return_vals[(int)CmdIBox::GET_USB_CHARGING_CURRENT] = 1;
        num_return_vals[(int)CmdIBox::GET_CONNECTION_STATES] = 1;
        num_return_vals[(int)CmdIBox::GET_HANDLES_ACTIVITY] = 1;
        num_return_vals[(int)CmdIBox::SET_FORCE_OFFSET] = 0;

        // Set all scale factor to 1.0 to begin with ...
        for (int i = 0; i < (int)CmdIBox::ALWAYS_LAST; i++)
            scale_factor[i] = 1.0f;
        // ... and now change those who are not 1.0
        scale_factor[(int)CmdIBox::GET_OPENING_VALUES] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_HANDLE_IDS] = 10000.0f;
        scale_factor[(int)CmdIBox::SET_ALL_FORCES] = 10000.0f;
        scale_factor[(int)CmdIBox::SET_CHAN_FORCE] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_BATTERY_VOLTAGE] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_BOARD_TEMP] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_CURRENT_DELTA_T] = 10000.0f;
        scale_factor[(int)CmdIBox::SET_LOOP_GAIN] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_OPTO_FORCES] = 10000.0f;;
        scale_factor[(int)CmdIBox::GET_POS_VOLTAGES] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_HANDLE_IDS_REAL] = 10000.0f;
        scale_factor[(int)CmdIBox::GET_OPTO_VOLTAGES] = 10000.0f;
        scale_factor[(int)CmdIBox::SET_MAX_USB_CHARGE_CURRENT] = 10000.0f;
        scale_factor[(int)CmdIBox::SET_FORCE_OFFSET] = 10000.0f;

        device_num_cmds = (int)CmdIBox::ALWAYS_LAST;
    }


    void HapticAvatar_DriverIbox::setupCmdLists()
    {
        // Setup the data you want to subscribe to from the port device here. Subscription commands can only be of type GET_... without input arguments.
        // It is good practice to use prime numbers to avoid that all commands are sent at once.
        subscribeTo((int)CmdIBox::GET_OPENING_VALUES, 1);
        subscribeTo((int)CmdIBox::GET_PEDAL_STATES, 11);
        subscribeTo((int)CmdIBox::GET_OPTO_FORCES, 13);
        subscribeTo((int)CmdIBox::GET_CURRENT_DELTA_T, 17);
        subscribeTo((int)CmdIBox::GET_LAST_PWM, 19);
        subscribeTo((int)CmdIBox::GET_STATUS, 1009);
        subscribeTo((int)CmdIBox::GET_CALIBRATION_STATUS, 1013);
        subscribeTo((int)CmdIBox::GET_CONNECTION_STATES, 1019);
        subscribeTo((int)CmdIBox::GET_BOARD_TEMP, 10007);
        subscribeTo((int)CmdIBox::GET_BATTERY_VOLTAGE, 10009);
        subscribeTo((int)CmdIBox::GET_USB_CHARGING_CURRENT, 10037);
        subscribeTo((int)CmdIBox::GET_PART_TEMPERATURES, 10039);
    }

    float HapticAvatar_DriverIbox::getOpeningValue(int toolId)
    {
        return getFloat((int)CmdIBox::GET_OPENING_VALUES, convertToolIdToChannel(toolId));
    }
    void HapticAvatar_DriverIbox::setForce(int toolId, float force)
    {
        appendIntFloat((int)CmdIBox::SET_CHAN_FORCE, convertToolIdToChannel(toolId), force);
    }
    int HapticAvatar_DriverIbox::getStatus()
    {
        return getInt((int)CmdIBox::GET_STATUS);
    }
    int HapticAvatar_DriverIbox::getCalibrationStatus(int toolId)
    {
        return getInt((int)CmdIBox::GET_CALIBRATION_STATUS, convertToolIdToChannel(toolId));
    }
    float HapticAvatar_DriverIbox::getBatteryVoltage()
    {
        return getFloat((int)CmdIBox::GET_BATTERY_VOLTAGE);
    }
    float HapticAvatar_DriverIbox::getBoardTemp()
    {
        return getFloat((int)CmdIBox::GET_BOARD_TEMP);
    }
    int HapticAvatar_DriverIbox::getLastPWM(int toolId)
    {
        return getInt((int)CmdIBox::GET_LAST_PWM, convertToolIdToChannel(toolId));
    }
    void HapticAvatar_DriverIbox::setForceFeedbackEnable(bool on)
    {
        appendInt((int)CmdIBox::SET_FF_ENABLE, int(on));
    }
    float HapticAvatar_DriverIbox::getCurrentDeltaT()
    {
        return getFloat((int)CmdIBox::GET_CURRENT_DELTA_T);
    }
    void HapticAvatar_DriverIbox::setLoopGain(int chan, float loopGainP, float loopGainD)
    {
        appendIntFloat((int)CmdIBox::SET_LOOP_GAIN, chan, loopGainP, loopGainD);
    }
    float HapticAvatar_DriverIbox::getSensedForce(int toolId)
    {
        return getFloat((int)CmdIBox::GET_OPTO_FORCES, convertToolIdToChannel(toolId));
    }
    void HapticAvatar_DriverIbox::setZeroForce(int toolId)
    {
        appendInt((int)CmdIBox::SET_ZERO_FORCE, convertToolIdToChannel(toolId));
    }
    float HapticAvatar_DriverIbox::getPosVoltage(int toolId)
    {
        return getFloat((int)CmdIBox::SET_ZERO_FORCE, convertToolIdToChannel(toolId));
    }
    int HapticAvatar_DriverIbox::getSerialNumber()
    {
        return getInt((int)CmdIBox::GET_SERIAL_NUM);
    }
    float HapticAvatar_DriverIbox::getChargingCurrent()
    {
        return getFloat((int)CmdIBox::GET_USB_CHARGING_CURRENT);
    }
    float HapticAvatar_DriverIbox::getPartTemperature(int part)
    {
        return getFloat((int)CmdIBox::GET_PART_TEMPERATURES, int(part));
    }




    int HapticAvatar_DriverIbox::convertToolIdToChannel(int toolId)
    {
        if (toolId >= 3 && toolId <= 3 + IBOX_NUM_CHANNELS) {
            return toolId - 3;
        }
        else {
            return -1;
        }
    }

    /*

    void HapticAvatar_DriverIbox::setHandleForces(float upperJawForce, float lowerJawForce)
    {
        // TODO convert force into string command

        // add value threashold check if needed
        //float maxPWM = 1000;
        //if (upperJawForce > maxPWM)
        //    upperJawForce = maxPWM;

        //if (lowerJawForce > maxPWM)
        //    lowerJawForce = maxPWM;

        int force = (int)(10000.0f * (upperJawForce - lowerJawForce));
        std::string args;
        args = std::to_string(force)
            + " " + std::to_string(0)
            + " " + std::to_string(0)
            + " " + std::to_string(0)
            + " " + std::to_string(0)
            + " " + std::to_string(0);
        //args = std::to_string(upperJawForce)
        //    +" " + std::to_string(lowerJawForce);

        //std::cout << "setHandleForces command args: " << args << std::endl;

        bool resB = sendCommandToDevice(CmdIBox::SET_IBOX_ALL_FORCES, args, nullptr);
        if (resB == false)
        {
            std::string fullCommand = std::to_string(CmdIBox::SET_IBOX_ALL_FORCES) + " " + args;
            std::cerr << "Error failed to send command: '" << fullCommand << "'" << std::endl;
        }
    }

    void HapticAvatar_DriverIbox::setLoopGain(int loopGain)
    {
        std::string args;
        args = std::to_string(loopGain);
        for (int chan = 0; chan < 6; chan++)
        {
            args = std::to_string(chan) + " " + std::to_string(loopGain) + " " + std::to_string(0);

            std::string fullCommand = std::to_string(CmdIBox::SET_IBOX_LOOP_GAIN) + " " + args;
            std::cout << "LoopGain command args: " << fullCommand << std::endl;

            bool resB = sendCommandToDevice(CmdIBox::SET_IBOX_LOOP_GAIN, args, nullptr);
            if (resB == false)
            {
                std::cerr << "Error failed to send command: '" << fullCommand << "'" << std::endl;
            }
        }
    }
*/
    void HapticAvatar_DriverIbox::printStatus()
    {
        std::cout << "Status for Ibox device S/N " << getSerialNumber() << " at " << getPortName() << std::endl;
        std::cout << "--------------------------------------------" << std::endl;

        std::cout << "  Amplifier board temperature " << getBoardTemp() << "°C" << std::endl;
        std::cout << "  Battery voltage " << getBatteryVoltage() << "V" << std::endl;
        std::cout << "  Charging current " << getChargingCurrent() << "A  (USB draw)" << std::endl;
        unsigned int status = (unsigned int)getStatus();
        //std::cout << "  Yaw calibrated: ";
        //if (status & 0x08 > 0)
        //    std::cout << "Yes     ";
        //else
        //    std::cout << "No      ";
        //std::cout << "Pitch calibrated: ";
        //if (status & 0x02 > 0)
        //    std::cout << "Yes ";
        //else
        //    std::cout << "No  ";
        //std::cout << std::endl;
        //if (status & 0x80 > 0)
        //    std::cout << "  Yaw Amplifier temperature warning!" << std::endl;
        //if (status & 0x20 > 0)
        //    std::cout << "  Pitch Amplifier temperature warning!" << std::endl;
        //if (status & 0x10 > 0)
        //    std::cout << "  Z or Rot Amplifier temperature warning!" << std::endl;
        //if (status & 0x800 > 0)
        //    std::cout << "  Yaw Amplifier undervoltage warning!" << std::endl;
        //if (status & 0x200 > 0)
        //    std::cout << "  Pitch Amplifier undervoltage warning!" << std::endl;
        //if (status & 0x100 > 0)
        //    std::cout << "  Z or Rot Amplifier undervoltage warning!" << std::endl;

        float temp = getPartTemperature((int)IboxThermalSimPart::MotorWinding0);
        std::cout << "  Handle Red winding temperature " << temp << " °C";
        if (temp > 85)
            std::cout << " HIGH, output reduced!";
        std::cout << std::endl;
        temp = getPartTemperature((int)IboxThermalSimPart::MotorWinding1);
        std::cout << "  Handle Orange winding temperature " << temp << " °C";
        if (temp > 85)
            std::cout << " HIGH, output reduced!";
        std::cout << std::endl;
        temp = getPartTemperature((int)IboxThermalSimPart::MotorWinding2);
        std::cout << "  Handle Yellow winding temperature " << temp << " °C";
        if (temp > 85)
            std::cout << " HIGH, output reduced!";
        std::cout << std::endl;
        temp = getPartTemperature((int)IboxThermalSimPart::MotorWinding3);
        std::cout << "  Handle Green winding temperature " << temp << " °C";
        if (temp > 85)
            std::cout << " HIGH, output reduced!";
        std::cout << std::endl << std::endl;

    }
}