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
#include <sofa/defaulttype/Vec.h>
#include <sofa/type/SVector.h>

#include <string>
#include <mutex>

namespace sofa::HapticAvatar
{
class HapticAvatar_HapticThreadManager;
class HapticAvatar_ArticulatedDeviceController;
class HapticAvatar_IBoxController;

/// Static singleton of HapticThreadManager
static HapticAvatar_HapticThreadManager* s_hapticThread = nullptr;

class SOFA_HAPTICAVATAR_API HapticAvatar_HapticThreadManager
{
public:
    /// Static method to create the HapticThreadManager singleton. Will create it if first time called and return pointer to it.
    static HapticAvatar_HapticThreadManager* getInstance();

    /// Static method to kill the HapticThreadManager singleton if @sa s_hapticThread is not null.
    static void kill();


    /// Main Haptic thread methods
    void Haptics(std::atomic<bool>& terminate, void* p_this);

    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminate;

    /** Method to register a new device inside the vector @m_devices
    * Will check to avoid redundencies and call @sa createHapticThreads
    */
    void registerDevice(HapticAvatar_ArticulatedDeviceController* device);

    /// Method to register the Ibox. Assume only one per scene ?
    void registerIBox(HapticAvatar_IBoxController* ibox);

    /// Method to notify that simulation is running
    void setSimulationStarted() { m_simulationStarted = true; }

    bool logThread = true;
private:
    HapticAvatar_HapticThreadManager();
    ~HapticAvatar_HapticThreadManager();

    /// Internal method to create the haptic thread only once. Will be called each time a device is successfully registered
    void createHapticThreads();

    /// haptic thread c++ object
    std::thread haptic_thread;

    bool hapticLoopStarted = false; ///< Bool to store the information is haptic thread is running or not.
    bool m_simulationStarted = false; ///< Bool to store the information that the simulation is running or not.

    /// Vector of registered device to be updated in the haptic thread loop.
    sofa::type::vector< HapticAvatar_ArticulatedDeviceController*> m_devices;
    /// Pointer to the iBox controller.
    HapticAvatar_IBoxController* m_IBox = nullptr;
};


} // namespace sofa::HapticAvatar


