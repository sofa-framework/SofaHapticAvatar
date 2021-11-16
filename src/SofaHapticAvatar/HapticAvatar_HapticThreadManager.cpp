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

#include <SofaHapticAvatar/HapticAvatar_HapticThreadManager.h>
#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>
#include <SofaHapticAvatar/HapticAvatar_DriverPort.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/thread/CTime.h>
#include <chrono>


namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;


HapticAvatar_HapticThreadManager* HapticAvatar_HapticThreadManager::getInstance()
{
    if (s_hapticThread == nullptr)
    {
        s_hapticThread = new HapticAvatar_HapticThreadManager();
    }
    else if (s_hapticThread->logThread)
    {
        std::cout << "singleton already created!" << std::endl;
    }

    return s_hapticThread;
}

void HapticAvatar_HapticThreadManager::kill()
{
    if (s_hapticThread != nullptr)
    {
        if (s_hapticThread->logThread)
        {
            std::cout << "kill s_hapticThread" << std::endl;
        }

        s_hapticThread->m_devices.clear();        
        delete s_hapticThread;
        s_hapticThread = nullptr;
    }
}

HapticAvatar_HapticThreadManager::HapticAvatar_HapticThreadManager()
    : m_terminate(true)
    , hapticLoopStarted(false)
{

}

HapticAvatar_HapticThreadManager::~HapticAvatar_HapticThreadManager()
{
    if (m_terminate == false)
    {
        m_terminate = true;
        haptic_thread.join();
    }
}



void HapticAvatar_HapticThreadManager::createHapticThreads()
{
    if (hapticLoopStarted)
        return;

    m_terminate = false;
    haptic_thread = std::thread(&HapticAvatar_HapticThreadManager::Haptics, this, std::ref(this->m_terminate), this);
    hapticLoopStarted = true;
}

void HapticAvatar_HapticThreadManager::Haptics(std::atomic<bool>& terminate, void* p_this)
{
    if (logThread)
        std::cout << "Main Haptics thread created" << std::endl;
    
    HapticAvatar_HapticThreadManager* threadMgr = static_cast<HapticAvatar_HapticThreadManager*>(p_this);

    // Loop Timer
    long targetSpeedLoop = 1; // Target loop speed: 1ms

    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;

    int cptLoop = 0;
    ctime_t startTimePrev = CTime::getRefTime();
    ctime_t summedLoopDuration = 0;
    
    while (!terminate)
    {
        ctime_t startTime = CTime::getRefTime();
        summedLoopDuration += (startTime - startTimePrev);
        startTimePrev = startTime;

        // loop over the devices
        for (auto device : m_devices) // mutex?
        {
            HapticAvatar_DriverPort* _driver = device->getHapticDriver();

            device->haptic_updateArticulations(m_IBox);

            // Force feedback computation
            if (m_simulationStarted)
            {
                device->haptic_updateForceFeedback(m_IBox);
            }

            _driver->update();

            if (m_IBox != nullptr)
            {
                m_IBox->update();
            }
        }

        if (logThread)
        {
            cptLoop++;
            if (cptLoop % 1000 == 0) {
                float updateFreq = 1000 * 1000 / ((float)summedLoopDuration / (float)refTicksPerMs); // in Hz
                std::cout << "DeviceName: " << " | Iteration: " << cptLoop << " | Average haptic loop frequency " << std::to_string(int(updateFreq)) << std::endl;
                summedLoopDuration = 0;
            }
        }


        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        duration = 0;
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }
    }

    std::cout << "Haptics thread END!!" << std::endl;
}




void HapticAvatar_HapticThreadManager::registerDevice(HapticAvatar_ArticulatedDeviceController* device)
{
    bool found = false;
    for (auto _device : m_devices)
    {
        if (_device == device)
        {
            found = true;
            break;
        }
    }


    if (!found)
    {
        m_devices.push_back(device);
        createHapticThreads();
    }
    else
    {
        msg_error("HapticAvatar_HapticThreadManager") << "Device: " << device->name.getValue() << " already registered in haptic thread.";
    }
}


void HapticAvatar_HapticThreadManager::registerIBox(HapticAvatar_IBoxController* ibox)
{
    m_IBox = ibox;
}

} // namespace sofa::HapticAvatar
