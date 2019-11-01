/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatarDriver.h>
#include <SofaHapticAvatar/HapticAvatarDefines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace controller
{

//constructeur
HapticAvatarDriver::HapticAvatarDriver()
    : d_positionBase(initData(&d_positionBase, Vec3d(0, 0, 0), "positionBase", "Position of the interface base in the scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the interface base in the scene world coordinates"))
    , d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Phantom Coordinates"))
    , d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , d_drawDevice(initData(&d_drawDevice, false, "drawDevice", "draw device"))
    , d_portName(initData(&d_portName, std::string("//./COM3"),"portName", "position of the base of the part of the device"))
{
    this->f_listening.setValue(true);

}


HapticAvatarDriver::~HapticAvatarDriver()
{
    clearDevice();
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatarDriver::init()
{
    msg_info() << "HapticAvatarDriver::init()";
    m_HA_API = new HapticAvatarAPI(d_portName.getValue());

    if (!m_HA_API->IsConnected())
        return;

    char incomingData[INCOMING_DATA_LEN];
    char flushData[512]; // don't forget to pre-allocate memory
    int dataLength = 512;
    int que = 0;
    unsigned int num_read_bytes = 0, failed_to_send_times = 0, loop_num = 0, num_reads = 0;
    char outgoingData[OUTGOING_DATA_LEN] = "2 2 2 2 2 2 \n";
    char outgoingData2[OUTGOING_DATA_LEN];
    for (int k = 0; k < OUTGOING_DATA_LEN - 1; k++)
        outgoingData2[k] = '0';
    outgoingData2[OUTGOING_DATA_LEN - 1] = 'X';

    // flush the read buffer
    bool read_flag = true;
    while (read_flag) {
        int n = m_HA_API->ReadData(flushData, 512, &que, true);
        num_read_bytes += n;
        read_flag = (n > 0);
        //Sleep(1);
        num_reads++;
    }

    int outlen = strlen(outgoingData);
    bool write_success = m_HA_API->WriteData(outgoingData, outlen);
    if (!write_success) {
        failed_to_send_times++;
        std::cout << "failed_to_send_times" << std::endl;
    }

    int n = 0;
    n = m_HA_API->ReadData(incomingData, INCOMING_DATA_LEN, &que, false);
    std::cout << "ReadData: " << n << std::endl;
}


void HapticAvatarDriver::clearDevice()
{
    msg_info() << "HapticAvatarDriver::clearDevice()";
}


void HapticAvatarDriver::bwdInit()
{
    msg_info() << "HapticAvatarDriver::bwdInit()";
}


void HapticAvatarDriver::reinit()
{
    msg_info() << "HapticAvatarDriver::reinit()";
}

void HapticAvatarDriver::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!d_drawDevice.getValue())
        return;

    //vparams->drawTool()->saveLastState();
    //vparams->drawTool()->restoreLastState();
}


void HapticAvatarDriver::handleEvent(core::objectmodel::Event *event)
{
    //if(m_errorDevice != 0)
    //    return;

    //if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    //{
    //    if (m_hStateHandles.size() && m_hStateHandles[0] == HD_INVALID_HANDLE)
    //        return;

    //    updatePosition();
    //}
}

int HapticAvatarDriverClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarDriver >()
;

} // namespace controller

} // namespace component

} // namespace sofa
