/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_RigidGrasperDeviceController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

int HapticAvatar_RigidGrasperDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_RigidGrasperDeviceController >()
    ;


//constructeur
HapticAvatar_RigidGrasperDeviceController::HapticAvatar_RigidGrasperDeviceController()
    : HapticAvatar_RigidDeviceController()
    , d_newMethod(initData(&d_newMethod, false, "newMethod", "Parameter to choose old/new method"))
    , l_iboxCtrl(initLink("iboxController", "link to IBoxController"))
    , m_distance(1.0f)
    , m_iboxCtrl(nullptr)    
    , m_detectionNP(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    HapticAvatar_RigidGrasperDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
}



//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_RigidGrasperDeviceController::initImpl()
{
    m_intersectionMethod = getContext()->get<core::collision::Intersection>();
    m_detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();
    if (m_intersectionMethod == nullptr) { msg_error() << "m_intersectionMethod not found. Add an Intersection method in your scene.";}
    if (m_detectionNP == nullptr) { msg_error() << "NarrowPhaseDetection not found. Add a NarrowPhaseDetection method in your scene.";}
    
    SReal alarmDist = m_intersectionMethod->getAlarmDistance();
    SReal contactDist = m_intersectionMethod->getContactDistance();
    m_distance = alarmDist - contactDist;

    // get ibox if one
    if (!l_iboxCtrl.empty())
    {
        m_iboxCtrl = l_iboxCtrl.get();
        if (m_iboxCtrl != nullptr)
        {
            msg_info() << "Device " << d_hapticIdentity.getValue() << " connected with IBox: " << m_iboxCtrl->d_hapticIdentity.getValue();
        }
    }


    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }


}


bool HapticAvatar_RigidGrasperDeviceController::createHapticThreads()
{   
    m_terminate = false;
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

    return true;
}


void HapticAvatar_RigidGrasperDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;

    HapticAvatar_RigidGrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_RigidGrasperDeviceController*>(p_this);
    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);

    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_RigidGrasperDeviceController cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_Driver cast failed");
        return;
    }

    // Loop Timer
    long targetSpeedLoop = 1; // Target loop speed: 1ms
    
    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());
    
    ctime_t lastTime = CTime::getRefTime();
    std::cout << "start time: " << lastTime << " speed: " << speedTimerMs << std::endl;
    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    int cptLoop = 0;

    bool debugThread = _deviceCtrl->d_dumpThreadInfo.getValue();
    bool newMethod = _deviceCtrl->d_newMethod.getValue();
    SReal damping = _deviceCtrl->m_forceScale.getValue();

    int cptF = 0;
    // Haptics Loop
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();
        //_deviceCtrl->m_hapticData.collisionForces = _driver->getLastCollisionForce();

        // get info regarding jaws
        //float jtorq = _driver->getJawTorque();
        if (_deviceCtrl->m_iboxCtrl)
        {
            float angle = _deviceCtrl->m_iboxCtrl->getJawOpeningAngle();
            _deviceCtrl->m_hapticData.jawOpening = angle;
        }

#if 1
        if (_deviceCtrl->m_simulationStarted && !_deviceCtrl->contactsHaptic.empty())
        {
            sofa::defaulttype::Vector3 totalForce = sofa::defaulttype::Vector3(0, 0, 0);

            for (auto contact : _deviceCtrl->contactsHaptic)
            {
                totalForce += contact.m_force * contact.distance;
            }

            if (newMethod)
            {
                sofa::defaulttype::Quat rotRot = sofa::defaulttype::Quat::fromEuler(0.0f, _deviceCtrl->m_hapticData.anglesAndLength[Dof::ROT], 0.0f);
                sofa::defaulttype::Mat4x4f R_rot = sofa::defaulttype::Mat4x4f::transformRotation(rotRot);
                sofa::defaulttype::Mat3x3f rotM;
                for (unsigned int i = 0; i < 3; i++) {
                    for (unsigned int j = 0; j < 3; j++) {
                        rotM[i][j] = R_rot[i][j];
                    }
                }

                Vec3 toolDir = Vec3(0, -1, 0);
                toolDir = _deviceCtrl->m_toolRot * toolDir;
                toolDir.normalize();
                

                Vec3 yawDir = Vec3(1, 0, 0);
                yawDir = _deviceCtrl->m_PortalRot * yawDir;
                yawDir.normalize();

                Vec3 pitchDir = Vec3(0, 0, 1);
                Vec3 pitchDirTool = Vec3(0, 0, 1);
                pitchDir = _deviceCtrl->m_PortalRot * pitchDir;
                pitchDirTool = _deviceCtrl->m_toolRot * pitchDirTool;
                pitchDir.normalize();
                
                totalForce = totalForce * damping;
                //Vec3 totalForceTM = _deviceCtrl->m_toolRotInv * totalForce;
                Vec3 h = cross(totalForce, toolDir);
                

                SReal zforce = dot(-toolDir, totalForce);
                SReal yawTorque = dot(yawDir, h);
                SReal pitchTorque = dot(pitchDir, h);                

                
                const HapticAvatar_RigidGrasperDeviceController::VecCoord& toolPosition = _deviceCtrl->d_toolPosition.getValue();
                Vec3 centerTool = toolPosition[3].getCenter();
                //Vec3 dirToolAccum = Vec3(0.0, 0.0, 0.0);
                int cpt = 0;
                SReal torqueAcc = 0;
                for (auto contact : _deviceCtrl->contactsHaptic)
                {
                    Vec3 dirPoint = contact.m_toolPosition - centerTool;
                    Vec3 cross1 = cross(-contact.m_normal, dirPoint);
                    torqueAcc += dot(cross1, toolDir);
                    //if (dot(dirPoint, toolDir) > 0)
                    //{
                    //    dirToolAccum += dirPoint;
                    //    cpt++;
                    //}


                }
                //if (cpt != 0)
                //    dirToolAccum = dirToolAccum/cpt;

                //Vec3 hTM = cross(dirToolAccum, toolDir);
                SReal toolTorque = torqueAcc;// dot(hTM, pitchDirTool);
                //_deviceCtrl->m_toolDir = toolDir;
                //_deviceCtrl->m_pitchDir = pitchDirTool;
                //_deviceCtrl->m_h = h;
                //_deviceCtrl->m_hTM = dirToolAccum;
                //_deviceCtrl->m_hTM = cross(toolDir, toolDir);


                if (cptF == 100)
                {
                    Vector3 root = toolPosition[3].getCenter();
                    

                    for (auto contact : _deviceCtrl->contactsHaptic)
                    {
                        SReal res = dot(contact.m_toolPosition - root, root + toolDir);
                        std::cout << res << std::endl;
                    }

                    std::cout << "zforce: " << zforce
                        << " | pitchTorque: " << pitchTorque
                        << " | yawTorque: " << yawTorque
                        << " | toolTorque: " << toolTorque
                        << std::endl;

                    cptF = 0;
                }
                //cptF++;

                //zforce = 0.0;
                //toolTorque = 0.0;
                //pitchTorque = 0.0;
                
                //yawTorque = 0.0;
                _driver->setManual_PWM(float(toolTorque), float(pitchTorque * 50), float(zforce), float(yawTorque * 50));
            }
            else
            {
                _driver->setManualForceVector(_deviceCtrl->m_toolRotInv * totalForce * damping, true);
            }
        }
        else
            _driver->releaseForce();
#endif
        
        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        //if (duration < targetTicksPerLoop)
        //    std::cout << "Need to Wait!!!" << std::endl;
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }

        // timer dump
        cptLoop++;

        if (debugThread && cptLoop % 100 == 0)
        {
            ctime_t stepTime = CTime::getRefTime();
            ctime_t diffLoop = stepTime - lastTime;
            lastTime = stepTime;

            auto t2 = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
            t1 = t2;
            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
        }
    }

    // ensure no force
    _driver->releaseForce();
    std::cout << "Haptics thread END!!" << std::endl;
}


void HapticAvatar_RigidGrasperDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatar_RigidGrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_RigidGrasperDeviceController*>(p_this);
    
    // Use computer tick for timer
    ctime_t targetSpeedLoop = 1/2; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());

    ctime_t lastTime = CTime::getRefTime();
    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    int cptLoop = 0;
    // Haptics Loop
    while (!terminate)
    {
        ctime_t startTime = CTime::getRefTime();
        _deviceCtrl->m_simuData = _deviceCtrl->m_hapticData;

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }


        //if (cptLoop % 100 == 0)
        //{
        //    ctime_t stepTime = CTime::getRefTime();
        //    ctime_t diffLoop = stepTime - lastTime;
        //    lastTime = stepTime;
        //    //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
        //    std::cout << "Copy nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;            
        //}
        cptLoop++;
    }
}

int cpt = 0;

void HapticAvatar_RigidGrasperDeviceController::updatePositionImpl()
{
    if (!m_HA_driver)
        return;

    sofa::defaulttype::Quat orien;
    orien.fromMatrix(m_toolRot);

    // compute bati position
    HapticAvatar_RigidGrasperDeviceController::Coord rootPos = m_portalMgr->getPortalPosition(m_portId);
    rootPos.getOrientation() = orien;

    HapticAvatar_RigidGrasperDeviceController::Coord & posDevice = *d_posDevice.beginEdit();    
    posDevice.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    d_posDevice.endEdit();

    // Update jaws rigid
    float _OpeningAngle = m_simuData.jawOpening * m_jawsData.m_MaxOpeningAngle * 0.01f;
    HapticAvatar_RigidGrasperDeviceController::Coord jawUp;
    HapticAvatar_RigidGrasperDeviceController::Coord jawDown;
    
    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    
    // Update jaws exterimies
    Vec3f posExtrem = Vec3f(0.0, -m_jawsData.m_jawLength, 0.0);
    HapticAvatar_RigidGrasperDeviceController::Coord jawUpExtrem = jawUp;
    HapticAvatar_RigidGrasperDeviceController::Coord jawDownExtrem = jawDown;
        
    jawUpExtrem.getCenter() += jawUpExtrem.getOrientation().rotate(posExtrem);
    jawDownExtrem.getCenter() += jawDownExtrem.getOrientation().rotate(posExtrem);

    // Udpate articulated device
    HapticAvatar_RigidGrasperDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition[0] = rootPos;
    toolPosition[1] = rootPos;
    toolPosition[2] = rootPos;
    toolPosition[3] = posDevice;

    toolPosition[4] = jawUp;
    toolPosition[5] = jawDown;
    toolPosition[6] = jawUpExtrem;
    toolPosition[7] = jawDownExtrem;
    d_toolPosition.endEdit();

   /* if (cpt == 20)
    {
        bool test = false;
        for (int i = 0; i < m_simuData.hapticForces.size(); i++)
        {
            double norm = m_simuData.hapticForces[i].getLinear().norm();
            if (norm > 0.001)
            {
                test = true;
                std::cout << i << " | Position: " << toolPosition[i] << " | force: " << m_simuData.hapticForces[i].getLinear() << " | angular: " << m_simuData.hapticForces[i].getAngular()
                    << " => " << m_simuData.hapticForces[i].getLinear().norm() << std::endl;
            }
            
        }

        if (test)
            std::cout << std::endl;
        cpt = 0;
    }
    cpt++;*/
}


void HapticAvatar_RigidGrasperDeviceController::retrieveCollisions()
{
    contactsSimu.clear();

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = m_detectionNP->getDetectionOutputs();
    if (detectionOutputs.size() == 0) {
        contactsHaptic = contactsSimu;
        return;
    }

    //std::cout << "detectionOutputs: " << detectionOutputs.size() << " count: " << m_detectionNP->getPrimitiveTestCount() << std::endl;    
    const ContactVector* contacts = NULL;
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        contacts = dynamic_cast<const ContactVector*>(it->second);
        if (contacts == nullptr || contacts->size() == 0) {
            continue;
        }

        size_t ncontacts = contacts->size();
        if (ncontacts == 0) {
            continue;
        }
        
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;

        //std::cout << " - collMod1: " << collMod1->getName() << " | collMod2: " << collMod2->getName() << std::endl;

        int id = -1;        
        sofa::core::CollisionModel* toolMod = nullptr;

        if (collMod1->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
        {
            id = 0;
            toolMod = collMod1;
        }
        else if (collMod2->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
        {
            id = 1;
            toolMod = collMod2;
        }
               
        if (id == -1) { // others collision than with tool
            continue;
        }

        int toolId = 0;
        
        //std::cout << " - ncontacts: " << ncontacts << std::endl;
        for (size_t j = 0; j < ncontacts; ++j)
        {
            const ContactVector::value_type& c = (*contacts)[j];
            HapticContact contact;
            if (id == 0)
            {
                contact.m_toolPosition = c.point[0];
                contact.m_objectPosition = c.point[1];
                contact.m_normal = c.normal * (-1); /// Normal of the contact, pointing outward from the first model                
            }
            else
            {
                contact.m_toolPosition = c.point[1];
                contact.m_objectPosition = c.point[0];
                contact.m_normal = c.normal ;
            }
            contact.m_force = (contact.m_toolPosition - contact.m_objectPosition).normalized();
            contact.distance = (m_distance - c.value)*(m_distance - c.value);
            //std::cout << "contact.distance: " << contact.distance << std::endl;

            contactsSimu.push_back(contact);
        }
    }

    contactsHaptic = contactsSimu;
    //std::cout << " - contactsSimu: " << contactsSimu.size() << std::endl;
}


void HapticAvatar_RigidGrasperDeviceController::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        m_simulationStarted = true;
        updatePosition();
    }
    else if (simulation::CollisionEndEvent::checkEventType(event))
    {
        retrieveCollisions();
    }
}

} // namespace sofa::HapticAvatar