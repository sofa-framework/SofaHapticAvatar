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
#include <SofaHapticAvatar/HapticAvatar_Portal.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <string>

class TiXmlElement;

namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;

/**
* HapticAvatar_PortalManager 
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_PortalManager : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(HapticAvatar_PortalManager, sofa::core::objectmodel::BaseObject);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;

    HapticAvatar_PortalManager();

    virtual ~HapticAvatar_PortalManager() {}

    void setFilename(std::string f);
    const std::string &getFilename();

    virtual void init() override;

    virtual void reinit() override;
    virtual void handleEvent(core::objectmodel::Event *) override;

    void updatePostion(int portId, float yawAngle, float pitchAngle);

    void updatePositionData();
    void printInfo();

    int getPortalId(std::string comStr);
    const sofa::defaulttype::Mat4x4f& getPortalTransform(int portId);
    const Coord& getPortalPosition(int portId);

    sofa::core::objectmodel::DataFileName m_configFilename;
    Data< Coord> m_portalPosition1;
    Data< Coord> m_portalPosition2;
    Data< Coord> m_portalPosition3;
    Data< Coord> m_portalPosition4;
    Data< Coord> m_portalPosition5;
protected:
    bool parseConfigFile();
    bool getIntAttribute(const TiXmlElement* elem, const char* attributeN, int* value);
    bool getFloatAttribute(const TiXmlElement* elem, const char* attributeN, float* value);

    void portalsSetup();

private:
    sofa::helper::vector<HapticAvatar_Portal* > m_portals;
};

} // namespace sofa::HapticAvatar
