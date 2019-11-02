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
#ifndef SOFA_HAPTICAVATAR_PORTALMANAGER_H
#define SOFA_HAPTICAVATAR_PORTALMANAGER_H

#include <SofaHapticAvatar/config.h>
#include <SofaHapticAvatar/PortalController.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <string>

class TiXmlElement;

namespace sofa 
{

namespace component 
{

namespace controller 
{

   
/**
* PortalManager 
*/
class SOFA_HAPTICAVATAR_API PortalManager : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(PortalManager, sofa::core::objectmodel::BaseObject);
    PortalManager();

    virtual ~PortalManager() {}

    void setFilename(std::string f);
    const std::string &getFilename();

    virtual void init() override;

    virtual void reinit() override;
    virtual void handleEvent(core::objectmodel::Event *) override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void printInfo();

    sofa::core::objectmodel::DataFileName m_configFilename;

protected:
    bool parseConfigFile();
    bool getIntAttribute(const TiXmlElement* elem, const char* attributeN, int* value);
    bool getFloatAttribute(const TiXmlElement* elem, const char* attributeN, float* value);
private:
    sofa::helper::vector<PortalController* > m_portals;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HAPTICAVATAR_PORTALMANAGER_H
