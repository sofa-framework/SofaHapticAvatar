/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#include <SofaHapticAvatar/config.h>

namespace sofa
{

namespace component
{

//Here are just several convenient functions to help user to know what contains the plugin

extern "C" {
    SOFA_HAPTICAVATAR_API void initExternalModule();
    SOFA_HAPTICAVATAR_API const char* getModuleName();
    SOFA_HAPTICAVATAR_API const char* getModuleVersion();
    SOFA_HAPTICAVATAR_API const char* getModuleLicense();
    SOFA_HAPTICAVATAR_API const char* getModuleDescription();
    SOFA_HAPTICAVATAR_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return "SofaHapticAvatar";
}

const char* getModuleVersion()
{
    return "0.1";
}

const char* getModuleLicense()
{
    return "LGPL";
}


const char* getModuleDescription()
{
    return "Plugin to manage Haptic Avatar device in SOFA.";
}

const char* getModuleComponentList()
{
    return "HapticAvatarDriver";
}



}

}

