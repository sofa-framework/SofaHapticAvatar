# SOFA plugin for Haptic Avatar device

[![Documentation](https://img.shields.io/badge/doc-on_website-green.svg)](https://github.com/sofa-framework/SofaHapticAvatar/blob/master/doc/SofaHapticAvatar%20plugin%20documentation.pdf)
[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions)
[![Gitter](https://img.shields.io/badge/chat-on_Gitter-ff69b4.svg)](https://app.gitter.im/#/room/#sofa-framework:gitter.im)

## Description

Haptic Avatar is a haptic device from [Follou Haptics](https://www.follou-haptics.com/) for minimally invasive surgical training, e.g. laparoscopy, arthroscopy and thoracoscopy. 
It has four degrees of freedom, all with force feedback, and a form factor that enables multiple and adjustable portal placement suitable for both individual and team training.

<img align="center" width="70%" height="auto" src="./doc/Device_picture.png">



### Features
https://github.com/
- Compatible with SOFA v21.12, v22.06 on Windows only. 
- Support multiple tools
- Multiple demo scenes for 1 and 2 devices in XML and python

Modelized in SOFA using Articulated System Mapping mechanism
<img align="center" width="80%" height="auto" src="./doc/HAvatar_Articulated_Nodes.png">

## Installation
This plugin should be added as an external plugin of SOFA using the CMAKE_EXTERNAL_DIRECTORIES CMake variable of SOFA. 
See SOFA documentation for more information

## License
This work is dual-licensed under either [LGPL](https://github.com/sofa-framework/SofaHapticAvatar/blob/main/LICENSE.md)

