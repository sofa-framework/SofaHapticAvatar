# SOFA plugin for Haptic Avatar device

[![Documentation](https://img.shields.io/badge/doc-in_pdf-green.svg)](https://github.com/sofa-framework/SofaHapticAvatar/blob/master/doc/SofaHapticAvatar%20plugin%20documentation.pdf)
[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions)
[![Gitter](https://img.shields.io/badge/chat-on_Gitter-ff69b4.svg)](https://app.gitter.im/#/room/#sofa-framework:gitter.im)
[![Follou](https://img.shields.io/badge/Follou-to_website-red.svg)](https://www.follou-haptics.com/)

## Description

Haptic Avatar is a haptic device from [Follou Haptics](https://www.follou-haptics.com/) for minimally invasive surgical training, e.g. laparoscopy, arthroscopy and thoracoscopy. 
It has four degrees of freedom, all with force feedback, and a form factor that enables multiple and adjustable portal placement suitable for both individual and team training.
It features a port-like part where a representation of a surgical tool can be inserted and retracted. The tool will be detected and identified at insertion.

<img align="center" width="60%" height="auto" src="./doc/Device_picture.png">

## Features
The real device is represented virtually in SOFA using an ArticulatedSystemMapping to define the different articulation of the device. Each articulation represents either a single rotation or a translation in one direction.
Very quickly, during the simulation, in addition to the simulation thread. A first thread is created to communicate with the Haptic device at high frequency. It is used to retrieve the tool information and send the force feedback using SOFA lCPForceFeedback mechanism. 
Then a second thread running as well at high frequency is used to copy the tool information into a data set which will be used by the simulation thread.
<img align="center" width="60%" height="auto" src="./doc/HAvatar_Articulated_Nodes.png">
<br>
<br>

Quick Description: 
- Compatible with SOFA v21.12, v22.06 on Windows only. 
- Support multiple tools
- Multiple demo scenes for 1 and 2 devices in XML and python
- Full documentation in [./doc](https://github.com/sofa-framework/SofaHapticAvatar/blob/master/doc/SofaHapticAvatar%20plugin%20documentation.pdf)

<img align="center" width="60%" height="auto" src="./doc/HapticAvatar_dual_tool.gif">


## Installation
This plugin should be added as an external plugin of SOFA using the CMAKE_EXTERNAL_DIRECTORIES CMake variable of SOFA. 
See SOFA documentation for more information

## License
This work is dual-licensed under either [LGPL](https://github.com/sofa-framework/SofaHapticAvatar/blob/main/LICENSE.md)

