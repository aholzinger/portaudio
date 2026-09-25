cwASIO-README.txt

This document contains information to help you compile PortAudio with 
ASIO support via cwASIO. If you find any omissions or errors in this
document please notify us on GitHub.


Building PortAudio with ASIO support via cwASIO
-----------------------------------------------

Only CMake is supported as build resp. configure system.

cwASIO support is controlled by the CMake option PA_USE_CWASIO, which is
ON by default. cwASIO does not need the ASIO SDK. PA_USE_CWASIO is
independent of PA_USE_ASIO: enabling cwASIO does not disable the ASIO
host API.


###
