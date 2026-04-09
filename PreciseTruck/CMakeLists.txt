find_package(Qt5 COMPONENTS Core Network SerialPort REQUIRED)
find_package(MAVSDK REQUIRED)

set(ROOT ${CMAKE_SOURCE_DIR})

add_executable(PreciseTruck
    ${ROOT}/WayWise/autopilot/purepursuitwaypointfollower.cpp
    ${ROOT}/WayWise/autopilot/purepursuitwaypointfollower.h
    ${ROOT}/WayWise/autopilot/waypointfollower.h
    ${ROOT}/WayWise/autopilot/emergencybrake.cpp
    ${ROOT}/WayWise/autopilot/followpoint.cpp
    ${ROOT}/WayWise/communication/jsonstreamparsertcp.cpp
    ${ROOT}/WayWise/communication/vehicleserver.h
    ${ROOT}/WayWise/communication/mavsdkvehicleserver.cpp
    ${ROOT}/WayWise/communication/vehicleconnections/vehicleconnection.cpp
    ${ROOT}/WayWise/communication/mavlinkparameterserver.cpp
    ${ROOT}/WayWise/communication/parameterserver.cpp
    ${ROOT}/WayWise/core/pospoint.cpp
    ${ROOT}/WayWise/core/simplewatchdog.cpp
    ${ROOT}/WayWise/core/vbytearray.cpp
    ${ROOT}/WayWise/core/geometry.h
    ${ROOT}/WayWise/external/pi-bno055/i2c_bno055.c
    ${ROOT}/WayWise/external/pi-as5600/driver_as5600.c
    ${ROOT}/WayWise/external/pi-as5600/iic.c
    ${ROOT}/WayWise/external/pi-as5600/driver_as5600_basic.c
    ${ROOT}/WayWise/external/pi-as5600/raspberrypi4b_driver_as5600_interface.c
    ${ROOT}/WayWise/external/vesc/vescpacket.cpp
    ${ROOT}/WayWise/routeplanning/routeutils.cpp
    ${ROOT}/WayWise/routeplanning/routeutils.h
    ${ROOT}/WayWise/sensors/camera/depthaicamera.cpp
    ${ROOT}/WayWise/sensors/camera/gimbal.h
    ${ROOT}/WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp
    ${ROOT}/WayWise/sensors/gnss/rtcm3_simple.cpp
    ${ROOT}/WayWise/sensors/gnss/rtcmclient.cpp
    ${ROOT}/WayWise/sensors/gnss/ublox.cpp
    ${ROOT}/WayWise/sensors/gnss/ubloxrover.cpp
    ${ROOT}/WayWise/sensors/gnss/gnssreceiver.cpp
    ${ROOT}/WayWise/sensors/imu/bno055orientationupdater.cpp
    ${ROOT}/WayWise/sensors/imu/imuorientationupdater.cpp
    ${ROOT}/WayWise/sensors/angle/as5600updater.cpp
    ${ROOT}/WayWise/sensors/angle/anglesensorupdater.cpp
    ${ROOT}/WayWise/vehicles/carstate.cpp
    ${ROOT}/WayWise/vehicles/controller/carmovementcontroller.cpp
    ${ROOT}/WayWise/vehicles/controller/motorcontroller.h
    ${ROOT}/WayWise/vehicles/controller/movementcontroller.cpp
    ${ROOT}/WayWise/vehicles/controller/servocontroller.cpp
    ${ROOT}/WayWise/vehicles/controller/vescmotorcontroller.cpp
    ${ROOT}/WayWise/vehicles/objectstate.cpp
    ${ROOT}/WayWise/vehicles/vehiclestate.cpp
    ${ROOT}/WayWise/vehicles/truckstate.cpp
    ${ROOT}/WayWise/vehicles/trailerstate.cpp
    ${ROOT}/WayWise/logger/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_include_directories(PreciseTruck PRIVATE ${ROOT} ${ROOT}/WayWise/)

target_link_libraries(PreciseTruck
    PRIVATE Qt5::Network
    PRIVATE Qt5::SerialPort
    PRIVATE MAVSDK::mavsdk
    PRIVATE gpiodcxx
)

add_definitions(-DPROJECT_ROOT="${CMAKE_SOURCE_DIR}")
