if(CONFIG_MOD_DEV_MOTOR_DJI)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dji/dji_c6xx.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dji)
endif()

if(CONFIG_MOD_DEV_MOTOR_DM)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dm/dm.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motors/dm)
endif()

if(CONFIG_MOD_DEV_POWERMETER)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter/powermeter.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/powermeter)
endif()

if(CONFIG_MOD_DEV_REMOTE)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/remote.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/dr16/dr16.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt12/vt12.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt13/vt13.cpp
	)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/remotes
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/dr16
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt12
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/vt13
	)
endif()

if(CONFIG_MOD_DEV_IMU)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/imu.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu)
endif()

if(CONFIG_MOD_DEV_IMU_BMI088)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/bmi088.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/imu
		${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088
	)
endif()
