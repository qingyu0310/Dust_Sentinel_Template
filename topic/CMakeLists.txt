if(CONFIG_TPC_TO_CAN_TX)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_can_tx)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/to_can_tx/to_can_tx.cpp)
endif()

if(CONFIG_TPC_REMOTE_TO)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote_to)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote_to/remote_to.cpp)
endif()

if(CONFIG_TPC_IMU_TO)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu_to)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu_to/imu_to.cpp)
endif()
