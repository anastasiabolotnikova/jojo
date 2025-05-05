set(controller_SRC
  TeleoperationDemoArmAssist.cpp
  ClosedLoopCollisionConstraint.cpp
)

set(controller_HDR
  TeleoperationDemoArmAssist.h
  ClosedLoopCollisionConstraint.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DTeleoperationDemoHRI_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_mori::devices)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
