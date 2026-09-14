# ihrs::algo - robotics algorithms library
# (planning, control, localization, kinematics, perception, decision, viz)

add_subdirectory(src)

if(IHIRS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(IHIRS_BUILD_TESTS)
  add_subdirectory(tests)
endif()
