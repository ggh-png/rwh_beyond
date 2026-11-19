# Package the registered poses without making a local calibration mandatory for clean builds.
if(EXISTS "${SOURCE}")
  configure_file("${SOURCE}" "${SNAPSHOT}" COPYONLY)
else()
  file(WRITE "${SNAPSHOT}.empty" [=[version: 1
units: radians
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
poses: {}
]=])
  configure_file("${SNAPSHOT}.empty" "${SNAPSHOT}" COPYONLY)
endif()
