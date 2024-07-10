# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "APilot_Console_autogen"
  "CMakeFiles/APilot_Console_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/APilot_Console_autogen.dir/ParseCache.txt"
  )
endif()
