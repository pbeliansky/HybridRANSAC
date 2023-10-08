cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

SET(CMAKE_CXX_FLAGS "-std=gnu++17")

add_definitions (-march=native)

file(GLOB_RECURSE CFILES "*.c")
SET_SOURCE_FILES_PROPERTIES(${CFILES} PROPERTIES LANGUAGE CXX )

include_directories (
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/solvers
	${CMAKE_SOURCE_DIR}/solvers/common  
	${CMAKE_SOURCE_DIR}/eigen
)

add_executable (tests tests.cpp test_sh45_2.h util.h)


if(MSVC)	
  target_compile_options(tests PRIVATE /bigobj /fp:fast)
else()
  target_compile_options(tests PRIVATE -march=native -ffast-math -O3)
endif()