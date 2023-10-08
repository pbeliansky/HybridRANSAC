cmake_minimum_required (VERSION 3.13)

SET(CMAKE_CXX_FLAGS "-std=gnu++17")

add_definitions (-march=native)

file(GLOB_RECURSE CFILES "*.c")
SET_SOURCE_FILES_PROPERTIES(${CFILES} PROPERTIES LANGUAGE CXX )

include_directories (
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/RansacLib
  ${CMAKE_SOURCE_DIR}/ransac_solvers
  ${CMAKE_SOURCE_DIR}/solvers
  ${CMAKE_SOURCE_DIR}/solvers/common
  ${CMAKE_SOURCE_DIR}/eigen
)

add_executable (eval_localization_calibrated eval_localization_calibrated.cpp "eval_utils.h" "${CMAKE_SOURCE_DIR}/ransac_solvers/hybrid_H50_ransac_solver.h" "${CMAKE_SOURCE_DIR}/ransac_solvers/hybrid_H50_ransac_solver.cpp" "data_handling.h")

if(MSVC)	
  target_compile_options(eval_localization_calibrated PRIVATE /bigobj /fp:fast)# /Ox)
else()
  target_compile_options(eval_localization_calibrated PRIVATE -march=native -ffast-math -O3)
endif()
