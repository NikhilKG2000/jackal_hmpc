cmake_minimum_required(VERSION 3.16)
project(mpc_solver LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mpc_base REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
  message(FATAL_ERROR
    "ACADOS_SOURCE_DIR is not set.")
endif()

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
set(ACADOS_INCLUDE_DIR "${ACADOS_SOURCE_DIR}/include")
set(ACADOS_LIBRARY_DIR "${ACADOS_SOURCE_DIR}/lib")

set(TMPC_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/jackal_tmpc/c_generated_code")

set(PMPC_GENERATED_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/jackal_pmpc/c_generated_code")

# Check that generated metadata matches the Python settings.
execute_process(
  COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/hmpc_settings.py"
  RESULT_VARIABLE SETTINGS_CHECK_RESULT
  OUTPUT_VARIABLE SETTINGS_CHECK_OUTPUT
  ERROR_VARIABLE SETTINGS_CHECK_ERROR
)

if(NOT SETTINGS_CHECK_RESULT EQUAL 0)
  message(FATAL_ERROR
    "${SETTINGS_CHECK_OUTPUT}${SETTINGS_CHECK_ERROR}")
endif()

find_library(ACADOS_LIBRARY acados
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)

find_library(BLASFEO_LIBRARY blasfeo
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)

find_library(HPIPM_LIBRARY hpipm
  PATHS "${ACADOS_LIBRARY_DIR}" NO_DEFAULT_PATH)

if(NOT ACADOS_LIBRARY OR NOT BLASFEO_LIBRARY OR NOT HPIPM_LIBRARY)
  message(FATAL_ERROR
    "Could not find acados, blasfeo and hpipm in ${ACADOS_LIBRARY_DIR}")
endif()

function(add_jackal_generated_solver layer)
  set(model_name "jackal_${layer}")

  set(generated_dir
    "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/${model_name}/c_generated_code")

  if(NOT EXISTS "${generated_dir}/acados_solver_${model_name}.h")
    message(FATAL_ERROR
      "Missing generated ${model_name} solver. Run its Python generator first.")
  endif()

  # Restrict collection to generated model, cost and constraint sources.
  # Standalone main/simulation programs are not included.
  file(GLOB generated_functions CONFIGURE_DEPENDS
    "${generated_dir}/${model_name}_model/*.c"
    "${generated_dir}/${model_name}_cost/*.c"
    "${generated_dir}/${model_name}_constraints/*.c"
  )

  add_library(${model_name}_acados_generated SHARED
    "${generated_dir}/acados_solver_${model_name}.c"
    ${generated_functions}
  )

  target_compile_features(
    ${model_name}_acados_generated PRIVATE c_std_99)

  target_include_directories(
    ${model_name}_acados_generated PRIVATE
    "${generated_dir}"
    "${ACADOS_INCLUDE_DIR}"
    "${ACADOS_INCLUDE_DIR}/acados"
    "${ACADOS_INCLUDE_DIR}/blasfeo/include"
    "${ACADOS_INCLUDE_DIR}/hpipm/include"
  )

  target_link_libraries(
    ${model_name}_acados_generated PUBLIC
    "${ACADOS_LIBRARY}"
    "${BLASFEO_LIBRARY}"
    "${HPIPM_LIBRARY}"
    m
  )

  set_target_properties(
    ${model_name}_acados_generated PROPERTIES
    BUILD_RPATH "${ACADOS_LIBRARY_DIR}"
    INSTALL_RPATH "${ACADOS_LIBRARY_DIR};$ORIGIN"
  )
endfunction()

add_jackal_generated_solver(tmpc)
add_jackal_generated_solver(pmpc)

add_library(${PROJECT_NAME} SHARED
  src/acados/acados_solver.cpp
  src/core/helper_functions.cpp
  src/jackal/jackal_solver_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wno-deprecated-declarations
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${TMPC_GENERATED_DIR}"
    "${PMPC_GENERATED_DIR}"
    "${ACADOS_INCLUDE_DIR}"
    "${ACADOS_INCLUDE_DIR}/acados"
    "${ACADOS_INCLUDE_DIR}/blasfeo/include"
    "${ACADOS_INCLUDE_DIR}/hpipm/include"
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  jackal_tmpc_acados_generated
  jackal_pmpc_acados_generated
)

ament_target_dependencies(${PROJECT_NAME}
  mpc_base
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ACADOS_LIBRARY_DIR}"
  INSTALL_RPATH "${ACADOS_LIBRARY_DIR};$ORIGIN"
)

install(
  TARGETS
    jackal_tmpc_acados_generated
    jackal_pmpc_acados_generated
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 mpc_base)

ament_package()