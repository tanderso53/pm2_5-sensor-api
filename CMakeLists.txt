cmake_minimum_required(VERSION 3.18)

#########################################
# API library for PMS 5003 PM2.5 Sensor #
#########################################

add_library(pm2_5-sensor-api INTERFACE)

target_sources(pm2_5-sensor-api INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/pm2_5.c)

target_include_directories(pm2_5-sensor-api INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include)

#########
# Tests #
#########

if(DEFINED PM2_5_BUILD_TESTS)

  enable_testing()

  add_executable(pm2_5-sensor-test
    ${CMAKE_CURRENT_LIST_DIR}/tests/pm2_5-api-tests.c)

  target_link_libraries(pm2_5-sensor-test PRIVATE
    pm2_5-sensor-api)

  add_test(NAME pm2_5-sensor-test-suite
    COMMAND $<TARGET_FILE:pm2_5-sensor-test>)

  if(DEFINED PM2_5_BUILD_INST_CODE)

    # Enable coverage instrumentation if coverage tools are present
    find_program(CC_COV clang REQUIRED)
    find_program(PROFILER llvm-profdata REQUIRED)
    find_program(LCOV llvm-cov REQUIRED)
    find_program(JQ jq REQUIRED)

    set(CC ${CC_COV})

    set_source_files_properties(src/pm2_5.c #tests/pm2_5-api-tests.c
      PROPERTIES
      COMPILE_OPTIONS "-fprofile-instr-generate;-fcoverage-mapping")

    target_link_options(pm2_5-sensor-test PRIVATE
      -fprofile-instr-generate -fcoverage-mapping)

    add_compile_options(-fprofile-instr-generate -fcoverage-mapping)

    add_custom_target(pm2_5-sensor-coverage
      LLVM_PROFILE_FILE=coverage.profraw $<TARGET_FILE:pm2_5-sensor-test>
      COMMAND ${PROFILER} merge -sparse coverage.profraw -o coverage.profdata
      COMMAND ${LCOV} export $<TARGET_FILE:pm2_5-sensor-test>
      -instr-profile=coverage.profdata > coverage.json
      COMMAND cat coverage.json | jq ".data[0].totals.lines.percent" |
      sed -E "s/(.*)/Coverage: \\1/"
      BYPRODUCTS coverage.profraw coverage.profdata coverage.json
      VERBATIM)

    add_dependencies(pm2_5-sensor-coverage pm2_5-sensor-test)

  endif()

endif()
