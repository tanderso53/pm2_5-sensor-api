cmake_minimum_required(VERSION 3.18)

project(pm2_5-sensor-driver)

##########################
# DOCUMENTATION OVERIDES #
##########################

if(NOT DEFINED PM2_5_BUILD_DOCS)

  set(PM2_5_BUILD_DOCS true)

endif()

set(DOXYGEN_OUTPUT_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/doc)

set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)

set(DOXYGEN_GENERATE_TREEVIEW YES)

set(DOXYGEN_FULL_SIDEBAR YES)

set(DOXYGEN_USE_MDFILE_AS_MAINPAGE
  ${PROJECT_SOURCE_DIR}/README.md)

set(DOXYGEN_HTML_EXTRA_STYLESHEET
  ${CMAKE_CURRENT_LIST_DIR}/doc/doxygen-awesome-css/doxygen-awesome.css)

set(DOXYGEN_PROJECT_LOGO
  ${CMAKE_CURRENT_LIST_DIR}/icon/pm2_5-icon-55x55.png)

set(DOXYGEN_EXCLUDE_PATTERNS
  */doxygen-awesome-css/*)

#########################################
# API library for PMS 5003 PM2.5 Sensor #
#########################################

add_library(pm2_5-sensor-api INTERFACE)

target_sources(pm2_5-sensor-api INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/pm2_5.c)

target_include_directories(pm2_5-sensor-api INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include)

#############################
# LIBRARY API DOCUMENTATION #
#############################

find_package(Doxygen
  REQUIRED dot
  OPTIONAL_COMPONENTS mscgen dia)

if(PM2_5_BUILD_DOCS AND DOXYGEN_FOUND)

  find_package(LATEX COMPONENTS PDFLATEX MAKEINDEX)

  if(NOT DEFINED PM2_5_BUILD_DOCS_PDF)

    set(PM2_5_BUILD_DOCS_PDF true)

  endif()

  if(LATEX_FOUND AND PM2_5_BUILD_DOCS_PDF)

    set(DOXYGEN_GENERATE_LATEX YES)

  else()

    message(NOTICE "PDF Documentation will not be build")

  endif()

  doxygen_add_docs(builddocs ALL
    ${PROJECT_SOURCE_DIR}
    COMMENT "Generate API documentation")

  # Install docs

  install(DIRECTORY ${DOXYGEN_OUTPUT_DIRECTORY}/html/
    DESTINATION ${CMAKE_INSTALL_DOCDIR}/html)

  # If LATEX available, build and install a PDF as well

  if(LATEX_FOUND AND PM2_5_BUILD_DOCS_PDF)

    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pdf
      COMMAND make -C ${DOXYGEN_OUTPUT_DIRECTORY}/latex
      LATEX_CMD=${PDFLATEX_COMPILER}
      COMMAND cp ${DOXYGEN_OUTPUT_DIRECTORY}/latex/refman.pdf
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pdf
      DEPENDS builddocs)

    add_custom_target(builddocs-pdf ALL
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pdf
      COMMENT "Building PDF Documentation")

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pdf
      TYPE DOC)

  endif()

else()

  message(NOTICE "Documentation will not be build")

endif()

#########
# Tests #
#########

if(DEFINED PM2_5_BUILD_TESTS)

  enable_testing()

  add_executable(pm2_5-sensor-test
    ${CMAKE_CURRENT_LIST_DIR}/tests/pm2_5-api-tests.c)

  target_link_libraries(pm2_5-sensor-test PRIVATE
    pm2_5-sensor-api)

  target_compile_options(pm2_5-sensor-test PRIVATE -Wall -g)

  target_compile_options(pm2_5-sensor-api INTERFACE -Wall -g)

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
