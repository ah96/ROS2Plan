cmake_minimum_required(VERSION 3.8)
project(ros2plan_dependencies)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ros2plan_knowledge_msgs REQUIRED)

option(ROS2PLAN_BUILD_RDDL  "Build the RDDL parser"                               ON)
option(ROS2PLAN_BUILD_PPDDL "Build the PPDDL parser (probabilistic PDDL support)"  ON)
option(ROS2PLAN_COMPILE_CHIMP "Build the CHIMP HTN planner" OFF)

# Get the git submodules
message(STATUS "[${PROJECT_NAME}] Checking out the submodules...")
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule init OUTPUT_QUIET ERROR_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} submodule update --force OUTPUT_QUIET ERROR_QUIET WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# sys/vtimes.h was removed from glibc 2.34+ (Ubuntu 22.04); strip the dead include
execute_process(COMMAND sed -i "/#include \"sys\\/vtimes.h\"/d"
    ${CMAKE_CURRENT_SOURCE_DIR}/rddl_parser/utils/system_utils.cc)

########################################################################################################################
## HDDL PARSER
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing HDDL parser dependency...")

find_package(BISON 3.0.4 REQUIRED)
find_package(FLEX REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser)

BISON_TARGET(HDDL_PARSER hddl_parser/src/hddl_parser.yacc
             ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/hddl_parser.tab.cpp)
FLEX_TARGET(HDDL_SCANNER hddl_parser/src/hddl_scanner.lex
             ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/hddl_scanner.yacc.cpp)
ADD_FLEX_BISON_DEPENDENCY(HDDL_SCANNER HDDL_PARSER)

set(HDDL_PARSER_SOURCES hddl_parser/src/hddl_parser.cpp)
include_directories(hddl_parser/include ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser)

add_library(hddl_parser SHARED ${BISON_HDDL_PARSER_OUTPUTS} ${FLEX_HDDL_SCANNER_OUTPUTS} ${HDDL_PARSER_SOURCES})
set_target_properties(hddl_parser PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_compile_options(hddl_parser PRIVATE -O1 -Wno-register)

install(TARGETS hddl_parser
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY hddl_parser/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

# Install bison/flex generated headers that hddl_parser.h depends on
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/hddl_parser.tab.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/location.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/position.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hddl_parser/stack.hh
    DESTINATION include)

########################################################################################################################
## RDDL PARSER  (optional — GCC 11 has ICEs on this code; enable with -DROS2PLAN_BUILD_RDDL=ON)
########################################################################################################################
if(ROS2PLAN_BUILD_RDDL)
  message(STATUS "[${PROJECT_NAME}] Preparing RDDL parser dependency...")

  BISON_TARGET(RDDL_PARSER rddl_parser/parser.ypp ${CMAKE_CURRENT_BINARY_DIR}/parser.tab.cc COMPILE_FLAGS -d)
  FLEX_TARGET(RDDL_SCANNER rddl_parser/lexer.l ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc)
  ADD_FLEX_BISON_DEPENDENCY(RDDL_SCANNER RDDL_PARSER)

  set(RDDL_PARSER_SOURCES
          rddl_parser/rddl.cc
          rddl_parser/instantiator.cc
          rddl_parser/preprocessor.cc
          rddl_parser/task_analyzer.cc
          rddl_parser/evaluatables.cc
          rddl_parser/logical_expressions.cc
          rddl_parser/states.cc
          rddl_parser/probability_distribution.cc
          rddl_parser/utils/timer.cc
          rddl_parser/utils/string_utils.cc
          rddl_parser/utils/system_utils.cc
          rddl_parser/utils/math_utils.cc
          rddl_parser/RDDLParser.cpp)
  include_directories("rddl_parser")

  add_library(rddl_parser SHARED ${BISON_RDDL_PARSER_OUTPUTS} ${FLEX_RDDL_SCANNER_OUTPUTS} ${RDDL_PARSER_SOURCES})
  set_target_properties(rddl_parser PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_compile_options(rddl_parser PRIVATE -O0 -Wno-sign-compare -Wno-deprecated -Wno-deprecated-copy)

  install(TARGETS rddl_parser
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY rddl_parser/
          DESTINATION include
          FILES_MATCHING PATTERN "*.h"
          PATTERN ".svn" EXCLUDE)
else()
  message(STATUS "[${PROJECT_NAME}] Skipping RDDL parser (enable with -DROS2PLAN_BUILD_RDDL=ON)")
endif()

########################################################################################################################
## VAL
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Preparing VAL dependency...")

add_definitions(-Wno-deprecated-declarations)

set(VAL_SOURCES
        VAL/src/DebugWriteController.cpp
        VAL/src/pddl+.cpp
        VAL/src/ptree.cpp)

include_directories("VAL/include")

add_library(val_kcl SHARED ${VAL_SOURCES})
set_target_properties(val_kcl PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_compile_options(val_kcl PRIVATE -O1 -Wno-deprecated-declarations -Wno-register)

install(TARGETS val_kcl
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY VAL/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

########################################################################################################################
## rddlsim
########################################################################################################################
if(ROS2PLAN_BUILD_RDDL)
  message(STATUS "[${PROJECT_NAME}] Preparing rddlsim dependency...")
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/bin)
      message(STATUS "[${PROJECT_NAME}] Compiling rddlsim...")
      execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/compile WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rddlsim/ OUTPUT_QUIET ERROR_QUIET)
  else()
      message(STATUS "[${PROJECT_NAME}] rddlsim is already compiled!")
  endif()
endif()

########################################################################################################################
## PPDDL PARSER  (optional — enable with -DROS2PLAN_BUILD_PPDDL=ON)
########################################################################################################################
if(ROS2PLAN_BUILD_PPDDL)
  message(STATUS "[${PROJECT_NAME}] Preparing PPDDL parser dependency...")

  set(PPDDL_DIR ${CMAKE_CURRENT_LIST_DIR}/ppddl_parser/src/PPDDLParser)

  BISON_TARGET(PPDDL_PARSER ${PPDDL_DIR}/parser.yy ${CMAKE_CURRENT_BINARY_DIR}/ppddl_parser.tab.cc COMPILE_FLAGS -d)
  FLEX_TARGET(PPDDL_SCANNER ${PPDDL_DIR}/tokenizer.ll ${CMAKE_CURRENT_BINARY_DIR}/ppddl_lex.yy.cc)
  ADD_FLEX_BISON_DEPENDENCY(PPDDL_SCANNER PPDDL_PARSER)

  set(FLEX_BISON_OUTPUTS ${BISON_PPDDL_PARSER_OUTPUTS} ${FLEX_PPDDL_SCANNER_OUTPUTS} CACHE INTERNAL "" FORCE)

  set(PPDDL_PARSER_SOURCES
          ${PPDDL_DIR}/actions.cc
          ${PPDDL_DIR}/domains.cc
          ${PPDDL_DIR}/effects.cc
          ${PPDDL_DIR}/expressions.cc
          ${PPDDL_DIR}/formulas.cc
          ${PPDDL_DIR}/functions.cc
          ${PPDDL_DIR}/predicates.cc
          ${PPDDL_DIR}/problems.cc
          ${PPDDL_DIR}/rational.cc
          ${PPDDL_DIR}/requirements.cc
          ${PPDDL_DIR}/terms.cc
          ${PPDDL_DIR}/types.cc)

  set(PPDDL_PARSER_HEADERS
          ${PPDDL_DIR}/actions.h
          ${PPDDL_DIR}/domains.h
          ${PPDDL_DIR}/effects.h
          ${PPDDL_DIR}/expressions.h
          ${PPDDL_DIR}/formulas.h
          ${PPDDL_DIR}/functions.h
          ${PPDDL_DIR}/predicates.h
          ${PPDDL_DIR}/problems.h
          ${PPDDL_DIR}/rational.h
          ${PPDDL_DIR}/refcount.h
          ${PPDDL_DIR}/requirements.h
          ${PPDDL_DIR}/terms.h
          ${PPDDL_DIR}/types.h)

  set(PPDDL_PARSER_ALL ${PPDDL_PARSER_SOURCES} ${PPDDL_PARSER_HEADERS} ${FLEX_BISON_OUTPUTS})

  include_directories(ppddl_parser/include ppddl_parser/src ${PPDDL_DIR} VAL/include)

  add_library(ppddl_parser SHARED
          ppddl_parser/include/PPDDLDeterminizator.h ppddl_parser/src/PPDDLDeterminizator.cpp
          ppddl_parser/src/PPDDLParserInterface.cpp ppddl_parser/include/PPDDLParserInterface.h
          ppddl_parser/src/PDDLPrinter.cpp ppddl_parser/include/PDDLPrinter.h
          ppddl_parser/src/VALConversion.cpp ppddl_parser/include/VALConversion.h
          ${PPDDL_PARSER_ALL}
          ppddl_parser/src/Strategies/MLODeterminizator.cpp ppddl_parser/include/Strategies/MLODeterminizator.h
          ppddl_parser/src/Strategies/AODeterminizator.cpp ppddl_parser/include/Strategies/AODeterminizator.h
          ppddl_parser/src/Strategies/TLDeterminizator.cpp ppddl_parser/include/Strategies/TLDeterminizator.h
          ppddl_parser/src/PPDDLDeterminizatorFactory.cpp ppddl_parser/include/PPDDLDeterminizatorFactory.h)
  target_link_libraries(ppddl_parser val_kcl)
  set_target_properties(ppddl_parser PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_compile_options(ppddl_parser PRIVATE -O0 -Wno-deprecated -Wno-deprecated-copy)

  install(TARGETS ppddl_parser
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY ppddl_parser/include/ ppddl_parser/src/
          DESTINATION include
          FILES_MATCHING PATTERN "*.h"
          PATTERN ".svn" EXCLUDE)
else()
  message(STATUS "[${PROJECT_NAME}] Skipping PPDDL parser (enable with -DROS2PLAN_BUILD_PPDDL=ON)")
endif()

########################################################################################################################
## ippc_server
########################################################################################################################
message(STATUS "[${PROJECT_NAME}] Adding ippc_server dependency...")

include_directories(ippc_server/include ippc_server/cpp-base64)

add_library(ippc_server SHARED ippc_server/src/ippc_server.cpp ippc_server/src/strxml.cpp ippc_server/cpp-base64/base64.cpp)
ament_target_dependencies(ippc_server ros2plan_knowledge_msgs)

install(TARGETS ippc_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ippc_server/include/ ippc_server/cpp-base64/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

########################################################################################################################
## CHIMP
########################################################################################################################

if(ROS2PLAN_COMPILE_CHIMP)
    message(STATUS "[${PROJECT_NAME}] Compiling CHIMP...")
    execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/chimp/gradlew build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/chimp/)
endif()

########################################################################################################################

set(EXPORT_LIBS val_kcl hddl_parser ippc_server)
if(ROS2PLAN_BUILD_RDDL)
  list(APPEND EXPORT_LIBS rddl_parser)
endif()
if(ROS2PLAN_BUILD_PPDDL)
  list(APPEND EXPORT_LIBS ppddl_parser)
endif()

ament_export_include_directories(include)
ament_export_libraries(${EXPORT_LIBS})
ament_package()
