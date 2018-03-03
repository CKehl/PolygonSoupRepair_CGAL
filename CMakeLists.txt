cmake_minimum_required(VERSION 2.8)
project(CGEL)

##cmakedefine USE_MYMATH

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DDEBUG")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_EXE_LINKER_FLAGS_MINSIZEREL}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} -DDEBUG")
#set(CMAKE_INSTALL_PREFIX "/usr/local/lib/CGEL")
set(CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER})
include(CMakeBackwardCompatibilityC)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

set(ADDITIONAL_HEADERS "")
set(ADDITIONAL_SOURCES "")
set(CUDA_SOURCES "")
set(KERNEL_O "")
set(SOURCE_FOLDER src)
set(ADDITIONAL_LIBRARY_PATHS "")
set(ADDITIONAL_LINKER_FLAGS "")
set(ADDITIONAL_CXX_FLAGS "-fpermissive -std=c++0x")
set(LIBRARIES "")
#osg osgDB ${CGAL_LIBRARIES} gmp core++ mpfr ${VTK_LIBRARIES} ${Boost_LIBRARIES}
set(INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FOLDER}" )

find_package(CGAL)
if(CGAL_FOUND)
  message(${CGAL_USE_FILE})
  include(${CGAL_USE_FILE})
  #set(ADDITIONAL_LIBRARY_PATHS "${ADDITIONAL_LIBRARY_PATHS} ${CGAL_LIBRARIES_DIR}")
  #set(LIBRARIES "${LIBRARIES} ${CGAL_LIBRARIES} gmp core++ mpfr")
  list(APPEND ADDITIONAL_LIBRARY_PATHS ${CGAL_LIBRARIES_DIR})
  list(APPEND CGAL_LIBRARIES CGAL_Core)
  list(APPEND CGAL_LIBRARIES CGAL_ImageIO)
  list(APPEND LIBRARIES ${CGAL_LIBRARIES})
  set(CORE_LIBRARIES "")
  list(APPEND CORE_LIBRARIES gmp)
  list(APPEND CORE_LIBRARIES core++)
  list(APPEND CORE_LIBRARIES mpfr)
  list(APPEND LIBRARIES ${CORE_LIBRARIES})
  list(APPEND INCLUDE_DIRS ${CGAL_INCLUDE_DIRS})
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCOMPILE_WITH_CGAL -frounding-math")
  #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -frounding-math")
  set (ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DCGAL_NO_WARNINGS -DCOMPILE_WITH_CGAL")
  set (ADDITIONAL_LINKER_FLAGS "${ADDITIONAL_LINKER_FLAGS} -frounding-math")
  #message(${CMAKE_CXX_FLAGS})
  add_definitions(${CGAL_3RD_PARTY_DEFINITIONS}  ${CGAL_DEFINITIONS} "-frounding-math" "-DCOMPILE_WITH_CGAL" "-DCGAL_NO_WARNINGS")
  link_libraries(${CGAL_LIBRARIES} ${CORE_LIBRARIES})
endif(CGAL_FOUND)

#eigen library
find_package(Eigen3)
if(EIGEN3_FOUND)
  list(APPEND 3RDPARTY_INCLUDE_PATHS ${EIGEN3_INCLUDE_DIR})
  set (ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} -DCOMPILE_WITH_EIGEN")
  add_definitions("-DCOMPILE_WITH_EIGEN")
  message(STATUS "Use Eigen3.")
  INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_FOUND)

find_package(Boost REQUIRED)
if(Boost_FOUND)
  #set(LIBRARIES "${LIBRARIES} ${Boost_LIBRARIES}")
  list(APPEND ADDITIONAL_LIBRARY_PATHS ${BOOST_LIBRARYDIR})
  list(APPEND LIBRARIES ${Boost_LIBRARIES})
  set(Boost_SUBSYSTEM "")
  list(APPEND Boost_SUBSYSTEM boost_system)
  list(APPEND Boost_SUBSYSTEM boost_filesystem)
  list(APPEND Boost_SUBSYSTEM boost_regex)
  list(APPEND Boost_SUBSYSTEM boost_thread)
  #set(INCLUDE_DIRS "${INCLUDE_DIRS} ${Boost_INCLUDE_DIR}")
  list(APPEND INCLUDE_DIRS ${Boost_INCLUDE_DIR})
  message(${CMAKE_CXX_FLAGS})
  link_directories(${BOOST_LIBRARYDIR})
  link_libraries(${Boost_LIBRARIES} ${Boost_SUBSYSTEM})
endif(Boost_FOUND)

#  LAPACK_FOUND - set to true if a library implementing the LAPACK interface
#    is found
#  LAPACK_LINKER_FLAGS - uncached list of required linker flags (excluding -l
#    and -L).
#  LAPACK_LIBRARIES - uncached list of libraries (using full path name) to
#    link against to use LAPACK
#  LAPACK95_LIBRARIES - uncached list of libraries (using full path name) to
#    link against to use LAPACK95
#  LAPACK95_FOUND - set to true if a library implementing the LAPACK f95
#    interface is found
#  BLA_STATIC  if set on this determines what kind of linkage we do (static)
#  BLA_VENDOR  if set checks only the specified vendor, if not set checks
#     all the possibilities
#  BLA_F95     if set on tries to find the f95 interfaces for BLAS/LAPACK
### List of vendors (BLA_VENDOR) valid in this module
##  Intel(mkl), ACML,Apple, NAS, Generic
#find_package(LAPACK REQUIRED)
#if(LAPACK_FOUND)
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LAPACK_LINKER_FLAGS}")
#  set(LIBRARIES "${LIBRARIES} ${LAPACK_LIBRARIES}")
#  message("Use LAPACK.")
#  message(${CMAKE_CXX_FLAGS})
#endif(LAPACK_FOUND)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  set (ADDITIONAL_CXX_FLAGS "${ADDITIONAL_CXX_FLAGS} ${OpenMP_EXE_LINKER_FLAGS} -fopenmp")
  set (ADDITIONAL_LINKER_FLAGS "${ADDITIONAL_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS} -fopenmp")
  message(${CMAKE_CXX_FLAGS})
endif(OPENMP_FOUND)

list(REMOVE_DUPLICATES ADDITIONAL_LIBRARY_PATHS)
list(REMOVE_DUPLICATES INCLUDE_DIRS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${ADDITIONAL_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ADDITIONAL_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${ADDITIONAL_CXX_FLAGS}")

#OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} CACHE STRING "(EXPERIMENTAL) You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX} CACHE STRING "Final Installation Path.")

set(M_LIBRARY)
if(UNIX)
	set(M_LIBRARY m)
endif(UNIX)
# gmp core++ mpfr

set(LIBS_WO_M "${LIBRARIES}")
LIST(APPEND LIBRARIES ${M_LIBRARY})

include_directories(${INCLUDE_DIRS} ${CMAKE_INCLUDE_PATH})
link_directories(${ComputerGraphicsElementsLibrary_SOURCE_DIR} ${ADDITIONAL_LIBRARY_PATHS} ${CMAKE_BINARY_DIR})
add_executable(PolygonalRepair "${SOURCE_FOLDER}/main.cpp")
install(TARGETS PolygonalRepair RUNTIME DESTINATION bin)
