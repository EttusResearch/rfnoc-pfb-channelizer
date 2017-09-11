# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND pfb_channelizer_sources
    chanmux_impl.cc
    chanmux_block_ctrl_impl.cpp
    poly_channelizer_impl.cc
)


set(pfb_channelizer_sources "${pfb_channelizer_sources}" PARENT_SCOPE)
if(NOT pfb_channelizer_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT pfb_channelizer_sources)

add_library(gnuradio-pfb_channelizer SHARED ${pfb_channelizer_sources})
target_link_libraries(gnuradio-pfb_channelizer ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} ${ETTUS_LIBRARIES})
set_target_properties(gnuradio-pfb_channelizer PROPERTIES DEFINE_SYMBOL "gnuradio_pfb_channelizer_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-pfb_channelizer PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-pfb_channelizer
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_pfb_channelizer_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_pfb_channelizer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_pfb_channelizer.cc
)

add_executable(test-pfb_channelizer ${test_pfb_channelizer_sources})

target_link_libraries(
  test-pfb_channelizer
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  ${ETTUS_LIBRARIES}
  ${PC_ETTUS_LIBDIR}
  gnuradio-pfb_channelizer
)



GR_ADD_TEST(test_pfb_channelizer test-pfb_channelizer)
