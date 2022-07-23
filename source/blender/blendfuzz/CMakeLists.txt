# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright 2006 Blender Foundation. All rights reserved.

#-----------------------------------------------------------------------------

include_directories(
  ../blenlib
  ../imbuf
  ../makesdna
  ../../../intern/guardedalloc
)

include_directories(
  SYSTEM
  ${ZLIB_INCLUDE_DIRS}
)

set(SRC
  src/blendfuzz.hh
  src/blendfuzz_imbuf.cc
)

set(SRC_CMD
src/blendfuzz.cc
)

add_executable(blender-fuzz ${SRC} ${SRC_CMD})
setup_platform_linker_flags(blender-fuzz)
target_link_libraries(blender-fuzz bf_blenlib)
target_link_libraries(blender-fuzz ${PTHREADS_LIBRARIES})
