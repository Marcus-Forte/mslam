# Logging and timing utilities owned by mslam. The optimizer library (moptim)
# no longer ships any logging framework, so these live here and are linked
# wherever they are needed.
add_library(mlog
  ILog.cc
  ConsoleLogger.cc
  Timer.cc)

add_library(mslam::mlog ALIAS mlog)
target_include_directories(mlog PUBLIC ${CMAKE_SOURCE_DIR}/include)
