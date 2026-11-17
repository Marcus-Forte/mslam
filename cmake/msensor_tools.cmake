# msensor tools that mslam builds next to itself and optionally installs.
#
# | executable         | build target    | install option                |
# |--------------------|-----------------|-------------------------------|
# | playback_publisher | replay-server   | MSLAM_INSTALL_REPLAY_SERVER   |
# | remote_recorder    | remote-recorder | MSLAM_INSTALL_REMOTE_RECORDER |
#
# playback_publisher replays a .pbscan as a sensor server; remote_recorder
# (`remote_recorder <host:port> [out.pbscan]`) records a sensor server's
# streams into a .pbscan. Both are written to the top of the build tree, next
# to mslam. Installation is opt-in to keep them out of deployment images by
# default; the gcc preset turns both on. Installed tools share mslam's runtime
# dependency set (mslam_runtime_targets), which the top-level install rule
# copies to lib/.
#
# Use: include this file before add_subdirectory(third_party/msensor) (the
# options must exist then), and call mslam_add_msensor_tools() after it (the
# targets must exist then).

option(MSLAM_INSTALL_REPLAY_SERVER
  "Install playback_publisher with mslam (disabled by default)" OFF)
option(MSLAM_INSTALL_REMOTE_RECORDER
  "Install remote_recorder with mslam (disabled by default)" OFF)

# msensor's own install rule for playback_publisher follows the same choice.
set(MSENSOR_INSTALL_PLAYBACK_PUBLISHER ${MSLAM_INSTALL_REPLAY_SERVER})

# Put one msensor executable next to mslam, give it a build target and, when
# `install` is true, install it with mslam's runtime dependencies.
function(mslam_msensor_tool executable build_target install)
  if(NOT TARGET ${executable})
    message(WARNING "msensor does not define ${executable}; ${build_target} is not available")
    return()
  endif()

  set_target_properties(${executable} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  add_custom_target(${build_target} DEPENDS ${executable})

  if(install)
    # msensor is added EXCLUDE_FROM_ALL; an installed tool must be built by default.
    set_property(TARGET ${executable} PROPERTY EXCLUDE_FROM_ALL FALSE)
    install(
      TARGETS ${executable}
      DESTINATION ${CMAKE_INSTALL_BINDIR}
      RUNTIME_DEPENDENCY_SET mslam_runtime_targets
    )
  endif()
endfunction()

function(mslam_add_msensor_tools)
  mslam_msensor_tool(playback_publisher replay-server ${MSLAM_INSTALL_REPLAY_SERVER})
  mslam_msensor_tool(remote_recorder remote-recorder ${MSLAM_INSTALL_REMOTE_RECORDER})
endfunction()
