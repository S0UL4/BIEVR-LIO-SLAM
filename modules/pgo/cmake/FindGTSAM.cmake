# Resolves GTSAM for bievr_pgo, with a vendor-and-build fallback.
#
# GTSAM has no apt package and no install script in this repo (see the top-level
# README's "Loop closure" section), so a fresh checkout on a machine that has
# never built this workspace before would otherwise fail at configure time with
# "Could not find a package configuration file provided by GTSAM".
#
# Resolution order:
#   1. Whatever find_package(GTSAM CONFIG) already finds (a prior manual
#      install, one baked into a Docker image via CMAKE_PREFIX_PATH, etc).
#   2. A previously vendored build of GTSAM under BIEVR_GTSAM_VENDOR_PREFIX.
#   3. Clone borglab/gtsam @ 4.2.0, build it with GTSAM_USE_SYSTEM_EIGEN=ON
#      (required -- GTSAM's bundled Eigen does not match the one the rest of
#      the estimator links against) and install it to BIEVR_GTSAM_VENDOR_PREFIX,
#      then find_package() it again so it resolves to a real CONFIG-mode
#      imported target (needed for bievr_pgo's own install(EXPORT ...)).
#
# The vendor prefix defaults to a per-user cache directory so this works
# without sudo; override -DBIEVR_GTSAM_VENDOR_PREFIX=... (e.g. to /usr/local
# inside a Docker image already running as root) to share it system-wide.

if(NOT DEFINED BIEVR_GTSAM_VENDOR_PREFIX)
  set(BIEVR_GTSAM_VENDOR_PREFIX "$ENV{HOME}/.cache/bievr-thirdparty/gtsam-4.2.0"
      CACHE PATH "Install prefix used to build GTSAM from source when it isn't found on the system")
endif()
list(APPEND CMAKE_PREFIX_PATH "${BIEVR_GTSAM_VENDOR_PREFIX}")

find_package(GTSAM CONFIG QUIET)

if(NOT GTSAM_FOUND)
  message(STATUS "GTSAM not found -- vendoring gtsam 4.2.0 into ${BIEVR_GTSAM_VENDOR_PREFIX} "
                  "(first time only; this builds from source and can take several minutes)")

  find_package(Git REQUIRED)

  set(_bievr_gtsam_src "${CMAKE_BINARY_DIR}/_deps/gtsam-src")
  if(NOT EXISTS "${_bievr_gtsam_src}/.git")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" clone --branch 4.2.0 --depth 1
              https://github.com/borglab/gtsam.git "${_bievr_gtsam_src}"
      RESULT_VARIABLE _bievr_gtsam_rc
    )
    if(NOT _bievr_gtsam_rc EQUAL 0)
      message(FATAL_ERROR "Failed to clone gtsam 4.2.0")
    endif()
  endif()

  set(_bievr_gtsam_build "${CMAKE_BINARY_DIR}/_deps/gtsam-build")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${_bievr_gtsam_src}" -B "${_bievr_gtsam_build}"
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_INSTALL_PREFIX=${BIEVR_GTSAM_VENDOR_PREFIX}
            -DGTSAM_USE_SYSTEM_EIGEN=ON
            -DGTSAM_BUILD_TESTS=OFF
            -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
            -DGTSAM_BUILD_UNSTABLE=OFF
    RESULT_VARIABLE _bievr_gtsam_rc
  )
  if(NOT _bievr_gtsam_rc EQUAL 0)
    message(FATAL_ERROR "Failed to configure gtsam 4.2.0")
  endif()

  include(ProcessorCount)
  ProcessorCount(_bievr_gtsam_nproc)
  if(_bievr_gtsam_nproc EQUAL 0)
    set(_bievr_gtsam_nproc 4)
  endif()

  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${_bievr_gtsam_build}" --parallel ${_bievr_gtsam_nproc}
    RESULT_VARIABLE _bievr_gtsam_rc
  )
  if(NOT _bievr_gtsam_rc EQUAL 0)
    message(FATAL_ERROR "Failed to build gtsam 4.2.0")
  endif()

  execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${_bievr_gtsam_build}"
    RESULT_VARIABLE _bievr_gtsam_rc
  )
  if(NOT _bievr_gtsam_rc EQUAL 0)
    message(FATAL_ERROR "Failed to install gtsam 4.2.0 to ${BIEVR_GTSAM_VENDOR_PREFIX}")
  endif()

  find_package(GTSAM CONFIG REQUIRED)
  message(STATUS "Vendored gtsam 4.2.0 installed to ${BIEVR_GTSAM_VENDOR_PREFIX}")
endif()
