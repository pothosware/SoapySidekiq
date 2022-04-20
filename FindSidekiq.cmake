# - Try to find Sidekiq
# Once done this will define
#  Sidekiq_FOUND - System has Sidekiq
#  Sidekiq_LIBRARIES - The Sidekiq libraries
#  Sidekiq_INCLUDE_DIRS - The Sidekiq include directories
#  Sidekiq_LIB_DIRS - The Sidekiq library directories

if(NOT Sidekiq_FOUND)

    find_path(Sidekiq_INCLUDE_DIR
            NAMES sidekiq_api.h
            HINTS ${Sidekiq_PKG_INCLUDE_DIRS} $ENV{Sidekiq_DIR}/include
            PATHS ~/sidekiq_sdk_current/sidekiq_core/inc/ /usr/local/include /usr/include /opt/include /opt/local/include)

    execute_process (
        COMMAND uname -m
        OUTPUT_VARIABLE outVar
    )    

    message(STATUS "type ${outVar}")

    if(${outVar} MATCHES "x86_64")
        set (libname  "libsidekiq__x86_64.gcc.a")
        set (epiq_name "~/sidekiq_sdk_current/lib/support/x86_64.gcc/usr/lib/epiq")
    else()
        set(libname  "libsidekiq__aarch64.gcc6.3.a")
        set (epiq_name "~/sidekiq_sdk_current/lib/support/aarch64.gcc6.3/usr/lib/epiq")
    endif()

    message(STATUS "library is ${libname} Epiq lib is ${epiq_name}")

    find_library(Sidekiq_LIBRARY
        NAMES ${libname}
        HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
        PATHS ~/sidekiq_sdk_current/lib/ /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

    find_library(Sidekiq_EPIQ_USB
        NAMES libusb-1.0.so
        HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
        PATHS ${epiq_name} /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

    find_library(Sidekiq_EPIQ_GLIB
        NAMES libglib-2.0.so
        HINTS ${Sidekiq_PKG_LIBRARY_DIRS} $ENV{Sidekiq_DIR}/include
        PATHS ${epiq_name} /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

    set(Sidekiq_LIBRARIES ${Sidekiq_LIBRARY})
    set(Sidekiq_INCLUDE_DIRS ${Sidekiq_INCLUDE_DIR})
    set(Sidekiq_EPIQ_USB ${Sidekiq_EPIQ_USB})
    set(Sidekiq_EPIQ_GLIB ${Sidekiq_EPIQ_GLIB})

    include(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set LibSidekiq_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(Sidekiq  DEFAULT_MSG
        Sidekiq_LIBRARY Sidekiq_INCLUDE_DIR Sidekiq_EPIQ_USB Sidekiq_EPIQ_GLIB)

    mark_as_advanced(Sidekiq_INCLUDE_DIR Sidekiq_LIBRARY Sidekiq_EPIQ_USB Sidekiq_EPIQ_GLIB)

endif(NOT Sidekiq_FOUND)
