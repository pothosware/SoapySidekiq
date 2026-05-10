# FindSidekiq.cmake
#
# Locate the Sidekiq SDK from Epiq Solutions and define an imported target
# `Sidekiq::Sidekiq` that carries the include directory, the architecture-
# specific static archive, and all transitive link dependencies (tirpc,
# libusb-1.0, glib-2.0, threads, dl, rt, m).
#
# Typical usage:
#
#   find_package(Sidekiq REQUIRED)
#   target_link_libraries(my_target PRIVATE Sidekiq::Sidekiq)
#
# Inputs (cache variable, env var, or -D on the command line):
#
#   SIDEKIQ_SDK_ROOT  Root of the unpacked Sidekiq SDK
#                     (the directory containing sidekiq_core/ and lib/).
#                     Defaults to $ENV{SIDEKIQ_SDK_ROOT}, then $ENV{Sidekiq_DIR}.
#
#   SIDEKIQ_VARIANT   Library variant suffix, e.g. "x86_64.gcc", "aarch64",
#                     "z3u". Autodetected from CMAKE_SYSTEM_PROCESSOR for
#                     x86_64 and aarch64 hosts; required otherwise.
#
# Outputs (legacy variables, kept for backwards compatibility):
#
#   Sidekiq_FOUND
#   Sidekiq_INCLUDE_DIRS
#   Sidekiq_LIBRARIES   (just the main archive; transitive deps are NOT
#                        included here — use the imported target instead)

# --- locate the SDK root ------------------------------------------------------

if(NOT SIDEKIQ_SDK_ROOT)
    if(DEFINED ENV{SIDEKIQ_SDK_ROOT})
        set(SIDEKIQ_SDK_ROOT "$ENV{SIDEKIQ_SDK_ROOT}")
    elseif(DEFINED ENV{Sidekiq_DIR})
        set(SIDEKIQ_SDK_ROOT "$ENV{Sidekiq_DIR}")
    endif()
endif()

# --- pick the library variant for this host -----------------------------------

if(NOT SIDEKIQ_VARIANT)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
        set(SIDEKIQ_VARIANT "x86_64.gcc")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        set(SIDEKIQ_VARIANT "aarch64")
    endif()
endif()

# --- find the Sidekiq archive and headers -------------------------------------

find_path(Sidekiq_INCLUDE_DIR
    NAMES sidekiq_api.h
    HINTS
        "${SIDEKIQ_SDK_ROOT}/sidekiq_core/inc"
    PATHS
        /usr/local/include
        /usr/include
        /opt/include
        /opt/local/include)

if(SIDEKIQ_VARIANT)
    find_library(Sidekiq_LIBRARY
        NAMES "libsidekiq__${SIDEKIQ_VARIANT}.a" "sidekiq__${SIDEKIQ_VARIANT}"
        HINTS
            "${SIDEKIQ_SDK_ROOT}/lib"
        PATHS
            /usr/local/lib
            /usr/lib
            /opt/lib
            /opt/local/lib)
endif()

# --- transitive dependencies --------------------------------------------------
#
# The Sidekiq archive is non-PIC, statically built, and pulls in symbols from:
#   - libtirpc    (Sun RPC; removed from glibc 2.32+, so almost always external)
#   - libusb-1.0
#   - glib-2.0
#   - pthread, dl, rt, m
#
# Resolution preference for each shared dep:
#   1. pkg-config (clean, gives correct compile/link flags)
#   2. system find_library, including /usr/lib/epiq/ which the Epiq installer
#      populates with unversioned .so symlinks
#   3. SDK-bundled static under lib/support/<variant>/usr/lib/epiq/
#      (mainly for embedded/cross builds)

find_package(PkgConfig QUIET)
find_package(Threads REQUIRED)

set(_sidekiq_bundled_dir
    "${SIDEKIQ_SDK_ROOT}/lib/support/${SIDEKIQ_VARIANT}/usr/lib/epiq")

# Helper: resolve one shared dep, populating ${out_var} with either a target
# (PkgConfig::xxx) or an absolute path.
function(_sidekiq_find_dep out_var pc_name lib_name bundled_name)
    set(_result "")
    if(PkgConfig_FOUND)
        pkg_check_modules(${out_var}_PC QUIET IMPORTED_TARGET ${pc_name})
        if(${out_var}_PC_FOUND)
            set(_result "PkgConfig::${out_var}_PC")
        endif()
    endif()
    if(NOT _result)
        find_library(${out_var}_PATH
            NAMES ${lib_name} "${lib_name}.0"
            HINTS /usr/lib/epiq "${_sidekiq_bundled_dir}/.."
            PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
        if(${out_var}_PATH)
            set(_result "${${out_var}_PATH}")
        endif()
    endif()
    if(NOT _result AND EXISTS "${_sidekiq_bundled_dir}/${bundled_name}")
        message(STATUS
            "FindSidekiq: using bundled ${bundled_name} from SDK; "
            "install the system -dev package for a smaller, share-able build.")
        set(_result "${_sidekiq_bundled_dir}/${bundled_name}")
    endif()
    set(${out_var} "${_result}" PARENT_SCOPE)
endfunction()

_sidekiq_find_dep(Sidekiq_TIRPC libtirpc    libtirpc.so libtirpc.a)
_sidekiq_find_dep(Sidekiq_USB   libusb-1.0  libusb-1.0.so libusb-1.0.a)
_sidekiq_find_dep(Sidekiq_GLIB  glib-2.0    libglib-2.0.so libglib-2.0.a)

# --- finalize -----------------------------------------------------------------

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sidekiq
    REQUIRED_VARS
        Sidekiq_LIBRARY
        Sidekiq_INCLUDE_DIR
        Sidekiq_TIRPC
        Sidekiq_USB
        Sidekiq_GLIB)

if(Sidekiq_FOUND)
    set(Sidekiq_LIBRARIES "${Sidekiq_LIBRARY}")
    set(Sidekiq_INCLUDE_DIRS "${Sidekiq_INCLUDE_DIR}")

    if(NOT TARGET Sidekiq::Sidekiq)
        add_library(Sidekiq::Sidekiq STATIC IMPORTED)
        set_target_properties(Sidekiq::Sidekiq PROPERTIES
            IMPORTED_LOCATION "${Sidekiq_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${Sidekiq_INCLUDE_DIR}")
        set_property(TARGET Sidekiq::Sidekiq PROPERTY
            INTERFACE_LINK_LIBRARIES
                "${Sidekiq_TIRPC}"
                "${Sidekiq_USB}"
                "${Sidekiq_GLIB}"
                Threads::Threads
                ${CMAKE_DL_LIBS}
                rt
                m)
    endif()
endif()

mark_as_advanced(
    Sidekiq_INCLUDE_DIR
    Sidekiq_LIBRARY
    Sidekiq_TIRPC_PATH
    Sidekiq_USB_PATH
    Sidekiq_GLIB_PATH)
