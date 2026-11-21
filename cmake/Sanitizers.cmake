# Sanitizer configuration. Flags are applied globally (before dependencies are added) because MSVC
# requires every object file in an AddressSanitizer link to be instrumented consistently.
#
#   GCC / Clang (Linux):  -fsanitize=address and/or undefined, no recovery.
#   MSVC cl:              /fsanitize=address. Debug's /RTC checks are incompatible; use RelWithDebInfo.
#   clang-cl:             UBSan in trap mode (-fsanitize-trap=undefined) so no runtime library is needed.

if(NOT (MC_ENABLE_ASAN OR MC_ENABLE_UBSAN))
  return()
endif()

if(MSVC AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if(MC_ENABLE_UBSAN)
    message(FATAL_ERROR "MSVC cl does not implement UBSan; configure with clang-cl or a GCC/Clang toolchain.")
  endif()
  add_compile_options(/fsanitize=address /Zi)
  add_link_options(/INCREMENTAL:NO /DEBUG)
  string(REGEX REPLACE "/RTC[1csu]+" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  # The dynamic ASan runtime ships next to cl.exe; copy it beside the executables.
  get_filename_component(_mc_msvc_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
  file(GLOB _mc_asan_dlls "${_mc_msvc_bin}/clang_rt.asan_dynamic-x86_64.dll"
       "${_mc_msvc_bin}/clang_rt.asan_dbg_dynamic-x86_64.dll")
  if(_mc_asan_dlls)
    file(COPY ${_mc_asan_dlls} DESTINATION "${PROJECT_BINARY_DIR}/bin")
  endif()
elseif(MSVC)
  if(MC_ENABLE_ASAN)
    message(FATAL_ERROR "Use MSVC cl (not clang-cl) for AddressSanitizer on Windows.")
  endif()
  add_compile_options(-fsanitize=undefined -fsanitize-trap=undefined)
else()
  set(_mc_sanitizers "")
  if(MC_ENABLE_ASAN)
    list(APPEND _mc_sanitizers address)
  endif()
  if(MC_ENABLE_UBSAN)
    list(APPEND _mc_sanitizers undefined)
  endif()
  list(JOIN _mc_sanitizers "," _mc_sanitizer_list)
  add_compile_options(-fsanitize=${_mc_sanitizer_list} -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=${_mc_sanitizer_list})
endif()
