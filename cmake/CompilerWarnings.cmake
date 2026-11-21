# Warning flags for ManipulatorCore's own targets. Third-party code is included as SYSTEM, so these
# flags never apply to Eigen, GoogleTest or Google Benchmark headers.

function(mc_set_project_warnings target)
  set(msvc_warnings
      /W4
      /permissive-
      /w14242 # conversion, possible loss of data
      /w14254 # larger bit-field to smaller bit-field
      /w14263 # member function does not override a base virtual function
      /w14265 # class has virtual functions but non-virtual destructor
      /w14287 # unsigned/negative constant mismatch
      /w14296 # expression is always true/false
      /w14311 # pointer truncation
      /w14545 /w14546 /w14547 /w14549 /w14555 # suspicious comma/expressions
      /w14619 # unknown warning number in pragma
      /w14640 # thread-unsafe static member initialization
      /w14826 # sign-extended conversion
      /w14905 /w14906 # string literal casts
      /w14928 # illegal copy-initialization
  )
  set(clang_cl_warnings /W4 -Wpedantic -Wshadow -Wnon-virtual-dtor -Woverloaded-virtual -Wcast-align
                        -Wold-style-cast -Wno-c++98-compat -Wno-c++98-compat-pedantic)
  set(gnu_warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wshadow
      -Wnon-virtual-dtor
      -Woverloaded-virtual
      -Wcast-align
      -Wold-style-cast
      -Wnull-dereference
      -Wimplicit-fallthrough)
  set(gcc_only_warnings -Wduplicated-cond -Wlogical-op -Wuseless-cast)

  if(MC_WARNINGS_AS_ERRORS)
    list(APPEND msvc_warnings /WX)
    list(APPEND clang_cl_warnings /WX)
    list(APPEND gnu_warnings -Werror)
  endif()

  if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(flags ${clang_cl_warnings})
  elseif(MSVC)
    set(flags ${msvc_warnings})
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(flags ${gnu_warnings})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(flags ${gnu_warnings} ${gcc_only_warnings})
  endif()

  target_compile_options(${target} PRIVATE ${flags})
  if(MSVC)
    # UTF-8 sources, standard-conforming __cplusplus, and external headers at warning level 0.
    target_compile_options(${target} PRIVATE /utf-8 /Zc:__cplusplus /external:W0)
  endif()

  if(MC_CLANG_TIDY_COMMAND)
    set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${MC_CLANG_TIDY_COMMAND}")
  endif()
endfunction()
