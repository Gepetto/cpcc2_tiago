#pragma once

// STL
#include <version>

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CPCC2_TIAGO_EXPORT [[gnu::dllexport]]
#define CPCC2_TIAGO_IMPORT [[gnu::dllimport]]
#else
#define CPCC2_TIAGO_EXPORT [[msvc::dllexport]]
#define CPCC2_TIAGO_IMPORT [[msvc::dllimport]]
#endif
#ifdef CPCC2_TIAGO_BUILDING_DLL
#define CPCC2_TIAGO_PUBLIC CPCC2_TIAGO_EXPORT
#else
#define CPCC2_TIAGO_PUBLIC CPCC2_TIAGO_IMPORT
#endif
#define CPCC2_TIAGO_PUBLIC_TYPE CPCC2_TIAGO_PUBLIC
#define CPCC2_TIAGO_LOCAL
#else
#define CPCC2_TIAGO_EXPORT [[gnu::visibility("default")]]
#define CPCC2_TIAGO_IMPORT
#if __GNUC__ >= 4
#define CPCC2_TIAGO_PUBLIC [[gnu::visibility("default")]]
#define CPCC2_TIAGO_LOCAL [[gnu::visibility("hidden")]]
#else
#define CPCC2_TIAGO_PUBLIC
#define CPCC2_TIAGO_LOCAL
#endif
#define CPCC2_TIAGO_PUBLIC_TYPE
#endif
