#ifndef CPCC2_TIAGO__VISIBILITY_CPCC2_TIAGO_H_
#define CPCC2_TIAGO__VISIBILITY_CPCC2_TIAGO_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CPCC2_TIAGO_EXPORT __attribute__((dllexport))
#define CPCC2_TIAGO_IMPORT __attribute__((dllimport))
#else
#define CPCC2_TIAGO_EXPORT __declspec(dllexport)
#define CPCC2_TIAGO_IMPORT __declspec(dllimport)
#endif
#ifdef CPCC2_TIAGO_BUILDING_DLL
#define CPCC2_TIAGO_PUBLIC CPCC2_TIAGO_EXPORT
#else
#define CPCC2_TIAGO_PUBLIC CPCC2_TIAGO_IMPORT
#endif
#define CPCC2_TIAGO_PUBLIC_TYPE CPCC2_TIAGO_PUBLIC
#define CPCC2_TIAGO_LOCAL
#else
#define CPCC2_TIAGO_EXPORT __attribute__((visibility("default")))
#define CPCC2_TIAGO_IMPORT
#if __GNUC__ >= 4
#define CPCC2_TIAGO_PUBLIC __attribute__((visibility("default")))
#define CPCC2_TIAGO_LOCAL __attribute__((visibility("hidden")))
#else
#define CPCC2_TIAGO_PUBLIC
#define CPCC2_TIAGO_LOCAL
#endif
#define CPCC2_TIAGO_PUBLIC_TYPE
#endif

#endif  // CPCC2_TIAGO__VISIBILITY_CPCC2_TIAGO_H_
