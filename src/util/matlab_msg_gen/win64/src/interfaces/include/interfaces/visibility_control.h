#ifndef INTERFACES__VISIBILITY_CONTROL_H_
#define INTERFACES__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTERFACES_EXPORT __attribute__ ((dllexport))
    #define INTERFACES_IMPORT __attribute__ ((dllimport))
  #else
    #define INTERFACES_EXPORT __declspec(dllexport)
    #define INTERFACES_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTERFACES_BUILDING_LIBRARY
    #define INTERFACES_PUBLIC INTERFACES_EXPORT
  #else
    #define INTERFACES_PUBLIC INTERFACES_IMPORT
  #endif
  #define INTERFACES_PUBLIC_TYPE INTERFACES_PUBLIC
  #define INTERFACES_LOCAL
#else
  #define INTERFACES_EXPORT __attribute__ ((visibility("default")))
  #define INTERFACES_IMPORT
  #if __GNUC__ >= 4
    #define INTERFACES_PUBLIC __attribute__ ((visibility("default")))
    #define INTERFACES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTERFACES_PUBLIC
    #define INTERFACES_LOCAL
  #endif
  #define INTERFACES_PUBLIC_TYPE
#endif
#endif  // INTERFACES__VISIBILITY_CONTROL_H_
// Generated 23-Feb-2025 23:50:24
 