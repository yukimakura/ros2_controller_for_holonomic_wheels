#ifndef VISIBILITY_CONTROL_H_
#define VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define Omni_3WD_CONTROLLER_EXPORT __attribute__((dllexport))
#define Omni_3WD_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define Omni_3WD_CONTROLLER_EXPORT __declspec(dllexport)
#define Omni_3WD_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef Omni_3WD_CONTROLLER_BUILDING_DLL
#define Omni_3WD_CONTROLLER_PUBLIC Omni_3WD_CONTROLLER_EXPORT
#else
#define Omni_3WD_CONTROLLER_PUBLIC Omni_3WD_CONTROLLER_IMPORT
#endif
#define Omni_3WD_CONTROLLER_PUBLIC_TYPE Omni_3WD_CONTROLLER_PUBLIC
#define Omni_3WD_CONTROLLER_LOCAL
#else
#define Omni_3WD_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define Omni_3WD_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define Omni_3WD_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define Omni_3WD_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define Omni_3WD_CONTROLLER_PUBLIC
#define Omni_3WD_CONTROLLER_LOCAL
#endif
#define Omni_3WD_CONTROLLER_PUBLIC_TYPE
#endif

#endif //VISIBILITY_CONTROL_H_