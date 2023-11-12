#ifndef VISIBILITY_CONTROL_H_
#define VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define QUAD_OMNI_CONTROLLER_EXPORT __attribute__((dllexport))
#define QUAD_OMNI_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define QUAD_OMNI_CONTROLLER_EXPORT __declspec(dllexport)
#define QUAD_OMNI_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef QUAD_OMNI_CONTROLLER_BUILDING_DLL
#define QUAD_OMNI_CONTROLLER_PUBLIC QUAD_OMNI_CONTROLLER_EXPORT
#else
#define QUAD_OMNI_CONTROLLER_PUBLIC QUAD_OMNI_CONTROLLER_IMPORT
#endif
#define QUAD_OMNI_CONTROLLER_PUBLIC_TYPE QUAD_OMNI_CONTROLLER_PUBLIC
#define QUAD_OMNI_CONTROLLER_LOCAL
#else
#define QUAD_OMNI_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define QUAD_OMNI_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define QUAD_OMNI_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define QUAD_OMNI_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define QUAD_OMNI_CONTROLLER_PUBLIC
#define QUAD_OMNI_CONTROLLER_LOCAL
#endif
#define QUAD_OMNI_CONTROLLER_PUBLIC_TYPE
#endif

#endif //VISIBILITY_CONTROL_H_