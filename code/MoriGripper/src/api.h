#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TeleoperationDemoGripper_DLLIMPORT __declspec(dllimport)
#  define TeleoperationDemoGripper_DLLEXPORT __declspec(dllexport)
#  define TeleoperationDemoGripper_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TeleoperationDemoGripper_DLLIMPORT __attribute__((visibility("default")))
#    define TeleoperationDemoGripper_DLLEXPORT __attribute__((visibility("default")))
#    define TeleoperationDemoGripper_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TeleoperationDemoGripper_DLLIMPORT
#    define TeleoperationDemoGripper_DLLEXPORT
#    define TeleoperationDemoGripper_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TeleoperationDemoGripper_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TeleoperationDemoGripper_DLLAPI
#  define TeleoperationDemoGripper_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TeleoperationDemoGripper_EXPORTS
#    define TeleoperationDemoGripper_DLLAPI TeleoperationDemoGripper_DLLEXPORT
#  else
#    define TeleoperationDemoGripper_DLLAPI TeleoperationDemoGripper_DLLIMPORT
#  endif // TeleoperationDemoGripper_EXPORTS
#  define TeleoperationDemoGripper_LOCAL TeleoperationDemoGripper_DLLLOCAL
#endif // TeleoperationDemoGripper_STATIC
