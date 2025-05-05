#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TeleoperationDemoTripod_DLLIMPORT __declspec(dllimport)
#  define TeleoperationDemoTripod_DLLEXPORT __declspec(dllexport)
#  define TeleoperationDemoTripod_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TeleoperationDemoTripod_DLLIMPORT __attribute__((visibility("default")))
#    define TeleoperationDemoTripod_DLLEXPORT __attribute__((visibility("default")))
#    define TeleoperationDemoTripod_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TeleoperationDemoTripod_DLLIMPORT
#    define TeleoperationDemoTripod_DLLEXPORT
#    define TeleoperationDemoTripod_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TeleoperationDemoTripod_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TeleoperationDemoTripod_DLLAPI
#  define TeleoperationDemoTripod_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TeleoperationDemoTripod_EXPORTS
#    define TeleoperationDemoTripod_DLLAPI TeleoperationDemoTripod_DLLEXPORT
#  else
#    define TeleoperationDemoTripod_DLLAPI TeleoperationDemoTripod_DLLIMPORT
#  endif // TeleoperationDemoTripod_EXPORTS
#  define TeleoperationDemoTripod_LOCAL TeleoperationDemoTripod_DLLLOCAL
#endif // TeleoperationDemoTripod_STATIC
