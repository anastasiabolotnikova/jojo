#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RoombotsController_DLLIMPORT __declspec(dllimport)
#  define RoombotsController_DLLEXPORT __declspec(dllexport)
#  define RoombotsController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RoombotsController_DLLIMPORT __attribute__((visibility("default")))
#    define RoombotsController_DLLEXPORT __attribute__((visibility("default")))
#    define RoombotsController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RoombotsController_DLLIMPORT
#    define RoombotsController_DLLEXPORT
#    define RoombotsController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RoombotsController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RoombotsController_DLLAPI
#  define RoombotsController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RoombotsController_EXPORTS
#    define RoombotsController_DLLAPI RoombotsController_DLLEXPORT
#  else
#    define RoombotsController_DLLAPI RoombotsController_DLLIMPORT
#  endif // RoombotsController_EXPORTS
#  define RoombotsController_LOCAL RoombotsController_DLLLOCAL
#endif // RoombotsController_STATIC