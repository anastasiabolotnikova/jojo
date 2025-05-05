#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TeleoperationDemoArmAssist_DLLIMPORT __declspec(dllimport)
#  define TeleoperationDemoArmAssist_DLLEXPORT __declspec(dllexport)
#  define TeleoperationDemoArmAssist_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TeleoperationDemoArmAssist_DLLIMPORT __attribute__((visibility("default")))
#    define TeleoperationDemoArmAssist_DLLEXPORT __attribute__((visibility("default")))
#    define TeleoperationDemoArmAssist_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TeleoperationDemoArmAssist_DLLIMPORT
#    define TeleoperationDemoArmAssist_DLLEXPORT
#    define TeleoperationDemoArmAssist_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TeleoperationDemoArmAssist_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TeleoperationDemoArmAssist_DLLAPI
#  define TeleoperationDemoArmAssist_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TeleoperationDemoArmAssist_EXPORTS
#    define TeleoperationDemoArmAssist_DLLAPI TeleoperationDemoArmAssist_DLLEXPORT
#  else
#    define TeleoperationDemoArmAssist_DLLAPI TeleoperationDemoArmAssist_DLLIMPORT
#  endif // TeleoperationDemoArmAssist_EXPORTS
#  define TeleoperationDemoArmAssist_LOCAL TeleoperationDemoArmAssist_DLLLOCAL
#endif // TeleoperationDemoArmAssist_STATIC