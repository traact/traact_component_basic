
#ifndef TRAACT_FACADE_EXPORT_H
#define TRAACT_FACADE_EXPORT_H

#ifdef TRAACT_TARGET_WINDOWS	   
#define TRAACT_FACADE_VISIBILITY_EXPORT __declspec(dllexport)
#else
#define TRAACT_FACADE_VISIBILITY_EXPORT __attribute__((visibility("default")))
#endif

#ifdef TRAACT_TARGET_WINDOWS
#define TRAACT_FACADE_VISIBILITY_INLINE_MEMBER_EXPORT
#else
#define TRAACT_FACADE_VISIBILITY_INLINE_MEMBER_EXPORT __attribute__((visibility("default")))
#endif

#ifdef TRAACT_TARGET_WINDOWS
#define TRAACT_FACADE_VISIBILITY_IMPORT __declspec(dllimport)
#else
#define TRAACT_FACADE_VISIBILITY_IMPORT __attribute__((visibility("default")))
#endif

#ifdef TRAACT_TARGET_WINDOWS
#define TRAACT_FACADE_VISIBILITY_LOCAL
#else
#define TRAACT_FACADE_VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#endif

#if defined(traact_facade_EXPORTS)
#define TRAACT_FACADE_EXPORT TRAACT_FACADE_VISIBILITY_EXPORT
#else
#define TRAACT_FACADE_EXPORT TRAACT_FACADE_VISIBILITY_IMPORT
#endif

#endif /* TRAACT_FACADE_EXPORT_H */
