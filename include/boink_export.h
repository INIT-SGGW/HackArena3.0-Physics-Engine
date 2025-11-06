
#ifndef BOINK_EXPORT_H
#define BOINK_EXPORT_H

#ifdef BOINK_STATIC_DEFINE
#  define BOINK_EXPORT
#  define BOINK_NO_EXPORT
#else
#  ifndef BOINK_EXPORT
#    ifdef boink_EXPORTS
        /* We are building this library */
#      define BOINK_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define BOINK_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef BOINK_NO_EXPORT
#    define BOINK_NO_EXPORT 
#  endif
#endif

#ifndef BOINK_DEPRECATED
#  define BOINK_DEPRECATED __declspec(deprecated)
#endif

#ifndef BOINK_DEPRECATED_EXPORT
#  define BOINK_DEPRECATED_EXPORT BOINK_EXPORT BOINK_DEPRECATED
#endif

#ifndef BOINK_DEPRECATED_NO_EXPORT
#  define BOINK_DEPRECATED_NO_EXPORT BOINK_NO_EXPORT BOINK_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef BOINK_NO_DEPRECATED
#    define BOINK_NO_DEPRECATED
#  endif
#endif

#endif /* BOINK_EXPORT_H */
