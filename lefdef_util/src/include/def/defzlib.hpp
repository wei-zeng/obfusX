#ifndef LEFDEFZIP_H
#define LEFDEFZIP_H

/* Definitions header file for K&R related defines */

#if defined(__STDC__) || defined(__cplusplus) || defined(WIN32)

#ifndef PROTO_PARAMS
#define PROTO_PARAMS(params) params
#define DEFINED_PROTO_PARAMS
#endif

#ifndef VAR_ARG_LIST
#define VAR_ARG_LIST ...
#define DEFINED_VAR_ARG_LIST
#endif

#ifndef EXTERN
#ifdef  LEFI_CPP_ANSI
#define EXTERN extern "C"
#else
#define EXTERN
#endif
#define DEFINED_EXTERN
#endif

#else

#ifndef PROTO_PARAMS
#define PROTO_PARAMS(params) (/* nothing */)
#define DEFINED_PROTO_PARAMS
#endif

#ifndef VAR_ARG_LIST
#define VAR_ARG_LIST va_tdcl(char *)
#define DEFINED_VAR_ARG_LIST
#endif

#ifndef EXTERN
#define EXTERN extern
#define DEFINED_EXTERN
#ifndef DEFINED_CONST
#define const /* nothing */
#define DEFINED_CONST
#endif
#endif

#endif /* __STDC__ */

typedef void* defGZFile;

/* 
 * Name: defGZipOpen
 * Description: Open a gzip file
 * Returns: A file pointer
 */
EXTERN defGZFile defGZipOpen(const char* gzipFile, const char* mode);

/* 
 * Name: defGZipClose
 * Description: Close a gzip file
 * Returns: 0 if no errors
 */
EXTERN int defGZipClose(defGZFile filePtr);

/*
 * Name: defrReadGZip
 * Description: Parse a def gzip file
 * Returns: 0 if no errors
 */
EXTERN int defrReadGZip(defGZFile file, const char* gzipFile, void* uData);

#endif
