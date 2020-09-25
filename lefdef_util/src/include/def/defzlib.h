#ifndef LEFDEFZIP_H
#define LEFDEFZIP_H

typedef void* defGZFile;

/* 
 * Name: defGZipOpen
 * Description: Open a gzip file
 * Returns: A file pointer
 */
extern defGZFile defGZipOpen(const char* gzipFile, const char* mode);

/* 
 * Name: defGZipClose
 * Description: Close a gzip file
 * Returns: 0 if no errors
 */
extern int defGZipClose(defGZFile filePtr);

/*
 * Name: defrReadGZip
 * Description: Parse a def gzip file
 * Returns: 0 if no errors
 */
extern int defrReadGZip(defGZFile file, const char* gzipFile, void* uData);

#endif
