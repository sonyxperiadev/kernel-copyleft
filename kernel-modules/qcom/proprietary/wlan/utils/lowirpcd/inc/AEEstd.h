/*
 * Copyright (c) 2004-2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc
 */
#ifndef AEESTD_H
#define AEESTD_H
/*====================================================================

DESCRIPTION:  Standard library; general-purpose utility functions.

====================================================================*/
#include "AEEVaList.h"
#include "AEEStdDef.h"

#define STD_CONSTRAIN( val, min, max ) (((val) < (min)) ? (min) : ((val) > (max)) ? (max) : (val))
#define STD_BETWEEN( val, minGE, maxLT ) \
                   ( (unsigned long)((unsigned long)(val) - (unsigned long)(minGE)) < \
                   (unsigned long)((unsigned long)(maxLT) - (unsigned long)(minGE)) )
#define STD_ARRAY_SIZE(a)             ((int)((sizeof((a))/sizeof((a)[0]))))
#define STD_ARRAY_MEMBER(p,a) (((p) >= (a)) && ((p) < ((a) + STD_ARRAY_SIZE(a))))

#define STD_SIZEOF(x)                 ((int)sizeof(x))
#define STD_OFFSETOF(type,member)     (((char*)(&((type*)1)->member))-((char*)1))

#define STD_RECOVER_REC(type,member,p) ((void)((p)-&(((type*)1)->member)),\
                                        (type*)(void*)(((char*)(void*)(p))-STD_OFFSETOF(type,member)))
#define STD_MIN(a,b)   ((a)<(b)?(a):(b))
#define STD_MAX(a,b)   ((a)>(b)?(a):(b))
//lint -emacro(545,STD_ZEROAT)
#define STD_ZEROAT(p)  std_memset((p), 0, sizeof(*p))

#define _STD_BITS_PER(bits)     (8*sizeof((bits)[0]))

#define STD_BIT_SET(bits, ix)     ((bits)[(ix)/_STD_BITS_PER((bits))] |= 0x1<<((ix) & (_STD_BITS_PER((bits))-1)))
#define STD_BIT_CLEAR(bits, ix)   ((bits)[(ix)/_STD_BITS_PER((bits))] &= ~(0x1<<((ix) & (_STD_BITS_PER((bits))-1))))
#define STD_BIT_TEST(bits, ix)    ((bits)[(ix)/_STD_BITS_PER((bits))] & (0x1<<((ix) & (_STD_BITS_PER((bits))-1))))

//
// Error codes
//
#define STD_NODIGITS   1
#define STD_NEGATIVE   2
#define STD_OVERFLOW   3
#define STD_BADPARAM   4
#define STD_UNDERFLOW  5

#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

//Version function
extern int           std_getversion(char *pcDst, int nDestSize);

//String functions
extern int           std_strlen(const char *s);
extern int           std_strcmp(const char *s1, const char *s2);
extern int           std_strncmp(const char *s1, const char *s2, int n);
extern int           std_stricmp(const char *s1, const char *s2);
extern int           std_strnicmp(const char *s1, const char *s2, int n);
extern int           std_strlcpy(char *pcDst, const char *pszSrc, int nDestSize);
extern int           std_strlcat(char *pcDst, const char *pszSrc, int nDestSize);
extern char *        std_strstr(const char *pszString, const char *pszSearch);

//Character functions
extern char          std_tolower(char c);
extern char          std_toupper(char c);

// Mem functions
extern void *        std_memset(void *p, int c, int nLen);
extern void *        std_memmove(void *pTo, const void *cpFrom, int nLen);
extern int           std_memscpy(void *dst, int dst_size, const void *src, int src_size);
extern int           std_memsmove(void *dst, int dst_size, const void *src, int src_size);
extern int           std_memcmp(const void *a, const void *b, int length);
extern void *        std_memchr(const void* s, int c, int n);
extern void *        std_memstr(const char* cpHaystack, const char* cpszNeedle, int nHaystackLen);
extern void *        std_memrchr(const void* s, int c, int n);
extern void *        std_memrchrbegin(const void* p, int c, int nLen);
extern void *        std_memchrend(const void* cpcSrch, int c, int nLen);
extern void *        std_memchrsend(const void *cpSrch, const char* cpszChars, int nLen);

//Other String functions
extern char *        std_strchr(const char* s, int c);
extern char *        std_strchrs(const char* sSrch, const char *sChars);
extern char *        std_strrchr(const char* s, int c);
extern char *        std_strchrend(const char *cpszSrch, char c);
extern char *        std_strchrsend(const char* s, const char* cpszSrch);
extern char *        std_strends(const char* cpsz, const char* cpszSuffix);
extern char *        std_striends(const char* cpsz, const char* cpszSuffix);
extern char *        std_strbegins(const char* cpsz, const char* cpszPrefix);
extern char *        std_stribegins(const char* cpsz, const char* cpszPrefix);
extern int           std_strcspn(const char* s, const char* cpszSrch);
extern int           std_strspn(const char* s, const char* cpszSrch);

//Wide char string functions
extern int           std_wstrlen(const AECHAR *s);
extern int           std_wstrlcpy(AECHAR *pcDst, const AECHAR *pszSrc, int nDestSize);
extern int           std_wstrlcat(AECHAR *pcDst, const AECHAR *pszSrc, int nDestSize);
extern int           std_wstrncmp(const AECHAR* s1, const AECHAR* s2, int nLen);
extern int           std_wstrcmp(const AECHAR* s1, const AECHAR* s2);
extern AECHAR*       std_wstrchr(const AECHAR* cpwszText, AECHAR ch);
extern AECHAR*       std_wstrrchr(const AECHAR* cpwszText, AECHAR ch);

//Path functions
extern int           std_makepath(const char *cpszDir,
                                  const char *cpszFile,
                                  char *pszDest, int nDestSize);
extern char *        std_splitpath(const char *cpszPath, const char *cpszDir);
extern char *        std_cleanpath(char *pszPath);
extern char *        std_basename(const char *pszPath);

//Inet functions, number functions
extern unsigned int        std_scanul(const char *pchBuf, int nRadix,
                                const char **ppchEnd, int *pnError);
extern uint64        std_scanull(const char *pchBuf, int nRadix,
                                 const char **ppchEnd, int *pnError);
extern double        std_scand(const char *pchBuf, const char **ppchEnd);

// Rand functions
extern unsigned      std_rand_next(unsigned uRand);
extern uint32        std_rand(uint32 uSeed, byte* pDest, int nSize);


// printf functions
extern int           std_vstrlprintf(char *pszDest, int nDestSize,
                                     const char *pszFmt, AEEVaList args);

extern int           std_strlprintf(char *pszDest, int nDestSize,
                                    const char *pszFmt, ...);

extern int           std_vsnprintf(char *pszDest, int nDestSize,
                                   const char *cpszFmt, AEEVaList args);

extern int           std_snprintf(char *pszDest, int nDestSize,
                                  const char *pszFmt, ...);

// endian swapping functions
extern int           std_CopyLE(void *pvDest,       int nDestSize,
                                const void *pvSrc,  int nSrcSize,
                                const char *pszFields);

extern int           std_CopyBE(void *pvDest,      int nDestSize,
                                const void *pvSrc, int nSrcSize,
                                const char *pszFields);

// sorting utilities
extern void  std_qsort(void* pElems, int nNumElems, int nElemWidth,
                       int (*pfnCompare)(void*, const void*, const void*),
                       void* pCompareCx);

extern int   std_bisect(const void* pElems, int nNumElems, int nElemWidth,
                        const void* pElem,
                        int (*pfnCompare)(void*, const void*, const void*),
                        void* pCompareCx);

extern void  std_merge(void* vpDst, int nDst,
                       const void* vpA, int nA,
                       const void* vpB, int nB,
                       int nElemWidth,
                       int (*pfnCompare)(void*, const void*, const void*),
                       void* pCompareCx);

extern int   std_uniq(void* vpElems, int nNumElems, int nElemWidth,
                      int (*pfnCompare)(void*, const void*, const void*),
                      void* pCompareCx);

#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */


#define STD_SWAPS(us) \
   ((((us) & 0xff) << 8) + (((us) & 0xff00) >> 8))


static __inline unsigned short std_swaps(unsigned short us)
{
   return STD_SWAPS(us);
}

/* note, STD_SWAPL() requires that ul be an l-value, and destroyable.
   this macro is not intended for use outside AEEstd.h */
#define STD_SWAPL(ul) \
    (((ul) = (((ul) & 0x00ff00ff) << 8) | (((ul)>>8) & 0x00ff00ff)),(((ul) >> 16) | ((ul) << 16)))

static __inline unsigned long std_swapl(unsigned long ul)
{
   return STD_SWAPL(ul);
}

#ifdef AEE_BIGENDIAN
#  define STD_HTONL(u)     (u)
#  define STD_HTONS(u)     (u)
#  define STD_HTOLEL(u)    (STD_SWAPL(u))
#  define STD_HTOLES(u)    (STD_SWAPS(u))
#else
#  define STD_HTONL(u)     (STD_SWAPL(u))
#  define STD_HTONS(u)     (STD_SWAPS(u))
#  define STD_HTOLEL(u)    (u)
#  define STD_HTOLES(u)    (u)
#endif

static __inline unsigned short std_letohs(unsigned short us)
{
   return STD_HTOLES(us);
}

static __inline unsigned short std_htoles(unsigned short us)
{
   return STD_HTOLES(us);
}

static __inline unsigned long std_letohl(unsigned long ul)
{
   return STD_HTOLEL(ul);
}

static __inline unsigned long std_htolel(unsigned long ul)
{
   return STD_HTOLEL(ul);
}

static __inline unsigned short std_ntohs(unsigned short us)
{
   return STD_HTONS(us);
}

static __inline unsigned short std_htons(unsigned short us)
{
   return STD_HTONS(us);
}

static __inline unsigned long std_ntohl(unsigned long ul)
{
   return STD_HTONL(ul);
}

static __inline unsigned long std_htonl(unsigned long ul)
{
   return STD_HTONL(ul);
}


#undef STD_HTONL   // private macro; not exported as a supported API
#undef STD_HTONS   // private macro; not exported as a supported API
#undef STD_HTOLEL  // private macro; not exported as a supported API
#undef STD_HTOLES  // private macro; not exported as a supported API
#undef STD_SWAPS   // private macro; not exported as a supported API
#undef STD_SWAPL   // private macro; not exported as a supported API


/*
=======================================================================
MACROS DOCUMENTATION
=======================================================================

STD_CONTSTRAIN()

Description:
  STD_CONTSTRAIN() constrains a number to be between two other numbers.

Definition:
   STD_CONSTRAIN( val, min, max ) \
          (((val) < (min)) ? (min) : ((val) > (max)) ? (max) : (val))

Parameters:
  val: number to constrain
  min: number to stay greater than or equal to
  max: number to stay less than or equal to

Evaluation Value:
   the constrained number

=======================================================================

STD_BETWEEN()

Description:
    STD_BETWEEN() tests whether a number is between two other numbers.

Definition:
    STD_BETWEEN( val, minGE, maxLT ) \
               ((unsigned)((unsigned)(val) - (unsigned)(minGE)) < \
                (unsigned)((unsigned)(maxLT) - (unsigned)(minGE)))

Parameters:
     val: value to test
     minGE: lower bound
     maxLT: upper bound

Evaluation Value:
     1 if val >= minGE and val < maxLT

=======================================================================

STD_ARRAY_SIZE()

Description:
   STD_ARRAY_SIZE() gives the number of elements in a statically allocated array.

Definition:
    STD_ARRAY_SIZE(a) (sizeof((a))/sizeof((a)[0]))

Parameters:
    a: array to test

Evaluation Value:
    number of elements in a

=======================================================================

STD_ARRAY_MEMBER()

Description:
   STD_ARRAY_MEMBER() tests whether an item is a member of a statically allocated array.

Definition:
   STD_ARRAY_MEMBER(p,a) (((p) >= (a)) && ((p) < ((a) + STD_ARRAY_SIZE(a))))

Parameters:
    p: item to test
    a: array to check

Evaluation Value:
    1 if p is in a

=======================================================================

STD_OFFSETOF()

Description:
  STD_OFFSETOF() gives the offset of member of a struct.

Definition:
   STD_OFFSETOF(type,member) (((char *)(&((type *)0)->member))-((char *)0))

Parameters:
    type: structured type
    member: name of member in the struct

Evaluation Value:
    offset of member (in bytes) in type

=======================================================================

STD_RECOVER_REC()

Description:
  STD_RECOVER_REC() provides a safe cast from a pointer to a member
    of a struct to a pointer to the containing struct

Definition:
  STD_RECOVER_REC(type,member,p) ((type*)(((char*)(p))-STD_OFFSETOF(type,member)))

Parameters:
    type: structured type
    member: name of member in the struct
    p: pointer to the member of the struct

Evaluation Value:
    a pointer of type type to the containing struct

=======================================================================

STD_MIN()

Description:
   STD_MIN() finds the smaller of two values.

Definition:
   STD_MIN(a,b)   ((a)<(b)?(a):(b))

Parameters:
    a, b: values to compare

Evaluation Value:
    smaller of a and b

=======================================================================

STD_MAX()

Description:
  STD_MAX() finds the larger of two values.

Definition:
   STD_MAX(a,b)   ((a)>(b)?(a):(b))

Parameters:
    a, b: values to compare

Evaluation Value:
    larger of a and b

=======================================================================

STD_ZEROAT()

Description:
   STD_ZEROAT() zero-initializes the contents of a typed chunk of memory.

Definition:
   STD_ZEROAT(p)  std_memset((p), 0, sizeof(*p))

Parameters:
    p: the chunk to initialize

Evaluation Value:
    p

=======================================================================

STD_BIT_SET()

Description:
   STD_BIT_SET(bits, ix) sets the bit in the memory stored in bits at
                         index ix

Parameters:
    bits: the memory address holding the  bits
    ix:   the index of the bit to set;

=======================================================================

STD_BIT_CLEAR()

Description:
   STD_BIT_CLEAR(bits, ix) clears the bit in the memory stored in bits
                           at index ix

Parameters:
    bits: the memory address holding the  bits
    ix:   the index of the bit to clear

=======================================================================

STD_BIT_TEST()

Description:
   STD_BIT_TEST(bits, ix) returns the bit in the memory stored in bits
                          at index ix

Parameters:
    bits: the memory address holding the bits
    ix:   the index of the bit to test

Evaluation Value:
    0x1 if set 0x0 if not set

=====================================================================
INTERFACES DOCUMENTATION
=======================================================================

std Interface

Description:
   This library provides a set of general-purpose utility functions.
   Functionality may overlap that of a subset of the C standard library, but
   this library differs in a few respects:

   - Functions are fully reentrant and avoid use of static variables.

   - The library can be supported consistently across all environments.
   Compiler-supplied libraries sometimes behave inconsistently and are
   unavailable in some environments.

   - Omits "unsafe" functions.  C standard library includes many functions
   that are best avoided entirely: strcpy, strcat, strtok, etc.


=======================================================================

std_getversion()

Description:

   The std_getversion() copies the stdlib version to pcDst.  This function
   takes the size of the destination buffer as an argument and guarantees
   to zero-terminate the result and not to overflow the nDestSize size.

   This function copies up to size-1 characters from the stdlib version
   string to pcDest and NUL-terminates the pcDest string.

Prototype:
   int  std_getversion(char *pcDst, int nDestSize)


Parameters:
   pcDst :     Destination string
   nDestSize:  Size of the destination buffer in bytes

Return Value:

   Returns the length of the version string (in characters).

=======================================================================

std_strlen()

Description:
   The std_strlen() computes the length of the given string.

Prototype:
   int std_strlen(const char *cpszStr)

Parameters:
   cpszStr : String whose length will be computed

Return Value:
   Length of the string in characters that precede the terminating NULL character.

=======================================================================

std_strcmp()

Description:
   The std_strcmp() compares two NUL-terminated character strings.
   Comparison is strictly by byte values with no character set
   interpretation.

Prototype:

   int std_strcmp(const char *s1, const char *s2);

Parameters:
   s1, s2: strings to compare

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

See Also:
   std_wstrcmp

=======================================================================

std_strncmp()

Description:
   The std_strncmp() compares at most n bytes of two NUL-terminated character strings.

Prototype:

   int std_strncmp(const char *s1, const char *s2, int n);

Parameters:
   s1, s2: strings to compare
   n: maximum number of bytes to compare.  if either s1 or s2 is
       shorter than n, the function terminates there

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

See Also:
   std_wstrncmp

=======================================================================

std_stricmp()

Description:
   The std_stricmp() compares two NUL-terminated character strings, case-folding any
   ASCII characters.

Prototype:

   int std_stricmp(const char *s1, const char *s2);

Parameters:
   s1, s2: strings to compare

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

=======================================================================

std_strnicmp()

Description:
   The std_strnicmp() compares at most n bytes of 2 NUL-terminated character strings,
   case-folding any ASCII characters.

Prototype:

   int std_strnicmp(const char *s1, const char *s2, int n);

Parameters:
   s1, s2: strings to compare
   n: maximum number of bytes to compare.  if either s1 or s2 is
       shorter than n, the function terminates there

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

=======================================================================

std_strlcpy()

Description:

   The std_strlcpy() copies pszSrc string to the pcDst.  It is a safer
   alternative to strcpy() or strncpy().  This function takes the size of the
   destination buffer as an argument and guarantees to NUL-terminate the
   result and not to overflow the nDestSize size.

   This function copies up to nDestSize-1 characters from the pszSrc string
   to pcDest and NUL-terminates the pcDest string.

Prototype:
   int std_strlcpy(char *pcDst, const char *pszSrc, int nDestSize)

Parameters:
   pcDst :     Destination string
   pcSrc :     Source string
   nDestSize:  Size of the destination buffer in bytes

Return Value:

   Returns the length of the string (in characters) it tried to create,
   which is same as length of pszSrc.

   Example:

   {
      char buf[64];
      if (std_strlcpy(buf, file_name, STD_ARRAY_SIZE(buf) >=
          STD_ARRAY_SIZE(buf)) {
         //Truncated -- Handle overflow....
      }
   }

Comment:

   Unlike strlcpy, std_strlcpy accepts an integer size and does nothing when a
   negative value is passed.  When passing valid sizes for objects on our
   supported platforms, this should not result in any observed difference.
   However, calling strlcpy() with UINT_MAX will result in the entire source
   string being copied, whereas std_strlcpy() will do nothing.  Passing INT_MAX
   to str_strlcpy() will achieve the same result (although both these cases are
   bad practice since they defeat bounds checking).


=======================================================================

std_strlcat()

Description:

   The std_strlcat() function concatenates a string to a string already
   residing in a buffer.  It is a safer alternative to strcat() or strncat().
   This function takes the size of the destination buffer as an argument and
   guarantees not to create an improperly terminated string and not to
   overflow the nDestSize size.

   This function appends pszSrc to pcDst, copying at most nDestSize minus
   the length of the string in pcDest minus 1 bytes, always NUL-terminating
   the result.

   For compatibility with "strlcat()", std_strlcat() does *not* zero-terminate
   the destination buffer in cases where the buffer lacks termination on entry
   to the function.  Do not rely on std_strlcat() to zero-terminate a buffer
   that is not already zero-terminated; instead ensure that the buffer is
   properly initialized using std_strlcpy() or some other means.

Prototype:

   int std_strlcat(char *pcDst, const char *pszSrc, int nDestSize)

Parameters:

   pcDst :     Destination string
   pcSrc :     Source string
   nDestSize:  Size of the destination buffer in bytes

Return Value:

   Returns the length of the string (in characters) it tried to create,
   which is same as length of pszSrc plus the length of pszDest.

   Example:

   {
      char buf[64];
      if (std_strlcat(buf, file_name, STD_ARRAY_SIZE(buf) >=
          STD_ARRAY_SIZE(buf)) {
         //Truncated -- Handle overflow....
      }
   }


=======================================================================

std_strstr()

Description:
   The std_strstr() finds the first occurrence of a substring in a string.

Prototype:

   char * std_strstr(const char *pszString, const char *pszSearch);

Parameters:
   pszString: string to search
   pszSearch: sub string to search for

Return Value:
   A pointer to the first character in the first occurrence of the substring if found, NULL otherwise

=======================================================================

std_tolower()

Description:
   The std_tolower() converts an uppercase letter to the corresponding
   lowercase letter.

Prototype:
   char std_tolower(char c);

Parameters:
   c: A character.

Return Value:
   the corresponding lowercase letter if c is an ASCII character whose
   value is representable as an uppercase letter, else the same character
   c is returned.

=======================================================================

std_toupper()

Description:
   The std_toupper() converts an lowercase letter to the corresponding
   uppercase letter.

Prototype:
   char std_toupper(char c);

Parameters:
   c: is a character.

Return Value:
   The corresponding uppercase letter if c is an ASCII character whose
   value is representable as an lowercase letter; else the same character
   c is returned.

=======================================================================

std_memset()

Description:
   The std_memset() sets each byte in a block of memory to a value.

Prototype:

   void *std_memset(void *p, int c, int nLen);

Parameters:
   p: memory block to set
   c: value to set each byte to
   nLen: size of p in bytes

Return Value:
   p

=======================================================================

std_memmove()

Description:
   The std_memmove() copies a block of memory from one buffer to another.

Prototype:

   void *std_memmove(void *pTo, const void *cpFrom, int nLen);

Parameters:
   pTo: destination buffer
   cpFrom: source buffer
   nLen: number of bytes to copy

Return Value:
   pTo

=======================================================================

std_memsmove()

Description:

  Size bounded memory move.

  Moves bytes from the source buffer to the destination buffer.

  This function ensures that there will not be a copy beyond
  the size of the destination buffer.

  This function should be used in preference to memscpy() if there
  is the possiblity of source and destination buffers overlapping.
  The result of the operation is defined to be as if the copy were from
  the source to a temporary buffer that overlaps neither source nor
  destination, followed by a copy from that temporary buffer to the
  destination.

Prototype:

  int std_memsmove(void *dst, int dst_size, const void *src, int src_size);

Parameters:
  @param[out] dst       Destination buffer.
  @param[in]  dst_size  Size of the destination buffer in bytes.
  @param[in]  src       Source buffer.
  @param[in]  src_size  Number of bytes to copy from source buffer.

Return value:
  The number of bytes copied to the destination buffer.  It is the
  caller's responsibility to check for trunction if it cares about it -
  truncation has occurred if the return value is less than src_size.
  A negative return value indicates an error

=======================================================================

std_memcmp()

Description:
   The std_memcmp() compares two memory buffers, byte-wise.

Prototype:

   int std_memcmp(const void *a, const void *b, int length);

Parameters:
   a, b: buffers to compare
   length: number of bytes to compare

Return Value:
   0 if buffers are the same for nLength ~
   < 0 if a is less than b ~
   > 0 if a is greater than b

=======================================================================

std_memscpy - Size bounded memory copy.

Description:

  Copies bytes from the source buffer to the destination buffer.

  This function ensures that there will not be a copy beyond
  the size of the destination buffer.

  The result of calling this on overlapping source and destination
  buffers is undefined.

Prototype:

   int std_memscpy(void *dst, int dst_size, const void *src, int src_size);

Parameters:

  @param[out] dst       Destination buffer.
  @param[in]  dst_size  Size of the destination buffer in bytes.
  @param[in]  src       Source buffer.
  @param[in]  src_size  Number of bytes to copy from source buffer.

Return value:

  The number of bytes copied to the destination buffer.  It is the
  caller's responsibility to check for trunction if it cares about it -
  truncation has occurred if the return value is less than src_size.
  Returs a negative value on error.

=======================================================================

std_memchr()

Description:
   The std_memchr() finds the first occurrence of a character in a memory
   buffer.

Prototype:

   void *std_memchr(const void* s, int c, int n);

Parameters:
   s: buffer to search
   c: value of byte to look for
   n: size of s in bytes

Return Value:
   A pointer to the occurrence of c. NULL if not found.

=======================================================================

std_memstr()

Description:
   The std_memstr() finds the first occurrence of a substring in a memory
   buffer.

Prototype:

   void *std_memstr(const char* cpHaystack, const char* cpszNeedle,
                    int nHaystackLen);

Parameters:
   cpHaystack: buffer to search
   cpszNeedle: NUL-terminated string to search for
   nHaystackLen: size of cpHaystack in bytes

Return Value:
   a pointer to the first occurrence of cpszNeedle if found,
       NULL otherwise

Comments:
   None

Side Effects:
   None

See Also:
   None

=======================================================================

std_memrchr()

Description:

   The std_memrchr() finds the last occurrence of a character in a memory
   buffer.

Prototype:

   void *std_memrchr(const void* s, int c, int n);

Parameters:
   s: buffer to search
   c: value of byte to look for
   n: size of s in bytes

Return Value:
   a pointer to the last occurrence of c, NULL if not found

=======================================================================

std_memrchrbegin()

Description:
   The std_memrchrbegin() finds the last occurrence of a character in a
   memory buffer.

Prototype:

   void *std_memrchrbegin(const void* s, int c, int n);

Parameters:
   s: buffer to search
   c: value of byte to look for
   n: size of s in bytes

Return Value:
   a pointer to the last occurrence of c, or s if not found

=======================================================================

std_memchrend()

Description:
   The std_memchrend() finds the first occurrence of a character in a
   memory buffer.

Prototype:

   void *std_memchrend(const void* s, int c, int n);

Parameters:
   s: buffer to search
   c: value of byte to look for
   n: size of s in bytes

Return Value:
   a pointer to the occurrence of c, s + n if not found

=======================================================================
std_memchrsend()

Description:
   The std_memchrsend() finds the first occurrence of any character in a
   NUL-terminated list of characters in a memory buffer.

Prototype:

   void *std_memchrend(const void* s, const char* cpszChars, int n);

Parameters:
   s: buffer to search
   cpszChars: characters to look for
   n: size of s in bytes

Return Value:
   a pointer to the first occurrence of one of cpszChars, s + n if not found

=======================================================================

std_strchr()

Description:
   The std_strchr() finds the first occurrence of a character in a
   NUL-terminated string.

Prototype:

   char *std_strchr(const char* s, int c);

Parameters:
   s: string to search
   c: char to search for

Return Value:
   pointer to first occurrence, NULL if not found

See Also:
   std_wstrchr

=======================================================================

std_strchrs()

Description:
   The std_strchrs() searches s, a NUL-terminated string, for the first
   occurrence of any characters in cpszSrch, a NUL-terminated list of
   characters.

Prototype:

   char *std_strchrs(const char* s, const char *cpszSrch);

Parameters:
   s: string to search
   cpszSrch: a list of characters to search for

Return Value:
   first occurrence of any of cpszSrch, NULL if not found

=======================================================================

std_strrchr()

Description:
   The std_strrchr() finds the last occurrence of a character in a
   NUL-terminated string.

Prototype:

   char *std_strrchr(const char* s, int c);

Parameters:
   s: string to search
   c: char to search for

Return Value:
   pointer to last occurrence, NULL if not found

See Also:
   std_wstrrchr

=======================================================================

std_strchrend()

Description:
   The std_strchrend() finds the first occurrence of a character in a
   NUL-terminated string.

Prototype:

   char *std_strchrend(const char* s, int c);

Parameters:
   s: string to search
   c: char to search for

Return Value:
   pointer to first occurrence, s + std_strlen(s) if not found

=======================================================================

std_strchrsend()

Description:
   The std_strchrsend() searches s, a NUL-terminated string, for the first
   occurrence of any characters in cpszSrch, a NUL-terminated list of
   characters.

Prototype:

   char *std_strchrsend(const char* s, const char* cpszSrch);

Parameters:
   s: string to search
   cpszSrch: a list of characters to search for

Return Value:
   first occurrence of any of cpszSrch or s+strlen(s) if not found

=======================================================================

std_strends()

Description:
   The std_strends() tests whether a string ends in a particular suffix.

Prototype:

   char *std_strends(const char* cpsz, const char* cpszSuffix);

Parameters:
   cpsz: string to test
   cpszSuffix: suffix to test for

Return Value:
   the first character of cpsz+std_strlen(cpsz)-std_strlen(cpszSuffix)
     if cpsz ends with cpszSuffix.  NULL otherwise.

=======================================================================

std_striends()

Description:
   The std_striends() tests whether a string ends in a particular suffix,
   case-folding ASCII characters.

Prototype:

   char *std_striends(const char* cpsz, const char* cpszSuffix);

Parameters:
   cpsz: string to test
   cpszSuffix: suffix to test for

Return Value:
   the first character of cpsz+std_strlen(cpsz)-std_strlen(cpszSuffix)
     if cpsz ends with cpszSuffix.  NULL otherwise.

=======================================================================

std_strbegins()

Description:
   The std_strbegins() tests whether a string begins with a particular
   prefix string.

Prototype:

   char *std_strbegins(const char* cpsz, const char* cpszPrefix);

Parameters:
   cpsz: string to test
   cpszPrefix: prefix to test for

Return Value:
   cpsz + std_strlen(cpszPrefix) if cpsz does begin with cpszPrefix,
     NULL otherwise

=======================================================================

std_stribegins()

Description:
   The std_stribegins() tests whether a string begins with a particular
   prefix string, case-folding ASCII characters.

Prototype:

   char *std_stribegins(const char* cpsz, const char* cpszPrefix);

Parameters:
   cpsz: string to test
   cpszPrefix: prefix to test for

Return Value:
   cpsz + std_strlen(cpszPrefix) if cpsz does begin with cpszPrefix,
     NULL otherwise


=======================================================================

std_strcspn()

Description:
   The std_strcspn() function searches s, a NUL-terminated string, for
   the first occurrence of any characters in cpszSrch, a NUL-terminated
   list of characters. This function returns the length of the longest
   initial substring of s which consists of characters not present in
   cpszSrch.

Prototype:

   int std_strcspn(const char* s, const char* cpszSrch);

Parameters:
   s: string to search
   cpszSrch: a list of characters to search for

Return Value:
   The index into the string s of the first occurrence of any of the
   characters in cpszSrch. If no match is found, then index of the
   terminating NUL character is returned.

See Also:
   std_strspn, std_strchr, std_strchrs

=======================================================================

std_strspn()

Description:
   The std_strspn() functions searches s, a NUL-terminated string, for
   the first occurrence of a character that matches none of the
   characters in cpszSrch, a NUL-terminated list of characters. This
   function returns the length of the longest initial substring of s
   which consists of characters present in cpszSrch.

Prototype:

   int std_strspn(const char* s, const char* cpszSrch);

Parameters:
   s: string to search
   cpszSrch: a list of characters to search for

Return Value:
   The index into the string s of the first occurrence of any character
   that matches none of the characters in cpszSrch. If all characters
   in s are present in cpszSrch, the index of the terminating NUL
   character is returned.

See Also:
   std_strcspn, std_strchr, std_strchrs

=======================================================================

std_wstrlcpy()

Description:

   The std_wstrlcpy() function copies a string.  It is equivalent to
   str_strlcpy() except that it operates on wide (16-bit) character strings.
   See std_strlcpy() for details.


Prototype:

   int std_wstrlcpy(AECHAR *pcDest, const AECHAR *pszSrc, int nDestSize);

Parameters:
   pcDst: destination string
   pszSrc: source string
   int nDestSize: size of pcDest __in AECHARs__

Return Value:
   Returns the length of the string (in AECHARs) it tried to create,
   which is same as length of pszSrc.

   Example:

   {
      AECHAR buf[64];
      if (std_wstrlcpy(buf, file_name, STD_ARRAY_SIZE(buf)) >=
          STD_ARRAY_SIZE(buf)) {
         //Truncated -- Handle overflow....
      }
   }

See Also:
   std_wstrlcat

=======================================================================

std_wstrlcat()

Description:

   The std_wstrlcat() function concatenates two strings.  It is equivalent to
   std_strlcat() except that it operates on wide (16-bit) character strings.
   See std_strlcat() for more information.

Prototype:
   int std_wstrlcat(AECHAR *pcDst, const AECHAR *pszSrc, int nDestSize)

Parameters:
   pcDst[out]: Destination string
   pcSrc :     Source string
   nDestSize:  Size of the destination buffer in AECHARs

Return Value:
   Returns the length of the string (in AECHARs) it tried to create,
   which is same as length of pszSrc + the length of pszDest.

   Example:

   {
      char buf[64];
      if (std_wstrlcat(buf, file_name, STD_ARRAY_SIZE(buf)) >=
          STD_ARRAY_SIZE(buf)) {
         //Truncated -- Handle overflow....
      }
   }

See Also:
   std_wstrlcpy

=======================================================================

std_wstrncmp()

Description:

   The std_wstrncmp() function compares up to a specified number of bytes
   in two NUL-terminated strings. It is equivalent to std_strncmp() except
   that it operates on wide (16-bit) character strings.

Prototype:
   int std_wstrncmp(const AECHAR* s1, const AECHAR* s2, int nLen);

Parameters:
   s1, s2: strings to compare
   n: maximum number of AECHARs to compare.  if either s1 or s2 is
      shorter than n, the function terminates there.

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

See Also:
   std_strncmp

=======================================================================

std_wstrcmp()

Description:
   The std_wstrcmp() compares two NUL-terminated strings. It is equivalent
   to std_strncmp() except that it operates on wide (16-bit) character
   strings. Comparison is strictly by byte values with no character set
   interpretation.

Prototype:

   int std_wstrcmp(const AECHAR* s1, const AECHAR* s2);

Parameters:
   s1, s2: strings to compare

Return Value:
   0 if strings are the same ~
   < 0 if s1 is less than s2 ~
   > 0 if s1 is greater than s2

See Also:
   std_strcmp

=======================================================================

std_wstrchr()

Description:
   This function is the wide string counterpart of std_strchr().
   The std_wstrchr() finds the first occurrence of a character in a
   NUL-terminated wide (16-bit) character string.

Prototype:

   AECHAR* std_wstrchr(const AECHAR* s, AECHAR ch);

Parameters:
   s: string to search
   ch: char to search for

Return Value:
   pointer to first occurrence, NULL if not found

See Also:
   std_strchr

=======================================================================

std_wstrrchr()

Description:
   This function is the wide string counterpart of std_strrchr().
   The std_wstrrchr() finds the last occurrence of a character in a
   NUL-terminated wide (16-bit) character string.

Prototype:

   AECHAR* std_wstrrchr(const AECHAR* s, AECHAR ch);

Parameters:
   s: string to search
   ch: char to search for

Return Value:
   pointer to last occurrence, NULL if not found

See Also:
   std_strrchr

=======================================================================

std_makepath()

Description:
   The std_makepath() constructs a path from a directory portion and a file
   portion, using forward slashes, adding necessary slashes and deleting extra
    slashes.  This function guarantees NUL-termination of pszDest

Prototype:

   int std_makepath(const char *cpszDir, const char *cpszFile,
                    char *pszDest, int nDestSize)

Parameters:
   cpszDir: directory part
   cpszFile: file part
   pszDest: output buffer
   nDestSize: size of output buffer in bytes

Return Value:
   the required length to construct the path, not including
   NUL-termination

Comments:
   The following list of examples shows the strings returned by
   std_makepath() for different paths.

Example:

   cpszDir  cpszFile  std_makepath()
   ""        ""           ""
   ""        "/"          ""
   "/"       ""           "/"
   "/"       "/"          "/"
   "/"       "f"          "/f"
   "/"       "/f"         "/f"
   "d"       "f"          "d/f"
   "d/"      "f"          "d/f"
   "d"       "/f"         "d/f"
   "d/"      "/f"         "d/f"

See Also:
   std_splitpath

=======================================================================

std_splitpath()

Description:
   The std_splitpath() finds the filename part of a path given an inclusive
   directory, tests for cpszPath being in cpszDir. The forward slashes are
   used as directory delimiters.

Prototype:

   char *std_splitpath(const char *cpszPath, const char *cpszDir);

Parameters:
   cpszPath: path to test for inclusion
   cpszDir: directory that cpszPath might be in

Return Value:
   the part of cpszPath that actually falls beneath cpszDir, NULL if
   cpszPath is not under cpszDir

Comments:
   The std_splitpath() is similar to std_strbegins(), but it ignores trailing
   slashes on cpszDir, and it returns a pointer to the first character of
   the subpath.

   The return value of std_splitpath() will never begin with a '/'.

   The following list of examples shows the strings returned by
   std_splitpath() for different paths.

Example:
   cpszPath cpszDir  std_splitpath()
   ""        ""           ""
   ""        "/"          ""
   "/"       ""           ""
   "/"       "/"          ""
   "/d"      "d"          null
   "/d"      "/"          "d"
   "/d/"     "/d"         ""
   "/d/f"    "/"          "d/f"
   "/d/f"    "/d"         "f"
   "/d/f"    "/d/"        "f"

See Also:
   std_makepath

=======================================================================

std_cleanpath()

Description:
   The std_cleanpath() removes double slashes, ".", and ".." from
   slash-delimited paths,. It operates in-place.

Prototype:

   char *std_cleanpath(char *pszPath);

Parameters:
   pszPath[in/out]: path to "clean"

Return Value:
   pszPath

Comments:
   Passing an "fs:/" path to this function may produce undesirable
   results.  This function assumes '/' is the root.

Examples:
       pszPath  std_cleanpath()
         "",           "",
         "/",          "/",

         // here"s, mostly alone
         "./",         "/",
         "/.",         "/",
         "/./",        "/",

         // "up"s, mostly alone
         "..",         "",
         "/..",        "/",
         "../",        "/",
         "/../",       "/",

         // fun with x
         "x/.",        "x",
         "x/./",       "x/",
         "x/..",       "",
         "/x/..",      "/",
         "x/../",      "/",
         "/x/../",     "/",
         "/x/../..",   "/",
         "x/../..",    "",
         "x/../../",   "/",
         "x/./../",    "/",
         "x/././",     "x/",
         "x/.././",    "/",
         "x/../.",     "",
         "x/./..",     "",
         "../x",       "/x",
         "../../x",    "/x",
         "/../x",      "/x",
         "./../x",     "/x",

         // double slashes
         "//",         "/",
         "///",        "/",
         "////",       "/",
         "x//x",       "x/x",


Side Effects:
   None

See Also:
   None


=======================================================================

std_basename()

Description:
   The std_basename() returns the filename part of a string,
   assuming '/' delimited filenames.

Prototype:

   char *std_basename(const char *cpszPath);

Parameters:
   cpszPath: path of interest

Return Value:
   pointer into cpszPath that denotes part of the string immediately
   following the last '/'

Examples:
     cpszPath       std_basename()
         ""            ""
         "/"           ""
         "x"           "x"
         "/x"          "x"
         "y/x"         "x"
         "/y/x"        "x"

 See Also:
    None

=======================================================================

std_rand_next()

Description:
  The std_rand_next() generates pseudo-random bytes.

Prototype:

   unsigned std_rand_next(unsigned uRand);

Parameters:
   uRand: a seed for the pseudo-random generator

Return Value:
   the next value in the generator from uRand

Comments:
   for best results, this function should be called with its last
   generated output.

   This is an example of code to generate 256 bytes of pseudo-random data.

   This is not crypto quality and should not be used for key generation
   and the like.

Example:
   {
       unsigned rand_buf[256/sizeof(unsigned)];
       int      i;
       unsigned uLast = std_rand_next(uCurrentTime);
       for (i = 0; i < STD_ARRAY_SIZE(rand_buf); i++) {
          rand_buf[i] = (uLast = std_rand_next(uLast));
       }
   }

See Also:
   std_rand()

=======================================================================

std_rand()

Description:
  The std_rand() functions generates pseudo-random bytes and places it
  in an output buffer of specified size.

Prototype:

   uint32 std_rand(uint32 uSeed, byte* pDest, int nSize);

Parameters:
   uSeed: A seed for the pseudo-random generator
   pDest: The output buffer where the random bytes are placed.
   nSize: The size in bytes of pDest.

Return Value:
   The new seed value that can be used in a subsequent call to
   std_rand().

Comments:

   std_rand() is a linear congruent psuedo-random number generator that
   is seeded using the input seed. This makes the ouput predictable if
   you can determine (or influence) the seed value used. Furthermore,
   the random sequence of bytes generated by two different calls to this
   function will be identical if both the calls use the same seed value.

   This is not crypto quality and should not be used for key generation
   and other cryptographic uses.

See Also:
   std_rand_next()

=======================================================================

std_CopyLE()

Description:

   The std_CopyLE() function copies data while translating numeric values
   between host byte ordering and "little endian" byte ordering.

   pvDest and pvSrc are NOT required to be 16 or 32-bit word aligned.

   Behavior is undefined when the destination and source arrays overlap,
   except in the special case where pvDest and pvSrc are equal.  In that case,
   std_CopyLE() modifies the buffer in-place.

   When the target byte ordering (little endian) matches the host byte
   ordering, in-place translations reduce to a no-op, and copies are
   delegated directly to std_memmove().


Prototype:
   int std_CopyLE(void *pvDest, int nDestSize,
                  const void *pvSrc,  int nSrcSize,
                  const char *pszFields);

Parameters:
   pvDest:    Pointer to destination buffer.
   nDestSize: Size of the destination buffer.
   pvSrc:     Pointer to buffer containing source data.
   nSrcSize:  Size of source data.
   pszFields: Description of the fields that comprise the source data.

              Each field size is given by a positive decimal integer or one of
              the following characters: "S", "L", "Q", or "*".  The letters
              denote fields that should be converted to the desired byte
              ordering:

===pre>
                S : a 2 byte (16 bit) value.
                L : a 4 byte (32 bit) value.
                Q : a 8 byte (64 bit) value.
===/pre>

              An integer gives a number of bytes and "*" represents the
              remainder of the pvSrc[] buffer.  No reordering is performed on
              data in these fields.

              Comparisons are case-sensitive.  Behavior is undefined when
              other characters are supplied in pszFields.

              For example: "L12S*" would be appropriate to copy a structure
              containing a uint32 followed by a 12 byte character array,
              followed by a uint16, followed by an arbitrary amount of
              character data.

              If nSrcSize is greater than the structure size (total of all the
              sizes in pszFields[]) then pvSrc[] is treated as an array of
              structures, each of which is described by pszFields.

Return Value:

   The number of bytes actually copied or translated in-place.  This will be
   the smaller of nDestSize and nSrcSize, or zero if one of them are negative.


=======================================================================

std_CopyBE()

Description:

   The std_CopyBE() function has the same semantics as std_CopyLE() except it
   copies between host byte ordering and big-endian ("network") byte order.

   See std_CopyLE() for more details.


Prototype:
   void *std_CopyBE(void *pvDest, const void *pvSrc,
                           int cbDest, int nItems, const char *pszFields);

Parameters:
   pvDest:    Pointer to destination buffer.
   nDestSize: Size of the destination buffer.
   pvSrc:     Pointer to buffer containing source data.
   nSrcSize:  Size of source data.
   pszFields: Description of the fields that comprise the source data,
              as defined in std_CopyLE.

Return Value:

   The number of bytes actually copied or translated in-place.  This will be
   the smaller of nDestSize and nSrcSize, or zero if one of them are negative.

=======================================================================

std_swapl()

Description:
   The std_swapl() changes endianness of an unsigned long.

Prototype:

   unsigned long std_swapl(unsigned long ul)

Parameters:
   ul: input unsigned long

Return Value:
   ul, reversed in byte-ordering

=======================================================================

std_swaps()

Description:
   The std_swaps() changes endianness of an unsigned short.

Prototype:

   unsigned short std_swaps(unsigned short us)

Parameters:
   us: input unsigned short

Return Value:
   us, reversed in byte-ordering

=======================================================================

std_letohs()

Description:
   The std_letohs() changes a short from little-endian to host byte order.

Prototype:

   unsigned short std_letohs(unsigned short us)

Parameters:
   us: short to convert

Return Value:
   us converted from little-endian to host byte order.  If the
     host is little endian, just returns us

=======================================================================

std_htoles()

Description:
   The std_htoles() converts a short from host byte-order to little-endian.

Prototype:

   unsigned short std_htoles(unsigned short us)

Parameters:
   us: short to convert

Return Value:
   us converted from host byte order to little-endian.  If the
   host is little endian, just returns us

=======================================================================

std_letohl()

Description:
   The std_letohl() changes a long from little-endian to host byte order.

Prototype:

   unsigned long std_letohl(unsigned long ul)

Parameters:
   ul: long to convert

Return Value:
   ul converted from little-endian to host byte order.  If the
   host is little endian, just returns ul

=======================================================================

std_htolel()

Description:
   The std_htolel() converts a long from host byte-order to little-endian.

Prototype:

   unsigned long std_htolel(unsigned long ul)

Parameters:
   ul: long to convert

Return Value:
   ul converted from host byte order to little-endian.  If the
   host is little endian, just returns ul.


=======================================================================

std_ntohs()

Description:
   The std_ntohs() changes a short from big-endian to host byte order.

Prototype:

   unsigned short std_ntohs(unsigned short us)

Parameters:
   us: short to convert

Return Value:
   us converted from big-endian to host byte order.  If the
   host is big endian, just returns us.

=======================================================================

std_htons()

Description:
   The std_htons() converts a short from host byte-order to big-endian.

Prototype:

   unsigned short std_htons(unsigned short us)

Parameters:
   us: short to convert

Return Value:
   us converted from host byte order to big-endian.  If the
   host is big endian, just returns us.

=======================================================================

std_ntohl()

Description:
   The std_ntohl() changes a long from big-endian to host byte order.

Prototype:

   unsigned long std_ntohl(unsigned long ul)

Parameters:
   ul: long to convert

Return Value:
   ul converted from big-endian to host byte order.  If the
   host is big endian, just returns ul.

=======================================================================

std_htonl()

Description:
   The std_htonl() converts a long from host byte-order to big-endian.

Prototype:

   unsigned long std_htonl(unsigned long ul)

Parameters:
   ul: long to convert

Return Value:
   ul converted from host byte order to big-endian.  If the
   host is big endian, just returns ul.


=======================================================================

std_strlprintf()

Description:

   The functions std_strlprintf() and std_vstrlprintf() write formatted
   output to a string.  These functions guarantee NUL-termination of
   the output buffer when its size is greater than zero.

   A format string is copied to the output buffer, except for conversion
   specifiers contained within the format string.  Conversion specifiers
   begin with a "%" and specify some action that consumes an argument from
   the argument list.

   Conversion specifiers have the following form:
===pre>
       %[FLAGS] [WIDTH] [.PRECISION] [TYPE] CONV
===/pre>

   CONV is the only required field.  It is always a single character,
   and determines the action to be taken.  Supported values are:

===pre>
    CONV | Description
   ======|=======================================================
     c   | Output a single character.
         |
     s   | Output a NUL-terminated single-byte character string.
         |
    d, i | Ouptut a signed decimal integer.
         |
     u   | Output an unsigned decimal integer.
         |
     o   | Output an unsigned octal integer.
         |
     x   | Output an unsigned hexadecimal integer, using
         | lower case digits.
         |
     X   | Output an unsigned hexadecimal integer, using
         | upper case digits.
         |
     p   | Output a pointer value as eight hexadecimal digits,
         | using upper case digits.
===/pre>

   The next argument from the argument list supplies the value to be
   formatted and output.

   FLAGS, WIDTH, and PRECISION can modify the formatting of the value.

   FLAGS consists of one or more of the following characters:

===pre>
   Flag | Meaning
   =====|=================================================================
     +  | Prefix positive numbers with "+" (%d and %i only).
   -----|-----------------------------------------------------------------
     -  | When padding to meet WIDTH, pad on the right.
   -----|-----------------------------------------------------------------
     0  | Pad with '0' characters when padding on the left to meet WIDTH.
   -----|-----------------------------------------------------------------
   blank| Prefix positive numbers with " " (%d and %i only).
   space|
   -----|-----------------------------------------------------------------
     #  | With %x or %X: prefixes non-zero values with "0x"/"0X".
        | With %o, ensure the value begins with "0" (increasing PRECISION
        |    if necessary).
        | Ignored for all other CONV specifiers.
   -----|-----------------------------------------------------------------
===/pre>

   WIDTH is an unsigned decimal integer or the character "*".

   WIDTH gives the minimum number of characters to be written.  The
   formatted value will be padded with spaces until the minimum size is
   met; it never causes a value to be truncated The sign of the WIDTH
   integer selects between left and right padding.  Padding will be on
   the left unless the "-" flag is specified.

   When "*" is used, an 'int' argument is consumed from the argument
   list and used as the WIDTH.  A negative argument specifies padding on
   the right, and its absolute value gives the amount of padding.

   If the "0" flags is specified, any padding on the left will consist
   of "0" characters.  An exception to this rule is that the "0" flag is
   ignored when precision is specified for a numeric value.

   PRECISION is a non-negative decimal integer or "*" preceded by ".".

   When PRECISION accompanies any of the numeric conversions, it
   specifies the minimum number of digits to output.  Values are padded
   on the left with '0' to meet the specified size.  PRECISION defaults
   to 1 for numbers.

   When PRECISION accompanies other conversions, it specifies the
   maximum number of characters from the value to output.  The value
   will be truncated to ensure that at most PRECISION characters are
   output.

   TYPE provides information about the type of arguments.  This is used
   to determine the size of integer arguments. Values larger than 'int'
   can be properly obtained from the argument list.  Their behavior
   should be considered undefined for CONV operations other than integer
   formatting.

===pre>
    TYPE  | Meaning
   =======|=====================
     hh   | sizeof(char)
   -------|---------------------
      h   | sizeof(short)
   -------|---------------------
      l   | sizeof(long)
   -------|---------------------
    L, ll | sizeof(long long)
   -------|---------------------
      j   | sizeof(int64)
   -------|---------------------
      z   | sizeof(size_t)
   -------|---------------------
===/pre>

   For 64-bit integers, "ll" may be the most widely-supported type
   specifier in other printf implementation, but "j" has been introduced
   in ISO C99. This implementation supports both.

   Note that arguments to variadic functions are promoted to 'int' when
   smaller than 'int', so 'h' and 'hh' have no observable effect.
   Static analysis tools that understand standard format string syntax
   may use this information for other purposes.

Prototype:

   int std_strlprintf(char *pszDest, int nDestSize,
                      const char *pszFmt, ...);
Parameters:
   pszDest [out]: output buffer, where output will be placed
   nDestSize:     size of pszDest in bytes
   pszFmt:        format string

Return Value:

   The size required to hold the entire untruncated output, NOT
   including NUL-termination.

Comments:

   Notable omissions from std_strlprintf() are lack of support for
   floating point and lack of support for "%n".

Side Effects:
   None

See Also:
   None

=======================================================================

std_vstrlprintf()

Description:

  The std_vstrlprintf() is documented with std_strlprintf(), it's the
  vector form of std_strlprintf().  See std_strlprintf() for a
  more complete description.

Prototype:
   int std_vstrlprintf(char *pszDest, int nDestSize,
                       const char *pszFmt, AEEVaList args);

Parameters:
   pszDest [out]: output buffer, where output will be placed
   nDestSize:     size of pszDest in bytes
   pszFmt:        format string
   args:          arguments


=======================================================================

std_snprintf()

Description:

   The functions std_snprintf() and std_vsnprintf() are similar to
   std_strlprintf and std_vstrlprintf that write formatted output to a
   string. Unlike std_strlprintf, std_snprintf also support the floating
   point conversion specifiers. These functions guarantee NUL-termination
   of the output buffer when its size is greater than zero.

   A format string is copied to the output buffer, except for conversion
   specifiers contained within the format string.  Conversion specifiers
   begin with a "%" and specify some action that consumes an argument from
   the argument list.

   Conversion specifiers have the following form:
===pre>
       %[FLAGS] [WIDTH] [.PRECISION] [TYPE] CONV
===/pre>

   CONV is the only required field.  It is always a single character,
   and determines the action to be taken.  For a detailed description of
   conversion sepcifiers, please refer to the documentation of
   std_strlprintf(). Here. we only provide description of these fields
   as it applies to the additional CONV values supported by
   std_snprintf().

   In addition to the values for CONV supported by std_strlprintf, this
   function supports the following values:

===pre>
    CONV | Description
   ======|=======================================================
    e, E | Outputs a double value representing a floating point
         | number in the style [-]d.ddd e±dd, where there is one
         | digit (which is nonzero if the argument is nonzero)
         | before the decimal-point character and the number of
         | digits after it is equal to the precision. If the
         | precision is missing, it is taken as 6. If the precision
         | is zero and the # flag is not specified, no decimal-point
         | character appears. The value is rounded to the appropriate
         | number of digits. The E conversion specifier produces a
         | number with E instead of e introducing the exponent. The
         | exponent always contains at least two digits, and only as
         | many more digits as necessary to represent the exponent.
         | If the value is zero, the exponent is zero.
         |
    f, F | Outputs a double value representing a floating point
         | number in the style [-]ddd.ddd, where the number of
         | digits after the decimal-point character is equal to the
         | precision specification. If the precision is missing, it
         | is taken as 6. If the precision is zero and the # flag is
         | not specified, no decimal-point character appears. If a
         | decimal-point character appears, at least one digit
         | appears before it. The value is rounded to the appropriate
         | number of digits.
         |
    g, G | Outputs a double value representing a floating point
         | number in the style f or e (or in style F or E in the case
         | of a G conversion specifier), with the precision specifying
         | the number of significant digits. If the precision is zero,
         | it is taken as 1. The style used depends on the value
         | converted. Style e (or E) is used only if the exponent
         | resulting from such a conversion is less than -4 or greater
         | than or equal to the precision. Trailing zeros are removed
         | from the fractional portion of the result unless the # flag
         | is specified; a decimal-point character appears only if it
         | is followed by a digit.
         |
    a, A | Outputs a double value representing a floating point
         | number in the style [-]0xh.hhhh p±d, where there is one
         | non-zero hexadecimal digit before the decimal-point
         | character and the number of hexadecimal digits after it is
         | equal to the precision. If the precision is missing then
         | the precision is assumed to be sufficient for an exact
         | representation of the value, except that trailing zeros
         | may be omitted. If the precision is zero and the # flag is
         | not specified, no decimal point character appears. The
         | letters 'abcdef' are used for '%a' conversion and the
         | letters ABCDEF for '%A' conversion. The '%A' conversion
         | specifier produces a number with 'X' and 'P' instead of 'x'
         | and 'p'. The exponent always contains at least one digit,
         | and only as many more digits as necessary to represent the
         | decimal exponent of 2. If the value is zero, the exponent
         | is zero.
         |
===/pre>

   For 'e', 'f', 'g' and 'a' convervsion specifiers, a double argument
   representing an infinity is converted in to the style '[-]inf' and
   a double argument representing a NaN is converted in to the stlye
   'nan'. The 'E', 'F', 'G' and 'A' conversion specifiers result in
   'INF' or 'NAN' instead of 'inf' or 'nan', respectively.

Prototype:

   int std_snprintf(char *pszDest, int nDestSize,
                    const char *pszFmt, ...);
Parameters:
   pszDest [out]: output buffer, where output will be placed
   nDestSize:     size of pszDest in bytes
   pszFmt:        format string

Return Value:

   The size required to hold the entire untruncated output, NOT
   including NUL-termination.

Comments:

   Notable omissions from std_strlprintf() lack of support for "%n".

Side Effects:
   None

See Also:
   std_strlprintf()

=======================================================================

std_vsnprintf()

Description:

  The std_vsnprintf() is documented with std_snprintf(), it's the
  vector form of std_snprintf(). See std_snprintf() for a more complete
  description.

Prototype:
   int std_vsnprintf(char *pszDest, int nDestSize,
                     const char *pszFmt, AEEVaList args);

Parameters:
   pszDest [out]: output buffer, where output will be placed
   nDestSize:     size of pszDest in bytes
   pszFmt:        format string
   args:          arguments


=======================================================================

std_scanul()

Description:

    The std_scanul() converts an ASCII representation of a number to an unsigned
    long.  It expects strings that match the following pattern:
===pre>
         spaces [+|-] digits
===/pre>

    'Spaces' is zero or more ASCII space or tab characters.

    'Digits' is any number of digits valid in the radix.  Letters 'A' through
    'Z' are treated as digits with values 10 through 35.  'Digits' may begin
    with "0x" when a radix of 0 or 16 is specified.

    Upper and lower case letters can be used interchangeably.


Prototype:

    uint32 std_scanul( const char *pchBuf, int nRadix, const char **ppchEnd,
                       int *pnError)

Parameters:

    pchBuf [in] : the start of the string to scan.

    nRadix [in] : the numeric radix (or base) of the number.  Valid values are
                  2 through 36 or zero, which implies auto-detection.
                  Auto-detection examines the digits field.  If it begins with
                  "0x", radix 16 is selected.  Otherwise, if it begins with
                  "0" radix 8 is selected.  Otherwise, radix 10 is selected.

    ppchEnd [out] : if ppchEnd is not NULL, *ppchEnd points to the first
                    character that did not match the expected pattern shown
                    above, except on STD_BADPARAM and STD_OVERFLOW when it is
                    set to the start of the string.

    pnError [out] : If pnError is not NULL, *pnError holds the error code,
                    which is one of the following:
~
        0 : Numeric value is from 0 to MAX_UINT32.

        STD_NEGATIVE : The scanned value was negative and its absolute value was
                       from 1 to MAX_UINT32.  The result is the negated value
                       (cast to a uint32).

        STD_NODIGITS : No digits were found.  The result is zero.

        STD_OVERFLOW : The absolute value exceeded MAX_UINT32.  The result
                       is set to MAX_UINT32 and *ppchEnd is set to pchBuf.

        STD_BADPARAM : An improper value for nRadix was received.  The result
                       is set to zero, and *ppchEnd is set to pchBuf.
*

Return Value:

    The converted numeric result.

Comments:

   The std_scanul() is similar to ANSI C's strtoul() but differs in the following
   respects:

     1. It returns an error/success code.  strtoul() results are ambiguous
        unless the caller sets errno to zero before calling it.

     2. std_scanul() is free of references to current locale and errno.  Some
        strtoul() implementations use locale; some don't.

     3. It provides more complete reporting of range underflow.  strtoul()
        does not distinguish between "-1" and "0xFFFFFFFF", and underflow is
        poorly defined.

     4. std_scanul() reports a "no digits" error code to distinguish "0" from
        whitespace, "+", etc..

See Also:

   std_scanull()

=======================================================================

std_scanull()

Description:

    The std_scanull() converts an ASCII representation of a number to an
    unsigned long long.  It expects strings that match the following pattern:
===pre>
         spaces [+|-] digits
===/pre>

    'Spaces' is zero or more ASCII space or tab characters.

    'Digits' is any number of digits valid in the radix.  Letters 'A' through
    'Z' are treated as digits with values 10 through 35.  'Digits' may begin
    with "0x" when a radix of 0 or 16 is specified.

    Upper and lower case letters can be used interchangeably.


Prototype:

    uint64 std_scanull(const char *pchBuf, int nRadix, const char **ppchEnd,
                       int *pnError)

Parameters:

    pchBuf [in] : the start of the string to scan.

    nRadix [in] : the numeric radix (or base) of the number.  Valid values are
                  2 through 36 or zero, which implies auto-detection.
                  Auto-detection examines the digits field.  If it begins with
                  "0x", radix 16 is selected.  Otherwise, if it begins with
                  "0" radix 8 is selected.  Otherwise, radix 10 is selected.

    ppchEnd [out] : if ppchEnd is not NULL, *ppchEnd points to the first
                    character that did not match the expected pattern shown
                    above, except on STD_BADPARAM and STD_OVERFLOW when it is
                    set to the start of the string.

    pnError [out] : If pnError is not NULL, *pnError holds the error code,
                    which is one of the following:
~
        0 : Numeric value is from 0 to MAX_UINT64.

        STD_NEGATIVE : The scanned value was negative and its absolute value was
                       from 1 to MAX_UINT64.  The result is the negated value
                       (cast to a uint64).

        STD_NODIGITS : No digits were found.  The result is zero.

        STD_OVERFLOW : The absolute value exceeded MAX_UINT64.  The result
                       is set to MAX_UINT64 and *ppchEnd is set to pchBuf.

        STD_BADPARAM : An improper value for nRadix was received.  The result
                       is set to zero, and *ppchEnd is set to pchBuf.
*

Return Value:

    The converted numeric result.

Comments:

   The std_scanull() is similar to ANSI C's strtoull() but differs in the following
   respects:

     1. It returns an error/success code.  strtoull() results are ambiguous
        unless the caller sets errno to zero before calling it.

     2. std_scanull() is free of references to current locale and errno.  Some
        strtoull() implementations use locale; some don't.

     3. It provides more complete reporting of range underflow.  strtoul()
        does not distinguish between "-1" and "0xFFFFFFFFFFFFFFFF", and underflow
        is poorly defined.

     4. std_scanull() reports a "no digits" error code to distinguish "0" from
        whitespace, "+", etc..

See Also:

   std_scanul()

=======================================================================

std_qsort()

Description:

  An implementation of the quicksort algorithm, a massively recursive,
  in-place sorting algorithm for an array.

  The contents of the array are sorted in ascending order according to
  the comparison function pointed to by pfnCompare.

  pfnCompare must return a value less than, equal to, or
  greater than zero if the first argument is considered to be
  less than, equal to, or greater than the second, respectively.

  std_qsort() is not a stable sort.

Prototype:
   void std_qsort(void* pElems, int nNumElems, int nElemWidth,
                  int (*pfnCompare)(void*, const void*, const void*),
                  void* pCompareCx);


Parameters:
   pElems: array of elements to be sorted in place.  It's size
           must be nNumElems * nElemWidth in bytes.
   nNumElems: number of elements in pElems
   nElemWidth: the width, in bytes of each element of pElems
   pfnCompare: callback comparison function, should return 0, less than
               zero or greater than zero if the left comparand is equal to, less
               than, or greater than, the right comparand, respectively.
   pCompareCx: the context passed as the first parameter by pfnCompare

Return Value:
   None

Comments:
   If nElemWidth is 2, 4, or 8, pElems is accessed internally as
   integer values for the purposes of reading and writing elements.
   Therefore, pElems must be aligned on a memory boundary compatible
   with integer access of the array elements.  I.e. if you pass 4 as
   nElemWidth, *(int*)pElems must succeed.

=======================================================================

std_bisect()

Description:

   Find an element in a sorted array of elements.  Uses a binary
   search.

Prototype:
   int std_bisect(const void* pElems, int nNumElems, int nElemWidth,
                  const void* pElemFind,
                  int (*pfnCompare)(void*, const void*, const void*),
                  void* pCompareCx);

Parameters:
   pElems: array of elements to be searched.  It's size
           must be nNumElems * nElemWidth in bytes.
   nNumElems: number of elements in pElems
   nElemWidth: the width, in bytes of each element of pElems
   pElemFind: the element value to find in the array
   pfnCompare: callback comparison function, should return 0, less than
               zero or greater than zero if the left comparand is equal to, less
               than, or greater than, the right comparand, respectively.
   pCompareCx: the context passed as the first parameter by pfnCompare

Return Value:
   index of the element such that pElems[index] <= elem < pElems[index + 1]
   nNumElems if elem is greater than all the elements in the list
   0 if the elem is less than or equal to the all the elements in the list

=======================================================================

std_merge()

Description:

   Merge two sorted arrays into another array.

Prototype:
   void std_merge(void* vpDst, int nDst,
                  const void* vpA, int nA,
                  const void* vpB, int nB,
                  int nElemWidth,
                  int (*pfnCompare)(void*, const void*, const void*),
                  void* pCompareCx);

Parameters:
   vpDst: destination array.  It's size must be nDst * nElemWidth in bytes.
   nDst: number of elements that vpDst can accomodate
   vpA: array of elements to be merged,  it's size must be nA * nElemWidth
         in bytes.
   nA: number of elements in vpA
   vpB: array of elements to be merged,  it's size must be nB * nElemWidth
         in bytes.
   nB: number of elements in vpB
   nElemWidth: the width, in bytes of each element of pElems
   pfnCompare: callback comparison function, should return 0, less than
               zero or greater than zero if the left comparand is equal to, less
               than, or greater than, the right comparand, respectively.
   pCompareCx: the context passed as the first parameter by pfnCompare

Return Value:
  none

=======================================================================

std_uniq()

Description:
   Removes duplicate entries from a sorted array.

Prototype:
   int std_uniq(void* vpElems, int nNumElems, int nElemWidth,
                int (*pfnCompare)(void*, const void*, const void*),
                void* pCompareCx);

Parameters:
   pElems: array of elements to be searched.  It's size
           must be nNumElems * nElemWidth in bytes.
   nNumElems: number of elements in pElems
   nElemWidth: the width, in bytes of each element of pElems
   pfnCompare: callback comparison function, should return 0, less than
               zero or greater than zero if the left comparand is equal to, less
               than, or greater than, the right comparand, respectively.
   pCompareCx: the context passed as the first parameter by pfnCompare

Return Value:
   the number of uniq elements left in vpElems

=======================================================================

std_scand()

Description:

   The std_scand() converts the initial portion of an input ASCII string
   to it's corresponding floating point value. It expects the input
   string to match the following pattern:
===pre>
         <Spaces><Subject String><Rest Of The String>
===/pre>

   'Spaces' - is zero or more ASCII space or tab characters.
   'Subject String' - is the part of the input string that represents a
                      valid floating point constant.
   'Rest Of The String' - is the remaining sequence of one or more
                          characters including the terminating null
                          character of the input string.

   A valid subject string can be one of the following:
      -- <NAN>, ignoring case. This is interpreted as a quiet NAN.
      -- [+|-]<INF|INFINITY>, ignoring case. This is interpreted as an
         infinity.
      -- [+|-]<Valid Floating Point Number>

   In general, a valid floating poing number can either be a decimal
   number or an hexadecimal number, and has the following form:
      <Integral Part>[.[<Fractional Part>]][<Exponent>]
   where the intergral, fractional and the exponent part may consist of
   sequence of valid decimal or hexadecimal digits. More specifically:

   For a decimal floating point number:
      'Integral Part' - <Decimal Digits>
      'Fractional Part' - <Decimal Digits>
      'Exponent' - <e|E><Decimal Digits>
   For a hexadecimal floating point number:
      'Integral Part' - <Hexadecimal Digits>
      'Fractional Part' - <Hexadecimal Digits>
      'Exponent' - <p|P><Decimal Digits>

   where:
      'Decimal Digits' - is any number of digits in the range [0,10].
      'Hexadecimal Digits' - is any number of digits in the range [0,10]
                             or the alphabets A through F.
      'e','E','p','P' - represent the exponent characters

Prototype:

    double std_scand(const char *pchBuf, const char **ppchEnd);

Parameters:

    pchBuf [in] : the start of the string to scan.

    ppchEnd [out] : if ppchEnd is not NULL, *ppchEnd points to the first
                    character after the parsed number.

Return Value:

    This function returns the converted numeric result. If the string
    does not contain a valid floating point number then the function
    returns zero. If the converted value is outside the range of
    representable values (overflow), [-]INFINITY is
    returned. In case of an underflow, the function returns zero.

=======================================================================*/

#endif // AEESTD_H


