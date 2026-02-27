/*
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * File: $Header: //depot/software/projects/feature_branches/gen5_phase1/os/linux/classic/ap/apps/include/aniAsfDict.h#3 $
 * Contains declarations for utilities that read a dictionary of key
 * value pairs from a text file. The dictionary can be queried to
 * return values as strings or as integers.
 * Author:      Mayank D. Upadhyay
 * Date:        03-July-2002
 * History:-
 * Date         Modified by     Modification Information
 * ------------------------------------------------------
 *
 */
#ifndef _ANI_ASF_DICT_H_
#define _ANI_ASF_DICT_H_

#include <sys/types.h>
#include <aniTypes.h>
#include <aniAsfHashTable.h>

#define ANI_MAX_DICT_LINE_SIZE 128

/**
 * An opaque data type that encapsulated a number of key-value
 * pairs. The dictionary can be queried with key strings to return the
 * values as strings or as integers.
 */
typedef struct tAniDict tAniDict;

/**
 * aniAsfDictCreate
 *
 * FUNCTION:
 * Creates a dictionary by reading key-value pairs from the file
 * specified by the given path. The file should contain ASCII
 * key-value pairs of the form "key=value", one per line. Any line
 * that begins with a "#" is ignored. Also, the lines should be no
 * larger than ANI_MAX_DICT_LINE_SIZE bytes including the "=" character,
 * but not the newline character.
 *
 * @param dictP a pointer that will be set to the newly allocated
 * dictionary instance
 * @param fileName the absolute or relative path to the file that
 * contains the dictionary key-value pairs.
 *
 * @return ANI_OK if the operation succeeds; ANI_E_MALLOC_FAILED if
 * memory cannot be allocated; ANI_E_ILLEGAL_ARG if the file cannot be
 * opened.
 */
int
aniAsfDictCreate(tAniDict **dictP, char *fileName);

/**
 * aniAsfDictFree
 *
 * FUNCTION:
 * Frees a previously allocated dictionary and all the key value pairs
 * contained within. 
 *
 * Note: Applications that retain pointers to string values returned
 * by the dictionary should make local copies of those values before
 * freeing the dictionary.
 *
 * @param dict the dictionary to free
 * @return ANI_OK if the operation succeeds
 * @see aniAsfDictCreate
 */
int
aniAsfDictFree(tAniDict *dict);

/**
 * aniAsfDictGetString
 *
 * FUNCTION:
 * Returns the string value stored for the given key. Any leading or
 * trailing white space around the string is trimmed off. e.g.,
 *   myKey=some Value 
 *   myKey = some Value
 * will both return the string "some Value" (without the
 * quotes and without the leading and trailing space).
 *
 * Note: This operation looks up the key within a table maintained by
 * the dictionary. Thus, a dictionary should not be used as storage
 * for values that are frequently looked up. Instead, such values
 * should be cached elsewhere once they have been obtained from the
 * dictionary.
 * 
 * @param dict the dictionary to query
 * @param key the key to lookup
 * @param valueP a pointer that is set to the result of the lookup if
 * successful. The memory for the value is internal to the dictionary
 * and will be freed when the dictionary is freed. Note that this is
 * different from the way aniAsfDictGetInt behaves. If the key cannot be
 * found, this pointer is set to the parameter defaultStr
 * @param defaultStr the default string to return if the key cannot be
 * found. This parameter may be NULL in which case the operation will
 * return ANI_E_FAILED if the key is not found.
 * 
 * @return a positive number indicating the length of the string that
 * is returned via valueP; or ANI_E_FAILED if the key is not found and
 * defaultStr is NULL.
 * 
 * @see aniAsfDictGetInt
 */
int
aniAsfDictGetString(tAniDict *dict, 
                 const u_char *key, 
                 u_char **valueP, 
                 u_char *defaultStr);

/**
 * aniAsfDictGetDelimitedString
 *
 * FUNCTION:
 * Returns the string value stored for the given key. The string is
 * expected to be enclosed within double quotes. This is useful for
 * retrieving values that can contain leading or trailing
 * white-space which the caller wants to retrieve. (The function
 * aniAsfDictGetString trims off leading and trailing white space
 * before the caller receives the value.) 
 *
 * e.g., the key-value pairs
 *     myPsk=" This is a random string! "
 *     myPsk = " This is a random string! "
 * both return the string " This is a random string! " (without the
 * quotes but with the leading and trailing space).
 *
 * ONCE THE QUOTES HAVE BEEN REMOVED FROM THE STRING, THE SAME KEY
 * CANNOT BE RETRIEVED FROM THE DICTIONARY. IF YOU WISH TO DO THAT
 * AGAIN, PLEASE CALL aniAsfDictSetDelimitedString(...) TO REPAIR THE
 * STRING PRIOR TO YOUR NEXT CALL TO
 * aniAsfDictGetDelimitedString(...).
 * 
 * Note: This operation looks up the key within a table maintained by
 * the dictionary. Thus, a dictionary should not be used as storage
 * for values that are frequently looked up. Instead, such values
 * should be cached elsewhere once they have been obtained from the
 * dictionary.
 * 
 * @param dict the dictionary to query
 * @param key the key to lookup
 * @param valueP a pointer that is set to the result of the lookup if
 * successful. The memory for the value is internal to the dictionary
 * and will be freed when the dictionary is freed. Note that this is
 * different from the way aniAsfDictGetInt behaves. If the key cannot be
 * found, this pointer is set to the parameter defaultStr
 * @param defaultStr the default string to return if the key cannot be
 * found. This parameter may be NULL in which case the operation will
 * return ANI_E_FAILED if the key is not found.
 * 
 * @return a positive number indicating the length of the string that
 * is returned via valueP; or ANI_E_FAILED if the key is not found and
 * defaultStr is NULL. If the key was found but did not contain
 * double-quotes at the start and end, then ANI_E_ILLEGAL_ARG is
 * returned.
 * 
 * @see aniAsfDictGetString
 * @see aniAsfDictSetDelimitedString
 * @see aniAsfDictGetInt
 * @see aniAsfDictGetBool
 */
int
aniAsfDictGetDelimitedString(tAniDict *dict, 
                             const u_char *key, 
                             u_char **valueP, 
                             u_char *defaultStr);

/**
 * aniAsfDictGetInt
 *
 * FUNCTION:
 * Returns the integer value stored for the given key.
 *
 * Note: This operation looks up the key within a table maintained by
 * the dictionary. Thus, a dictionary should not be used as storage
 * for values that are frequently looked up. Instead, such values
 * should be cached elsewhere once they have been obtained from the
 * dictionary.
 * 
 * @param dict the dictionary to query
 * @param key the key to lookup
 * @param value the integer value associated with the key is stored in
 * this parameter. Note that this is different from the valueP used in
 * aniAsfDictGetString in that the storage for this is allocated by the
 * caller.
 * @param defaultInt the default integer value that should be stored
 * in value if the key is not found
 * 
 * @return ANI_OK if the key was found and its value successfully
 * parsed
 * 
 * @see aniAsfDictGetString
 */
int
aniAsfDictGetInt(tAniDict *dict, const u_char *key, int *value, int defaultInt);

/**
 * aniAsfDictGetBool
 *
 * Returns the boolean value stored for the given key. Boolean values
 * are defined as the strings "true" and "false".
 *
 * Note: This operation looks up the key within a table maintained by
 * the dictionary. Thus, a dictionary should not be used as storage
 * for values that are frequently looked up. Instead, such values
 * should be cached elsewhere once they have been obtained from the
 * dictionary.
 * 
 * @param dict the dictionary to query
 * @param key the key to lookup
 * @param value the boolean value associated with the key is stored in
 * this parameter. Storage for this is allocated by the caller.
 * @param default the default boolean value that should be stored
 * if the key is not found or if its value is not parseable.
 * 
 * @return ANI_OK if the key was found and its value successfully
 * parsed
 * 
 * @see aniAsfDictGetString
 */
int
aniAsfDictGetBool(tAniDict *dict, 
                  const u_char *key, 
                  tAniBoolean *value, 
                  tAniBoolean defaultVal);

/**
 * aniAsfDictSetString
 *
 * Stores the given key-value mapping in the dictionary. If the key is
 * already present, the older mapping is deleted.  NOTE: this is one
 * reason the application must not maintain pointers to the internal
 * data of the dictionary.
 *
 * If the key and value were written out to a file, the total length
 * of the entry "key=value" should not be greater than
 * ANI_MAX_DICT_LINE_SIZE.
 *
 * Passing in a value of NULL will clear any existing key-value
 * mapping for the same key.
 *
 * @param dict the dictionary to store in
 * @param key the key
 * @param value the value. If this is NULL, then nothing is stored,
 * and any existing key-value mapping is deleted.
 *
 * @return ANI_OK if the operation succeeds. If this is a delete
 * operation (where value is NULL), then ANI_E_FAILED might indicate
 * that the key could not be found and should not be treated as a
 * severe error.
 */
inline int
aniAsfDictSetString(tAniDict *dict, ANI_U8 *key, ANI_U8 *value);

/**
 * aniAsfDictSetDelimitedString
 *
 * Stores the given key-value mapping in the dictionary, but with the
 * value quoted inside a delimiter. This is useful when the value
 * contains leading or trailing spaces because the retreival functions
 * strip off extra spaces unless the string is quoted. 
 *
 * e.g., calling this function with the key "myKey" and the value
 * " myValue  " will create the following entry in the dictionary
 * file:
 *    myKey=" myValue  "
 *
 * As with the basic aniAsfDictSetString(...) function, if the key is
 * already present, the older mapping is deleted (irrespective of
 * whether it was quoted or not).
 *
 * If the key and value were written out to a file, the total length
 * of the entry "key=value" should not be greater than
 * ANI_MAX_DICT_LINE_SIZE.
 *
 * Passing in a value of NULL will clear any existing key-value
 * mapping for the same key.
 *
 * @param dict the dictionary to store in
 * @param key the key
 * @param value the value.  If this is NULL, then nothing is stored,
 * and any existing key-value mapping is deleted.
 *
 * @return ANI_OK if the operation succeeds. If this is a delete
 * operation (where value is NULL), then ANI_E_FAILED might indicate
 * that the key could not be found and should not be treated as a
 * severe error.
 */
inline int
aniAsfDictSetDelimitedString(tAniDict *dict, ANI_U8 *key, ANI_U8 *value);

/**
 * aniAsfDictSave
 *
 * Saves the entries contained within the dictionary to an ASCII file
 * in a key=value form, one entry per line.
 *
 * @param dict the dictionary whose entries should be saved
 * @param fileName the complete path to the file where this should be
 * saved
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfDictSave(tAniDict *dict, char *fileName);

#endif // _ANI_ASF_DICT_H_
