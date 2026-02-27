/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * File: $Header: //depot/software/projects/feature_branches/gen5_phase1/os/linux/classic/ap/apps/asf/aniAsfDict.c#3 $
 * Contains definitions for utilities that read a dictionary of key
 * value pairs from a text file. The dictionary can be queried to
 * return values as strings or as integers.
 * Author:      Mayank D. Upadhyay
 * Date:        03-July-2002
 * History:-
 * Date         Modified by     Modification Information
 * ------------------------------------------------------
 *
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

#include <aniAsfDict.h>
#include <aniAsfHashTable.h>
#include <aniErrors.h>
#include <aniAsfLog.h>

#define NUM_HASH_BUCKETS 16

#define ANI_DICT_TRUE_STR  "true"
#define ANI_DICT_FALSE_STR "false"

#define ANI_DICT_STR_DELIMITER '"'

struct tAniDict {
    tAniHashTable *table;
};

typedef struct sAniAsfDictEntry {
    u_char line[ANI_MAX_DICT_LINE_SIZE + 1];
    u_char *key;
    u_char *value;
} tAniAsfDictEntry;

static u_char 
*trim(u_char *str);

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
 * Note: If this function is able to read at least one key-value pair
 * from the dictionary file before an error occurs, it will return a
 * non-NULL result in dictP. The error might be an out of memory
 * error, in which case it makes sense to return the dictionary that
 * has been read so far, as well as an ANI_E_MALLOC_FAILED error code.
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
aniAsfDictCreate(tAniDict **dictP, char *fileName)
{
    tAniDict *dict;
    tAniAsfDictEntry *entry;
    tAniAsfDictEntry *oldEntry;
    FILE *dictFile;
    char line[ANI_MAX_DICT_LINE_SIZE + 1];
    char *ptr;
    int retVal;

    assert(fileName);
    if (fileName == NULL)
        return ANI_E_ILLEGAL_ARG;
    
    dictFile = fopen(fileName, "r");
    if (dictFile == NULL) {
        aniAsfLogMsg(LOG_WARNING, 
                     ANI_LOG_FUNC, 
                     "Could not open dictionary file %s: %s",
                     fileName,
                     strerror(errno));
        assert(0 && "Failed to open dictionary file!");
        return ANI_E_FAILED;
    }
    
    dict = calloc(sizeof(tAniDict), 1);
    if (dict == NULL) {
        aniAsfLogMsg(LOG_CRIT, 
                     ANI_LOG_FUNC, 
                     "Could not allocate memory for dictionary!");
        fclose(dictFile);
        assert(0 && "Failed to allocate dictionary!");
        return ANI_E_MALLOC_FAILED;
    }

    retVal = aniAsfHashTableInit(&(dict->table), 
                                 NUM_HASH_BUCKETS, 
                                 (tAniAsfCompare) aniAsfHashCmpString,
                                 (tAniAsfHash) aniAsfHashString);
    assert(retVal == ANI_OK);
    if (retVal != ANI_OK) {
        fclose(dictFile);
        free(dict);
        return retVal;
    }
    
    // Now dict will be passed back no matter what
    *dictP = dict;

    // Now loop and read all key-value pairs, one per line
    do {

        for (ptr = line; ((ptr - line) < (int)(sizeof(line) - 1)); ptr++) {
            if ((fread(ptr, sizeof(line[0]), 1, dictFile) != sizeof(line[0])) ||
                (*ptr == '\n'))
                break; 
        }

        // Terminate the string, potentially overwriting the trailing newline.
        *ptr = '\0';

        if (line[0] == '#')
            continue; // Ignore comments
        ptr = strchr(line, '=');
        if (ptr == NULL || *(ptr + 1) == '\0')
            continue; // Ignore malformed line

        *ptr = '\0';
        ptr++;

        entry = calloc(sizeof(tAniAsfDictEntry), 1);
        assert(entry != NULL);
        if (entry == NULL) {
            fclose(dictFile);
            return ANI_E_MALLOC_FAILED;
        }

        // Copy all input chars
        memcpy(entry->line, line, sizeof(line));

        // Find the first char after the '='
        entry->value = entry->line + (ptr - line);

        // Trim the key and value
        entry->value = trim(entry->value);
        entry->key = trim(entry->line);

        retVal = aniAsfHashTableInsert(dict->table,
                                    entry->key, 
                                    entry, 
                                    (void **)&oldEntry);
        assert(retVal == ANI_OK);
        if (retVal != ANI_OK) {
            free(entry);
            fclose(dictFile);
            return retVal;
        }
        if (oldEntry != NULL) {
            aniAsfLogMsg(LOG_DEBUG, 
                         ANI_LOG_FUNC, 
                         "Dictionary file %s has duplicate mappings for %s. "
                         "Overriding %s with %s.",
                         fileName, entry->key, oldEntry->value, entry->value);
            free(oldEntry);
        }

    } while (!feof(dictFile) && !ferror(dictFile));
    
    fclose(dictFile);

    return ANI_OK;
}

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
aniAsfDictFree(tAniDict *dict)
{
    assert(dict != NULL);
    if (dict == NULL)
        return ANI_E_NULL_VALUE;
    assert(dict->table != NULL);
    if (dict->table != NULL)
        aniAsfHashTableFree(dict->table, (tAniAsfWalk) free);
    free(dict);
    return ANI_OK;
}


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
                 u_char *defaultStr)
{
    int retVal;
    tAniAsfDictEntry *entry;

    assert(dict != NULL);
    if (dict == NULL)
        return ANI_E_NULL_VALUE;
    assert(dict->table != NULL);
    if (dict->table == NULL)
        return ANI_E_NULL_VALUE;
    retVal = aniAsfHashTableLookup(dict->table, 
                                   (u_char *) key, 
                                   (void **) &entry);
    if (retVal == ANI_OK)
        *valueP = entry->value;
    else
        *valueP = defaultStr;
    if (*valueP == NULL)
        retVal = ANI_E_FAILED;
    else
        retVal = strlen((char *)*valueP);
    return retVal;
}

/**
 * aniAsfDictGetDelimitedString
 *
 * FUNCTION:
 * Returns the string value stored for the given key. The string is
 * expected to be enclosed within double quotes. This is useful for
 * retrieving values that can contain leading or trailing
 * white-space which the caller wants to retrieve. (The function
 * aniAsfDictGetString trims off leading and trailing white spac
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
                             u_char *defaultStr)
{
    int retVal;
    ANI_U8 *tmpStr = NULL;

    retVal = aniAsfDictGetString(dict, key, &tmpStr, defaultStr);
    if (retVal >= 0) {

        // First check if we have obtained the default string. That
        // would imply that the dictionary did not contain a mapping
        // for this key.
        if (tmpStr == defaultStr) {

            // Return the default string to the caller
            *valueP = tmpStr;

        } else {

            // The dictionary did contain a mapping. Now check if it
            // is enclosed in the delimiter
            if (retVal < 2) {

                // This cannot possibly be enclosed in the delimiter
                retVal = ANI_E_ILLEGAL_ARG;

            } else {

                // The string potentially contains the delimiter around it
                if (tmpStr[0] != ANI_DICT_STR_DELIMITER ||
                    tmpStr[retVal - 1] != ANI_DICT_STR_DELIMITER) {

                    // Nope...not enclosed in delimiter
                    retVal = ANI_E_ILLEGAL_ARG;

                } else {

                    // This is a valid string. Trim the delimiters
                    *valueP = tmpStr + 1;
                    tmpStr[retVal - 1] = '\0';
                    retVal -= 2;
                } // Valid string
            } // Potentially valid string
        } // Dictionary has a mapping
    } // Dictionary returned some string, default or otherwise

    if (retVal == ANI_E_ILLEGAL_ARG) {
        // Some error was detected
        aniAsfLogMsg(LOG_WARNING, 
                     ANI_LOG_FUNC, 
                     "Could not find legal mapping for %s enclosed in %c...%c!",
                     key, ANI_DICT_STR_DELIMITER, ANI_DICT_STR_DELIMITER);
    }

    return retVal;
}

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
 *  * 
 * @return ANI_OK if the key was found and its value successfully
 * parsed. Note that an empty string will not be considered
 * successfully parsed.
 * 
 * @see aniAsfDictGetString
 */
int
aniAsfDictGetInt(tAniDict *dict, const u_char *key, int *value, int defaultInt)
{
    int len;

    u_char *str;
    len = aniAsfDictGetString(dict, key, &str, NULL);
    if (len <= 0) {
        *value = defaultInt;
        return ANI_E_FAILED;
    }
    *value = atoi((char *)str);
    return ANI_OK;
}

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
 * @param defaultVal the default boolean value that should be stored
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
                  tAniBoolean defaultVal)
{
    int retVal = ANI_OK;
    u_char *str = NULL;

    // Obtain the key-value mapping from the hashtable
    aniAsfDictGetString(dict, key, &str, NULL);
    if (str != NULL) {
        /*
         * Use the key-value mapping only if it is parseable
         */
        if (strcmp((const char *)str, ANI_DICT_TRUE_STR) == 0) {
            *value = eANI_BOOLEAN_TRUE;
        } else if (strcmp((const char *)str, ANI_DICT_FALSE_STR) == 0) {
            *value = eANI_BOOLEAN_FALSE;
        } else {
            str = NULL;
        }
    }

    // Either the string was not available, or unparseable
    if (str == NULL) {
            *value = defaultVal;
            retVal = ANI_E_FAILED;
    }

    return retVal;
}

// Stores the string quoted within the provided delimiter. If value is
// NULL, then this is a delete operation.
int
setString(tAniDict *dict, ANI_U8 *key, ANI_U8 *value, ANI_U8 delimiter)
{
    tAniAsfDictEntry *entry = NULL;
    tAniAsfDictEntry *oldEntry = NULL;
    int len;
    int retVal;

    assert(dict != NULL);
    assert(dict->table != NULL);

    if (dict == NULL || dict->table == NULL)
        return ANI_E_NULL_VALUE;    

    if (value == NULL) {
        // This is a delete operation
        retVal = aniAsfHashTableRemove(dict->table,
                                       key, 
                                       (void **)&oldEntry);
        if (oldEntry != NULL) {
            aniAsfLogMsg(LOG_DEBUG,
                         ANI_LOG_FUNC,
                         "Deleting mapping %s=%s.",
                         oldEntry->key, oldEntry->value);
            free(oldEntry);
        }
        return retVal;
    }

    // We allocate storage for both strings, their NULL terminators,
    // and the delimiter characters if provided
    len = strlen((char *)key) + strlen((char *)value) + 1 + 1 + (delimiter == '\0' ? 0 : 2);
    if (len > ANI_MAX_DICT_LINE_SIZE)
        return ANI_E_ILLEGAL_ARG;

    entry = calloc(sizeof(tAniAsfDictEntry), 1);
    assert(entry != NULL);
    if (entry == NULL) {
        return ANI_E_MALLOC_FAILED;
    }

    entry->key = entry->value = entry->line;
    entry->value += snprintf((char *)entry->line, ANI_MAX_DICT_LINE_SIZE, "%s", key);
    entry->value++;
    len = ANI_MAX_DICT_LINE_SIZE - (entry->value - entry->line);
    if (delimiter == '\0') {
        snprintf((char *)entry->value, len, "%s", value);
    } else {
        snprintf((char *)entry->value, len, "%c%s%c", ANI_DICT_STR_DELIMITER, value, ANI_DICT_STR_DELIMITER);
    }

    retVal = aniAsfHashTableInsert(dict->table,
                                   entry->key, 
                                   entry, 
                                   (void **)&oldEntry);
    assert(retVal == ANI_OK);
    if (retVal != ANI_OK) {
        free(entry);
        return retVal;
    }
    if (oldEntry != NULL) {
        aniAsfLogMsg(LOG_DEBUG,
                     ANI_LOG_FUNC, 
                     "Replacing mapping for %s from %s to %s.",
                     entry->key, oldEntry->value, entry->value);
        free(oldEntry);
    }

    return ANI_OK;
}

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
aniAsfDictSetString(tAniDict *dict, ANI_U8 *key, ANI_U8 *value)
{
    return setString(dict, key, value, '\0');
}

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
aniAsfDictSetDelimitedString(tAniDict *dict, ANI_U8 *key, ANI_U8 *value)
{
    return setString(dict, key, value, ANI_DICT_STR_DELIMITER);
}

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
aniAsfDictSave(tAniDict *dict, char *fileName)
{
    int retVal;
    int i;

    tAniAsfIterator itor;
    tAniAsfDictEntry *entry;
    FILE *dictFile;

    assert(dict != NULL);
    if (dict == NULL)
        return ANI_E_NULL_VALUE;

    assert(dict->table != NULL);
    if (dict->table == NULL)
        return ANI_E_ILLEGAL_ARG;

    assert(fileName);
    if (fileName == NULL)
        return ANI_E_ILLEGAL_ARG;
    
    dictFile = fopen(fileName, "w");
    if (dictFile == NULL) {
        aniAsfLogMsg(LOG_WARNING, 
                     ANI_LOG_FUNC, 
                     "Could not open dictionary file %s for write: %s",
                     fileName,
                     strerror(errno));
        assert(0 && "Failed to open dictionary file for write!");
        return ANI_E_FAILED;
    }
    
    retVal = aniAsfHashTableGetIterator(dict->table, &itor);
    if (retVal != ANI_OK) {
        assert(0 && "Could not get iterator from dictionary!");
        goto error;
    }

    for (i = 0; i < itor.numValues; i++) {
        entry = (tAniAsfDictEntry *) itor.values[i];
        fprintf(dictFile, "%s=%s\n", entry->key, entry->value);
    }

    free(itor.values);

 error:
    fclose(dictFile);

    return retVal;
}

static u_char 
*trim(u_char *str)
{
    u_char *ptr;

    // Find the first non white-space
    for (ptr = str; isspace(*ptr); ptr++);

    if (*ptr == '\0')
        return str; // Fail another way!

    // This is the new start of the string
    str = ptr;

    // Find the last non white-space
    ptr += strlen((char *)ptr) - 1;
    for (; ptr != str && isspace(*ptr); ptr--);

    // Null terminate the following character
    ptr[1] = '\0';

    return str;
}
