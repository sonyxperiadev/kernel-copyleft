/*
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * $File: //depot/software/projects/feature_branches/gen5_phase1/os/linux/classic/ap/apps/include/aniAsfHashTable.h $
 *
 * Contains declarations for a flexible hashtable module. This
 * hashtable library is thread-safe. A pthread_mutex is maintained per
 * hashtable instance.
 *
 * NOTE: Allocation and freeing of this structure is not mutext
 * protected. The use of the structure for lookups and storage is
 * mutex protected.
 *
 * Author:      Mayank D. Upadhyay
 * Date:        02-July-2002
 * History:-
 * Date         Modified by     Modification Information
 * ------------------------------------------------------
 *
 */
#ifndef _ANI_ASF_HASHTABLE_H_
#define _ANI_ASF_HASHTABLE_H_

#include <sys/types.h>
#include <aniUtils.h>

typedef int (*tAniAsfCompare)(void *ele1, void *ele2);
typedef int (*tAniAsfHash)(void *ele, int n);
typedef int (*tAniAsfWalk)(void *ele);

/**
 * The opaque hashtable structure that maps keys to values. The
 * hashtable is initialized to a fixed number of buckets and provided
 * with a hashing function that takes a key and returns the index of
 * the bucket that key would fall into (starting at 0).
 *
 * Note: This hashtable relies on the application to do memory
 * management of the key-value pairs that are passed in. The
 * hashtable simple stores internal pointers to the data that the
 * caller provides.
 *
 * NOTE: Allocation and freeing of this structure is not mutext
 * protected. The use of the structure for lookups and storage is
 * mutex protected.
 *
 * The library is thread-safe and maintains a pthread_mutex per
 * instance of hashtable.
 */
typedef struct tAniHashTable tAniHashTable;

/**
 * aniAsfHashTableInit
 *
 * FUNCTION:
 * Allocates storage for a hashtable and initializes it to the given
 * size. The hashtable will use the provided hashing function and
 * comparison function when entries need to be stored or looked up.
 *
 * NOTE: Allocation and freeing of this structure is not mutext
 * protected. The use of the structure for lookups and storage is
 * mutex protected.
 *
 * @param tableP will be set to the newly allocated hashtable if the
 * operation succeeds.
 * @param size the number of buckets that the hashtable will contain.
 * @param cmp the comparison function that will be used to compare the
 * keys within the same bucket. When called cmp(a,b) should return a
 * negative value if a<b, 0 if a==b, and a positive value if a>b.
 * @hash the hashing function that will be used to determine the
 * bucket that a key falls into. The hashing function is assumed to
 * return an integer in the range 0 to size-1 (inclusive).
 *
 * @return ANI_OK if the operation succeeds.
 * @see aniAsfHashTableFree
 */
int
aniAsfHashTableInit(tAniHashTable **tableP, 
                 u_int size, 
                 tAniAsfCompare cmp,
                 tAniAsfHash hash);

/**
 * aniAsfHashTableInsert
 *
 * FUNCTION:
 * Adds a given key-value pair in the hashtable. Storage for both the
 * key and the value is managed by the caller. The hashtable simply
 * maintains pointers to the parameters passed in. If the hashtable
 * has an existing key-value mapping for the same key, then the new
 * mapping overrides the older one and the function returns a pointer
 * to the older value. (Note: It is assumed that the application has
 * access to the storage for the older key -- probably as a field of
 * the its value structure.)
 *
 * @param table the hashtable to add to
 * @param key the key value
 * @param value the value associated with the key
 * @param oldValueP if the key has an existing value in the hashtable,
 * then that value is returned thru this pointer; otherwise, this
 * pointer is set to NULL.
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfHashTableInsert(tAniHashTable *table, 
                   void *key,
                   void *value,
                   void **oldValueP);

/**
 * aniAsfHashTableLookup
 *
 * FUNCTION:
 * Searches thru a hashtable and returns the value associated with the
 * given key. The storage used for value returned is the same that was
 * passed by the caller when the value was inserted into the
 * hashtable.
 *
 * @param table the hashtable
 * @param key the key that needs to be looked up
 * @param valuePtr if the key is found then this pointer is set to the
 * value that is associated with the key
 *
 * @return ANI_OK if the key was found; ANI_E_FAILED if the key is
 * not found.
 */
int
aniAsfHashTableLookup(tAniHashTable *table, 
                   void *key,
                   void **valuePtr);

/**
 * aniAsfHashTableRemove
 *
 * FUNCTION:
 * Searches thru a hashtable and removes a key-value association. The
 * function returns the value associated with the given key. The
 * storage used for value returned is the same that was  passed by the
 * caller when the value was inserted into the hashtable.
 *
 * @param table the hashtable
 * @param key the key that needs to be looked up
 * @param valuePtr if the key is found then this pointer is set to the
 * value that is associated with the key
 *
 * @return ANI_OK if the key was found; ANI_E_FAILED if the key is
 * not found.
 */
int
aniAsfHashTableRemove(tAniHashTable *table, 
                   void *key,
                   void **valuePtr);

/**
 * aniAsfHashTableWalk
 *
 * FUNCTION:
 * Iterates over all entries in the hashtable and invokes an
 * application specified callback on each of them.
 *
 * @param table the hashtable to iterate over
 * @param callback the application callback to invoke
 *
 * @see aniAsfHashTableGetIterator 
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfHashTableWalk(tAniHashTable *table, tAniAsfWalk callback);

/**
 * aniAsfHashTableNumEntries
 *
 * FUNCTION:
 * Returns the number of entries currently stored in this hashtable.
 * 
 * @param table the hashtable to operate on
 *
 * @return the non-negative number of entries currently stored in this
 * hashtable if the operation succeeds.
 */
int
aniAsfHashTableNumEntries(tAniHashTable *table);

/**
 * aniAsfHashTableGetIterator
 *
 * Returns a lightweight iterator for the hashtable with the current
 * list of entries in it.
 *
 * NOTE: It is the responsibility of the caller to free the "values"
 * member of the iterator, ie., the array of pointers contained inside
 * the iterator. The caller must not free the data dereferenced by
 * these pointers (e.g., values[0]) as that data is shared with the
 * hashtable.
 *
 * @param table the hashtable to operate on
 * @param itor the iterator that will be populated with the list of
 * pointers to existing values. If there are no entries in the
 * hashtable, then itor->values will be set to NULL while
 * itor->numValues will be set to 0.
 * 
 * @see aniAsfHashTableWalk
 *
 * @return ANI_OK if th eoperation succeeds
 */
int
aniAsfHashTableGetIterator(tAniHashTable *table, 
                           tAniAsfIterator *itor);

/**
 * aniAsfHashTableFree
 *
 * FUNCTION:
 * Frees all key-value associations maintained in the hashtable, and
 * invokes an application specified callback on each value as its
 * association is freed.
 *
 * Note: The function does not free the actual value that was present
 * in the association. The application has to do that in the
 * callback. Also, it is assumed that the memory for the key is
 * accessible to the application thru some means, e.g., as a field in
 * the value structure.
 *
 * NOTE: Allocation and freeing of this structure is not mutext
 * protected. The use of the structure for lookups and storage is
 * mutex protected.
 *
 * @param table the hashtable to free
 * @param callback the callback that will be invoked on each
 * value stored in the hashtable (not the key)
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfHashTableFree(tAniHashTable *table, tAniAsfWalk callback);

#ifdef ANI_DEBUG
int
aniAsfHashTablePrint(tAniHashTable *table, tAniAsfWalk callback);
#endif // ANI_DEBUG

/**
 * aniAsfHashString
 *
 * FUNCTION:
 * A utility hash function that applications can use as a callback
 * for their hashtables storing null terminated strings.
 *
 * @param str the string to be hashed
 * @param tableSize the upper limit on the hash value
 *
 * @return an integer from 0 to tableSize-1
 */
int
aniAsfHashString(u_char *str, u_int tableSize);

/**
 * Utility function to use as string comparison callback for hashtable
 * containing strings. // TBD: Make this an alias for strcmp
 */
int
aniAsfHashCmpString(u_char *str1, u_char *str2);

/**
 * aniAsfHashMacAddr
 *
 * FUNCTION:
 * A utility hash function that application can use as a callback
 * for their hashtables storing MAC addresses.
 *
 * @param macAddr the MAC address to be hashed
 * @param tableSize the upper limit on the hash value
 *
 * @return an integer from 0 to tableSize-1
 */
int
aniAsfHashMacAddr(tAniMacAddr macAddr, u_int tableSize);

/**
 * Utility function to do MAC address comparison for hashtables that
 * store MAC addresses.
 */
int
aniAsfHashCmpMacAddr(tAniMacAddr mac1, tAniMacAddr mac2);

#endif // _ANI_ASF_HASHTABLE_H_
