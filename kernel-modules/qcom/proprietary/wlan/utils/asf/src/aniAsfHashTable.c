/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * $File: //depot/software/projects/feature_branches/gen5_phase1/os/linux/classic/ap/apps/asf/aniAsfHashTable.c $
 *
 * Contains definitions for a flexible hashtable module. This
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

#include <malloc.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include <aniAsfHashTable.h>
#include <aniUtils.h>
#include <aniErrors.h>
#include <aniAsfLog.h>

#define FIND_BUCKET(table, key) \
    ((table)->buckets + (table)->hash(key, (table)->size));

#define NO_OP       0
#define DELETE_NODE 1

#define LOCK(lock) \
                   if (pthread_mutex_lock(&lock) != 0) { \
                       aniAsfLogMsg(ANI_MUTEX_ERR); \
                       assert(0 && "Failed to lock mutex!"); \
                       return ANI_E_FAILED; \
                   }

#define UNLOCK(lock) \
                   if (pthread_mutex_unlock(&lock) != 0) { \
                       aniAsfLogMsg(ANI_MUTEX_ERR); \
                       assert(0 && "Failed to unlock mutex!"); \
                       return ANI_E_FAILED; \
                   }

#define ASSERT_NON_NULL(value) \
                assert(value != NULL); \
                if (value == NULL) { \
                   return ANI_E_NULL_VALUE; \
                }

#define ASSERT_NO_ERROR_UNLOCK(status) \
                assert(status >= 0); \
                if (status < 0)  { \
                   retVal = status; \
                   goto done; \
                }

typedef struct tEntry tEntry;

struct tEntry {
    void *key;
    void *value;
    tEntry *next;
};

struct tAniHashTable {
    tEntry *buckets;
    u_int size;
    int numEntries;
    pthread_mutex_t lock;
    tAniAsfCompare cmp;
    tAniAsfHash hash;
};

static int
searchList(tEntry *bucketHead, 
           void *key,
           tAniAsfCompare cmp, 
           tEntry **nodeP,
           tEntry **prevP);

static int 
freeHashTable(tAniHashTable *table, tAniAsfWalk callback);

static int
walkAndProcess(tAniHashTable *table, tAniAsfWalk callback, u_char op);

static int
setValuesInIterator(tAniHashTable *table, tAniAsfIterator *itor);


/**
 * aniAsfHashTableInit
 *
 * FUNCTION:
 * Allocates storage for a hashtable and initializes it to the given
 * size. The hashtable will use the provided hashing function and
 * comparison function when entries need to be stored or looked
 * up. Also initializes the pthread_mutex for this hashtable.
 *
 * NOTE: Allocation and freeing of this structure is not mutext
 * protected. The use of the structure for lookups and storage is
 * mutex protected.
 *
 * @param tableP will be set to the newly allocated hashtable if the
 * operation succeeds.
 * @param size the number of buckets that the hashtable will contain
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
                 tAniAsfHash hash)
{
    tAniHashTable *hashtable;

    assert(cmp != NULL);
    assert(hash != NULL);
    if (cmp == NULL || hash == NULL)
        return ANI_E_ILLEGAL_ARG;

    hashtable = calloc(sizeof(tAniHashTable), 1);
    if (hashtable == NULL) {
        aniAsfLogMsg(LOG_CRIT, 
                     ANI_LOG_FUNC, 
                     "Could not allocate memory for hashtable!");
        assert(0 && "Could not allocate hashtable!");
        return ANI_E_MALLOC_FAILED;
    }

    hashtable->buckets = calloc(sizeof(tEntry), size);
    if (hashtable->buckets == NULL) {
        free(hashtable);

        aniAsfLogMsg(LOG_CRIT, 
                     ANI_LOG_FUNC, 
                     "Could not allocate memory for hashtable buckets!");
        assert(0 && "Failed to allocate hashtable buckets!");
        return ANI_E_MALLOC_FAILED;
    }

    // No need to initialize all buckets to NULL since we used calloc
    // memset(...)

    hashtable->size = size;
    hashtable->numEntries = 0;
    hashtable->cmp = cmp;
    hashtable->hash = hash;

    if (pthread_mutex_init(&hashtable->lock, NULL) != 0) {
        aniAsfLogMsg(ANI_MUTEX_ERR);
        assert(0 && "Failed to allocate mutex!");
        freeHashTable(hashtable, NULL);
        return ANI_E_FAILED;
    }

    *tableP = hashtable;

    return ANI_OK;
}

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
 * @return ANI_OK if the operation succeeds; ANI_E_MALLOC_FAILED if
 * the required book-keeping memory cannot be allocated.
 */
int
aniAsfHashTableInsert(tAniHashTable *table, 
                   void *key,
                   void *value,
                   void **oldValueP)
{
    tEntry *node;
    tEntry *prevNode;
    tEntry *newNode;
    tEntry *bucketHead;
    int retVal;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(key);

    LOCK(table->lock);

    *oldValueP = NULL;
    bucketHead = FIND_BUCKET(table, key);

    /*
     * Check if the bucket is empty; if so, store in the head of the
     * bucket
     */
    if (bucketHead->key == NULL) {
        bucketHead->key = key;
        bucketHead->value = value;
        bucketHead->next = NULL;
        table->numEntries++;
        retVal = ANI_OK;
        goto done;
    }

    /*
     * There is atleast one entry in the bucket. Find the first
     * entry in the bucket whose key is the same as or greater than
     * this key.
     */
    retVal = searchList(bucketHead, key, table->cmp, &node, &prevNode);

    if (retVal == ANI_OK) {
        // We found an entry with the same key, so simply replace it
        *oldValueP = node->value;
        node->value = value;
        node->key = key;
        retVal = ANI_OK;
        goto done;
    }

    // We have to allocate a new node for sure!
    newNode = calloc(sizeof(tEntry), 1);
    if (!newNode) {
        aniAsfLogMsg(LOG_CRIT, 
                     ANI_LOG_FUNC, 
                     "Could not allocate memory for new hashtable entry!");
        assert(0 && "Failed to allocate hashtable entry!");
        retVal = ANI_E_MALLOC_FAILED;
        goto done;
    }

    table->numEntries++;

    // Would this key go into the head of the linked list?
    if (prevNode == NULL) {
        // Move the data from the head over to the new node, and fill
        // the new data into the head
        newNode->key = bucketHead->key;
        newNode->value = bucketHead->value;
        newNode->next = bucketHead->next;
        bucketHead->key = key;
        bucketHead->value = value;
        bucketHead->next = newNode;
        retVal = ANI_OK;
        goto done;
    }

    // newNode will be inserted in the list after node
    newNode->next = prevNode->next;
    prevNode->next = newNode;

    newNode->key = key;
    newNode->value = value;

    retVal = ANI_OK;

 done:
    UNLOCK(table->lock);
    return retVal;
}

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
 * @param valueP if the key is found then this pointer is set to the
 * value that is associated with the key
 *
 * @return ANI_OK if the key was found; ANI_E_FAILED if the key is
 * not found.
 */
int
aniAsfHashTableLookup(tAniHashTable *table, 
                   void *key,
                   void **valueP)
{
    tEntry *node;
    tEntry *bucketHead;
    tEntry *prevNode;
    int retVal;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(key);

    LOCK(table->lock);

    bucketHead = FIND_BUCKET(table, key);
    retVal = searchList(bucketHead, key, table->cmp, &node, &prevNode);
    if (retVal == ANI_OK)
        *valueP = node->value;

    UNLOCK(table->lock);

    return retVal;
}

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
 * @param valueP if the key is found then this pointer is set to the
 * value that is associated with the key
 *
 * @return ANI_OK if the key was found; ANI_E_FAILED if the key is
 * not found.
 */
int
aniAsfHashTableRemove(tAniHashTable *table, 
                  void *key,
                  void **valueP)
{
    tEntry *node;
    tEntry *bucketHead;
    tEntry *prevNode;
    int retVal;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(key);

    LOCK(table->lock);

    bucketHead = FIND_BUCKET(table, key);

    /*
     * Check if the bucket is empty; if so, simply return
     */
    if (bucketHead->key == NULL) {
        retVal = ANI_E_FAILED;
        goto done;
    }

    /*
     * There is atleast one entry in the bucket. Find the first
     * entry in the bucket whose key is the same as or greater than
     * this key.
     */
    retVal = searchList(bucketHead, key, table->cmp, &node, &prevNode);

    if (retVal != ANI_OK || prevNode == NULL || node == NULL) {
        retVal = ANI_E_FAILED;
        goto done;
    }

    // We found an entry with the same key
    *valueP = node->value;
    table->numEntries--;

    // If this node is in the head of the bucket, then handle
    // specially
    if (node == bucketHead) {
        tEntry *nextNode = node->next;
        if (nextNode != NULL) {
            node->value = nextNode->value;
            node->key = nextNode->key;
            node->next = nextNode->next;
            free(nextNode);
        } else {
            // Simply NULL out the bucket head
            node->value = node->key = NULL;
        }
        retVal = ANI_OK;
        goto done;
    }

    prevNode->next = node->next;
    free(node);

    retVal = ANI_OK;

 done:
    UNLOCK(table->lock);
    return retVal;;
}

/**
 * aniAsfHashTableWalk
 *
 * FUNCTION:
 * Iterates over all entries in the hashtable and invokes an
 * application specified callback on each of them. 
 *
 * NOTE: Applications should be careful that this function grabs a
 * mutex on the hashtables as it walks through all entries. Therefore,
 * the application callback should not try to perform an operation on
 * the same hashtable.
 *
 * @param table the hashtable to iterate over
 * @param callback the application callback to invoke
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfHashTableWalk(tAniHashTable *table, tAniAsfWalk callback)
{
    int retVal;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(callback);

    LOCK(table->lock);

    retVal = walkAndProcess(table, callback, NO_OP);

    UNLOCK(table->lock);
    return retVal;
}

/**
 * aniAsfHashTableNumEntries
 *
 * FUNCTION:
 * Returns the number of entries currently stored in this hashtable.
 * 
 * @return the non-negative number of entries currently stored in this
 * hashtable if the operation succeeds.
 */
int
aniAsfHashTableNumEntries(tAniHashTable *table)
{
    int retVal;

    ASSERT_NON_NULL(table);

    LOCK(table->lock);

    retVal = table->numEntries;

    UNLOCK(table->lock);

    return retVal;
}

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
                           tAniAsfIterator *itor)
{
    int retVal;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(itor);

    LOCK(table->lock);

    itor->numValues = table->numEntries;

    if (itor->numValues == 0) {
        itor->values = NULL;
        retVal = ANI_OK;
        goto done;
    }
        
    itor->values = (void **) calloc(sizeof(void *), itor->numValues);
    if (itor->values == NULL) {
        aniAsfLogMsg(LOG_CRIT, 
                     ANI_LOG_FUNC, 
                     "Could not allocate memory for "
                     "hashtable iterator array!");
        assert(0 && "Could not allocate hashtable iterator array!");
        retVal = ANI_E_MALLOC_FAILED;
        goto done;
    }

    setValuesInIterator(table, itor);

    retVal = ANI_OK;

 done:
    UNLOCK(table->lock);
    return retVal;
}

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
aniAsfHashTableFree(tAniHashTable *table, tAniAsfWalk callback)
{
    int retVal;

    ASSERT_NON_NULL(table);

    if (pthread_mutex_destroy(&table->lock) != 0) {
        aniAsfLogMsg(ANI_MUTEX_ERR);
        assert(0 && "Failed to destroy mutex!");
    }

    retVal = freeHashTable(table, callback);

    return retVal;
}

static int 
freeHashTable(tAniHashTable *table, tAniAsfWalk callback) {
    walkAndProcess(table, callback, DELETE_NODE);
    free(table->buckets);
    free(table);
    return ANI_OK;
}

/**
 * walkAndProcess
 *
 * FUNCTION:
 * Iterates over all associations in the hashtable and invokes the
 * callback on the value stored in each of them. Additionally, the op
 * can request that the association be cleared after the callback
 * returns. 
 *
 * @param table the hashtable to walk
 * @param callback the callback to invoke for each value stored in the
 * associations
 * @param op any additional post-processing to perform on the
 * association 
 *
 * @return ANI_OK if the operation succeeds
 */
static int
walkAndProcess(tAniHashTable *table, tAniAsfWalk callback, u_char op)
{
    tEntry *node;
    tEntry *tmp;
    unsigned int i;
    int head;

    for (i = 0; i < table->size; i++) {
        head = 1;
        node = table->buckets + i;
        // If this bucket is empty, go to next bucket
        if (node->key == NULL)
            continue;
        // Iterate thru entire linked list of the bucket
        while (node) {
            if (callback != NULL)
                callback(node->value);
            tmp = node;
            node = node->next;
            if (op == DELETE_NODE) {
                if (!head) {
                    free(tmp);
                } else {
                    head = 0;
                }
            }
        }
    }

    return ANI_OK;
}

/**
 * searchList
 *
 * FUNCTION:
 * Searches thru a bucket in the hashtable to find the node containing
 * the given key, and the node previous to that in the bucket's linked
 * list.
 *
 * ASSUMPTIONS:
 * The linked list forming the bucket is an ordered list in increasing
 * order of key value. The node that is the bucketHead also doubles
 * as a node in the linked list that stores valid data. If the bucket
 * is empty, then bucketHead->key and bucketHead->value are NULL.
 *
 * @param bucketHead the start of the bucket's linked list
 * @param key the key that needs to be looked up in the bucket linked
 * list
 * @param cmp the comparison function to use on the keys
 * @param nodeP if the key is found in the linked list then this
 * pointer is set to the node containing the key; if the key is not
 * found, then the pointer is set to NULL.
 * @param prevP if the key is found, then this pointer is set to the
 * node previous to the node containing the key. If the key is
 * not found, then this pointer is set to the node in the sorted list
 * which would precede a new node containing the key. If the
 * correct position for the key is the head of the bucket, then this
 * pointer is set to NULL.
 *
 * @return ANI_OK if the key was found; ANI_E_FAILED if the key is
 * not found.
 */
static int
searchList(tEntry *bucketHead, 
           void *key,
           tAniAsfCompare cmp, 
           tEntry **nodeP,
           tEntry **prevP)
{
    int tmp;
    tEntry *prevNode;
    tEntry *nextNode;

    if (bucketHead->key == NULL) {
        *nodeP = NULL;
        *prevP = NULL;
        return ANI_E_FAILED;
    }

    /*
     * There is atleast one entry in the bucket. Find the first
     * entry in the bucket whose key is the same as or greater than
     * this key.
     */
    tmp = -1;
    prevNode = nextNode = bucketHead;
    while (nextNode != NULL) {
        tmp = cmp(nextNode->key, key);
        if (tmp >= 0)
            break; // Found something that is >= key
        prevNode = nextNode;
        nextNode = nextNode->next;
    }

    // If we found an exact match, return that node
    if (tmp == 0) {
        *nodeP = nextNode;
        // If the key is at the head of the bucket, NULL out prevP
        // because there is no preceeding node in the lined list
        if (nextNode == bucketHead)
            *prevP = NULL;
        else
            *prevP = prevNode;
        return ANI_OK;
    }

    // We didn't find the key in any node
    *nodeP = NULL;

    // Send back the location of the node that would preceed the key
    if (nextNode == bucketHead) {
        // The key would be first in the linked list, so there is no
        // preceeding node
        *prevP = NULL;
    } else {
        *prevP = prevNode;
    }

    return ANI_E_FAILED;
}

/**
 * setValuesInIterator
 *
 * FUNCTION:
 * Iterates over all associations in the hashtable stores references
 * to the current entries in the iterator array being passed in.
 *
 * @param table the hashtable to walk
 * @param itor the iterator to populate with the current entries
 * @param arraySize the number of elements in the array
 *
 * @return a non-negative number of elements stored if the operation
 * succeeds, or ANI_E_FAILED if not
 */
static int
setValuesInIterator(tAniHashTable *table, tAniAsfIterator *itor)
{
    tEntry *node;
    int count;
    unsigned int bucketNum;

    for (bucketNum = 0, count = 0; 
         (bucketNum < table->size) && (count < itor->numValues); 
         bucketNum++) {

        node = table->buckets + bucketNum;

        // If this bucket is empty, go to next bucket
        if (node->key == NULL)
            continue;

        // Iterate thru entire linked list of the bucket
        while (node && (count < itor->numValues)) {
            itor->values[count++] = node->value;
            node = node->next;
        }
    }

    assert(count == table->numEntries);

    return ANI_OK;
}

#ifdef ANI_DEBUG

int
aniAsfHashTablePrint(tAniHashTable *table, tAniAsfWalk callback)
{
    tEntry *node;
    tEntry *tmp;
    int i;
    int head;
    int retVal = ANI_OK;

    ASSERT_NON_NULL(table);
    ASSERT_NON_NULL(callback);

    printf("------------------------------\n");
    printf("Printing hashtable of size %d", table->size);
    for (i = 0; i < table->size; i++) {
        // printf("\n%d: ", i);
        head = 1;
        node = table->buckets + i;
        // If this bucket is empty, go to next bucket
        if (node->key == NULL)
            continue;
        printf("\n%d: ", i);
        // Iterate thru entire linked list of the bucket
        while (node) {
            callback(node->value);
            tmp = node;
            node = node->next;
            if (!head) {
                // printf(" ");
            } else {
                // printf("--> ");
                head = 0;
            }
        }
    }

    printf("\n------------------------------\n");

    return retVal;
}

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
aniAsfHashString(u_char *str, u_int tableSize)
{
    u_long sum;

    sum = 0;
    while (*str != '\0') {
        sum += *str;
        str++;
    }

    return (sum % tableSize);
}

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
aniAsfHashMacAddr(tAniMacAddr macAddr, u_int tableSize)
{
    return ((macAddr[3] + macAddr[4] + macAddr[5]) % tableSize);
}

/**
 * Utility function to use as string comparison callback for hashtable
 * containing strings. // TBD: Make this an alias for strcmp
 */
int
aniAsfHashCmpString(u_char *str1, u_char *str2)
{
    return strcmp((const char *)str1, (const char *)str2);
}

/**
 * Utility function to do MAC address comparison for hashtables that
 * store MAC addresses.
 */
int
aniAsfHashCmpMacAddr(tAniMacAddr mac1, tAniMacAddr mac2)
{
    u_char a;
    u_char b;

#define SET_BYTES(a, b, i) \
             a = mac1[i]; \
             b = mac2[i]

#define CMP_BYTES(a, b) \
             if (a < b) return -1; \
             else if (a > b) return 1

    // Unroll the loop

    SET_BYTES(a, b, 0);
    CMP_BYTES(a, b);
    else {
        SET_BYTES(a, b, 1);
        CMP_BYTES(a, b);
        else {
            SET_BYTES(a, b, 2);
            CMP_BYTES(a, b);
            else {
                SET_BYTES(a, b, 3);
                CMP_BYTES(a, b);
                else {
                    SET_BYTES(a, b, 4);
                    CMP_BYTES(a, b);
                    else {
                        SET_BYTES(a, b, 5);
                        CMP_BYTES(a, b);
                        else {
                            return 0; // They are equal!
                        }  
                    } // 6th byte
                } // 5th byte
            } // 4th byte
        } // 3rd byte
    } // 2nd byte

    // We never reach here!
    assert(0 && "Code should not be reached!");
    return ANI_OK; // Same as 0
}
