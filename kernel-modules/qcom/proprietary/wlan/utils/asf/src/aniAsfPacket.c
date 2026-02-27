/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * File: $File: //depot/software/projects/feature_branches/nova_phase1/ap/apps/asf/aniAsfPacket.c $
 * Contains definitions for packet manipulation routines that make it
 * easy to create and parse multi-layered network frames. This module
 * minimizes buffer copies while adding or removing headers, and
 * adding or removing payload.
 *
 * Author:      Mayank D. Upadhyay
 * Date:        19-June-2002
 * History:-
 * Date         Modified by     Modification Information
 * ------------------------------------------------------
 *
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#ifdef ANI_KERNEL_2_6
#include <asm/types.h>
#define _LINUX_TYPES_H
#define __KERNEL_STRICT_NAMES
typedef __u16 __le16;
typedef __u16 __be16;
typedef __u32  __le32;
typedef __u32  __be32;
typedef __u16  __sum16;
typedef __u32  __wsum;
#endif
#include <linux/if_ether.h>

#include <aniAsfPacket.h>
#include <aniUtils.h>
#include <aniAsfTtlv.h>
#include <aniErrors.h>
#include <aniAsfLog.h>

/*
 * Allocate one more than required because the last bytes is waste. We
 * waste the last byte because in the adopted model, the tail always
 * points to the next location where data should be stored. In a full
 * buffer we don't want to position the tail to memory we haven't
 * allocated ourself.
 */
#define ANI_INTERNAL_DEFAULT_PACKET_SIZE (ANI_DEFAULT_PACKET_SIZE + 4)

#define TAIL_SPACE(packet) \
    ((packet)->buf + (packet)->size - (packet)->tail)

#define HEAD_SPACE(packet) \
     ((packet)->head - (packet)->buf)

/**
 * Opaque packet structure with internal storage for raw bytes.
 * Conceptually, a tAniPacket is a pre-allocated buffer that contains
 * data in the middle and free space on either side. The start of the
 * data is called the head. Routines are provided to add data at the
 * front or at the rear. The length of the packet is the total number
 * of valid data bytes contained in it. The size of the packet is the
 * total number of preallocated bytes.
 */
struct tAniPacket {
    u_char *buf;
    u_int size;
    u_char *head;
    u_char *tail;
    u_char *recordHeader;
    u_int len;
};

/**
 * aniAsfPacketAllocate
 *
 * FUNCTION:
 * Create a packet of size 2*ANI_DEFAULT_PACKET_SIZE and positions the
 * head of the packet in the center. The allocated storage can be free
 * with a call to aniAsfPacketFree.
 *
 * LOGIC:
 * Allocates storage for tAniPacket and its internal raw data
 * buffer. Positions the head and tail pointers in the middle of the
 * raw data buffer.
 *
 * @param packetPtr pointer that will be set to newly allocated
 * tAniPacket if the operation succeeds.
 *
 * @return ANI_OK if the operation succeeds; ANI_E_MALLOC_FAILED if
 * memory could not be allocated.
 * @see aniAsfPacketFree
 */
int
aniAsfPacketAllocate(tAniPacket **packetPtr)
{
  return aniAsfPacketAllocateExplicit(packetPtr,
                                   ANI_INTERNAL_DEFAULT_PACKET_SIZE,
                                   ANI_INTERNAL_DEFAULT_PACKET_SIZE/2);
}

/**
 * aniAsfPacketAllocateExplicit
 *
 * FUNCTION:
 * Create a packet of the desired size and position the head of the
 * packet at the desired offset in the internal raw data buffer. An
 * application would normally set this offset to the expected length
 * of the protocol header, then append the payload, and finally,
 * prepend the header. The allocated storage can be free with a call
 * to aniAsfPacketFree.
 *
 * LOGIC:
 * Allocates storage for tAniPacket and its internal raw data
 * buffer. Positions the head and tail pointers at the given offset in
 * the internal raw data buffer.
 *
 * @param packetPtr pointer that will be set to newly allocated
 * tAniPacket if the operation succeeds.
 * @param size the size of the internal raw data buffer
 * @param offset the offset in the internal raw data buffer where the
 * head of the packet will be positioned initially
 *
 * @return ANI_OK if the operation succeeds; ANI_E_MALLOC_FAILED if
 * memory could not be allocated.
 * @see aniAsfPacketFree
 */
int
aniAsfPacketAllocateExplicit(tAniPacket **packetPtr,
                             u_int size,
                             u_int offset)
{
  tAniPacket *packet = NULL;
  u_int maxHead = size;

  *packetPtr = NULL;
  assert(size > 0);
  if (size <= 0)
    return ANI_E_ILLEGAL_ARG;

  assert(ANI_CHECK_RANGE(0, offset, maxHead));
  if (!ANI_CHECK_RANGE(0, offset, maxHead))
    return ANI_E_ILLEGAL_ARG;

  packet = (tAniPacket *) calloc(sizeof(tAniPacket), 1);

  if (packet == NULL) {
      aniAsfLogMsg(LOG_CRIT,
                   ANI_LOG_FUNC,
                   "malloc of packet failed!");
      assert(0 && "Failed to allocate packet!");
      return ANI_E_MALLOC_FAILED;
  }

  // transparently add one to the size since last byte is wasted
  size = (size + 4) & 0xfffffffc;

  packet->buf = (u_char *) calloc(sizeof(u_char), size);
  if (packet->buf == NULL) {
      aniAsfLogMsg(LOG_CRIT,
                   ANI_LOG_FUNC,
                   "malloc of packet buf failed!");
      free(packet);
      assert(0 && "Failed to allocate packet buffer!");
      return ANI_E_MALLOC_FAILED;
  }

  packet->size = size; // Should not be visible to the user
  packet->head = packet->buf + offset;
  packet->tail = packet->head;
  packet->len = 0;

  *packetPtr = packet;
  return ANI_OK;
}

/**
 * aniAsfPacketDuplicate
 *
 * Duplicates a given packet exactly. That is, the contents, the size
 * of the packet, and the positions of the pointers are maintained in
 * the new copy.
 *
 * @param newPacketPtr is set to a newly allocated packet that is a
 * duplicate of oldPacket
 * @param oldPacket the original packet that should be duplicated
 *
 * @return ANI_OK if the operation succeeds; ANI_E_NULL if oldPacket
 * is NULL; 
 */
int
aniAsfPacketDuplicate(tAniPacket **newPacketPtr, tAniPacket *oldPacket)
{
    int retVal;
    int recordPos;
    tAniPacket *packet = NULL;

    if (oldPacket == NULL)
        return ANI_E_NULL_VALUE;

    retVal = aniAsfPacketAllocateExplicit(&packet,
                                          oldPacket->size,
                                          oldPacket->head - oldPacket->buf);
    if (retVal != ANI_OK)
        return retVal;

    retVal = aniAsfPacketAppendBuffer(packet,
                                      oldPacket->head,
                                      oldPacket->len);
    if (retVal != ANI_OK) {
        assert(0 && "Could not duplicate packet correctly!");
        aniAsfPacketFree(packet);
        return ANI_E_FAILED;
    }

    if (oldPacket->recordHeader != NULL) {
        recordPos = oldPacket->recordHeader - oldPacket->buf;
        packet->recordHeader = packet->buf + recordPos;
    }
    *newPacketPtr = packet;

    return ANI_OK;
}

/**
 * aniAsfPacketFree
 *
 * FUNCTION:
 * Free a previously allocated tAniPacket and its internal raw data
 * buffer.
 *
 * @param packet the packet to free
 *
 * @return ANI_OK if the operation succeeds; ANI_E_NULL_VALUE if an
 * unexpected NULL pointer is encountered
 */
int
aniAsfPacketFree(tAniPacket *packet)
{
  assert(packet != NULL);
  if (packet == NULL)
    return ANI_E_NULL_VALUE;

  assert(packet->buf != NULL);
  if (packet->buf != NULL)
    free(packet->buf);

  free(packet);

  return ANI_OK;
}

/**
 * aniAsfPacket2Str
 *
 * FUNCTION:
 * Returns a printable representation of the data contained in the
 * packet. 
 * Note: This function returns a static buffer used by aniAsfHexStr.
 *
 * @param packet the packet whose contents need to be printed. If the
 * packet is NULL, or its length is 0, an empty string is returned.
 */
u_char
*aniAsfPacket2Str(tAniPacket *packet)
{
    ANI_U8 *ptr;
    u_int len;

    if (packet != NULL) {
        ptr = packet->head;
        len = packet->len;
    } else {
        ptr = NULL;
        len = 0;
    }

    return aniAsfHexStr(ptr, len);
}

/**
 * aniAsfPacketAppendBuffer
 *
 * FUNCTION:
 * Appends the data contained in buf to the end of the data in
 * destAniPacket. The head of destAniPacket remains unchanged, while its
 * length increases by len.
 *
 * If there isn't enough free space in destAniPacket for all len bytes
 * then the routine fails and the length of destAniPacket remains
 * unchanged.
 *
 * LOGIC:
 * Check that there is enough free space in the packet to append the
 * buffer. If not, bail. Otherwise, copy bytes from the buffer into
 * the packet's internal raw data buffer and increase the value of its
 * length to reflect this.
 *
 * @param packet the packet to append to
 * @param buf the buffer containing data to be appended to the packet
 * @param len the number of bytes to append
 *
 * @return ANI_OK if the operation succeeds; ANI_E_FAILED if the
 * packet does not have enough free space for the complete buffer
 * @see aniAsfPacketPrependBuffer
 */
int
aniAsfPacketAppendBuffer(tAniPacket *destPacket,
                      const u_char *buf,
                      u_int len)
{
  if (aniAsfPacketCanAppendBuffer(destPacket, len) != ANI_OK)
      return ANI_E_FAILED;

  assert(buf != NULL);
  if (buf == NULL)
    return ANI_E_NULL_VALUE;

  memcpy(destPacket->tail, buf, len);
  destPacket->tail += len;
  destPacket->len += len;
  return ANI_OK;
}

/**
 * aniAsfPacketPrependBuffer
 *
 * FUNCTION:
 * Prepends the data contained in buf to the start of the data in
 * destPacket. The head of destPacket is repositioned and the length
 * of destPacket increases by len.
 *
 * If there isn't enough free space in destPacket for all len bytes
 * then the routine fails and the length of destPacket remains
 * unchanged.
 *
 * LOGIC:
 * Check that there is enough free space in the packet to prepend the
 * buffer. If not, bail. Otherwise, copy bytes from the buffer into
 * the packet's internal raw data buffer and increase the value of its
 * length to reflect this.
 *
 * @param packet the packet to prepend to
 * @param buf the buffer containing data to be prepended to the packet
 * @param len the number of bytes to prepend
 *
 * @return ANI_OK if the operation succeeds; ANI_E_FAILED if the
 * packet does not have enough free space for the complete buffer
 * @see aniAsfPacketAppendBuffer
 */
int
aniAsfPacketPrependBuffer(tAniPacket *destPacket,
                       const u_char *buf,
                       u_int len)
{
  if (aniAsfPacketCanPrependBuffer(destPacket, len) != ANI_OK)
      return ANI_E_FAILED;

  assert(buf != NULL);
  if (buf == NULL)
      return ANI_E_NULL_VALUE;

  destPacket->head -= len;
  destPacket->len += len;
  memcpy(destPacket->head, buf, len);
  return ANI_OK;

}

/**
 * aniAsfPacketCanAppendBuffer
 *
 * FUNCTION:
 * Determines if len bytes can be safely appended to destPacket
 * without overflowing.
 *
 * LOGIC:
 * Current packet tail plus len of buffer should not exceed packet
 * start plus packet size
 *
 * Note: This does not return a boolean value, but instead an integer
 * code.
 *
 * @param packet the packet to append to
 * @param len the number of bytes to append
 *
 * @return ANI_OK if the append operation would succeed; ANI_E_FAILED
 * otherwise
 */
int
aniAsfPacketCanAppendBuffer(tAniPacket *destPacket,
                         u_int len)
{
  assert(destPacket != NULL);
  if (destPacket == NULL)
    return ANI_E_FAILED;

  assert(len >= 0);

  if ((int)len <= TAIL_SPACE(destPacket))
      return ANI_OK;
  else
      return ANI_E_FAILED;
}

/**
 * aniAsfPacketCanPrependBuffer
 *
 * FUNCTION:
 * Determines if len bytes can be safely prepended to destPacket
 * without overflowing.
 *
 * LOGIC:
 * Current packet head minus len of buffer should not be less than
 * start of packet.
 *
 * Note: This does not return a boolean value, but instead an integer
 * code.
 *
 * @param packet the packet to prepend to
 * @param len the number of bytes to prepend
 *
 * @return ANI_OK if the append operation would succeed; ANI_E_FAILED
 * otherwise
 */
int
aniAsfPacketCanPrependBuffer(tAniPacket *destPacket,
                          u_int len)
{
  assert(destPacket != NULL);
  if (destPacket == NULL)
      return ANI_E_FAILED;

  assert(len >= 0);

  if ((int)len <= HEAD_SPACE(destPacket))
      return ANI_OK;
  else
      return ANI_E_FAILED;
}

/**
 * aniAsfPacketTruncateFromFront
 *
 * FUNCTION:
 * Removes len bytes from the front of the packet by moving its
 * head. The length of the packet is decremented by len.
 *
 * @param packet the packet to truncate from the front
 * @param len the number of bytes to truncate
 *
 * @return ANI_OK if the append operation would succeed; ANI_E_FAILED
 * otherwise
 */
int
aniAsfPacketTruncateFromFront(tAniPacket *packet,
                           u_int len)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    if (!ANI_CHECK_RANGE(0, len, packet->len))
        return ANI_E_FAILED;

    packet->head += len;
    packet->len -= len;

    return ANI_OK;
}

/**
 * aniAsfPacketTruncateFromRear
 *
 * FUNCTION:
 * Removes len bytes from the rear of the packet by moving its
 * tail. The length of the packet is decremented by len.
 *
 * @param packet the packet to truncate from the rear
 * @param len the number of bytes to truncate
 *
 * @return ANI_OK if the append operation would succeed; ANI_E_FAILED
 * otherwise
 */
int
aniAsfPacketTruncateFromRear(tAniPacket *packet,
                          u_int len)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    if (!ANI_CHECK_RANGE(0, len, packet->len))
        return ANI_E_FAILED;

    packet->tail -= len;
    packet->len -= len;

    return ANI_OK;
}

/**
 * aniAsfPacketGetLen
 *
 * FUNCTION:
 * Returns the number of valid data bytes stored in the packet.
 *
 * @param packet the packet whose len we need
 *
 * @return the non-negative number of bytes stored in the packet
 */
int
aniAsfPacketGetLen(tAniPacket *packet)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    return packet->len;
}

/**
 * aniAsfPacketGetBytes
 *
 * FUNCTION:
 * Returns a pointer to the head of the valid data stored in the
 * packet.
 *
 * @param packet the packet whose bytes we need
 * @param rawBytesPtr the pointer that will be set the start of the
 * raw bytes.
 *
 * @return The non-negative number of bytes stored in the packet if
 * the operation succeeded. That is the same value as what would be
 * returned by aniAsfPacketGetLen.
 */
int
aniAsfPacketGetBytes(tAniPacket *packet, u_char **rawBytesPtr)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    *rawBytesPtr = packet->head;
    return packet->len;
}

/**
 * aniAsfPacketGetN
 *
 * Returns N bytes from the packet and moves the head of the packet
 * beyond those bytes.
 *
 * @param packet the packet to read from
 * @param n the number of bytes to read
 * @param bytesPtr is set to the start of the octets
 *
 * @return ANI_OK if the operation succeeds; ANI_E_SHORT_PACKET if the
 * packet does not have n bytes.
 */
inline int
aniAsfPacketGetN(tAniPacket *packet, int n, ANI_U8 **bytesPtr)
{
    int retVal;
    ANI_U8 *bytes = NULL;

    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    retVal = aniAsfPacketGetBytes(packet, &bytes);
    if (retVal < n)
        return ANI_E_SHORT_PACKET;

    aniAsfPacketTruncateFromFront(packet, n);

    *bytesPtr = bytes;

    return ANI_OK;
}

/**
 * aniAsfPacketEmpty
 *
 * FUNCTION:
 * Re-initializes the packet by positioning the head to the middle and
 * setting the length to zero.
 *
 * @param packet the packet to empty
 *
 * @return ANI_OK if the operation succeeded
 */
int
aniAsfPacketEmpty(tAniPacket *packet)
{
    return aniAsfPacketEmptyExplicit(packet, packet->size/2);
}

/**
 * aniAsfPacketEmptyExplicit
 *
 * FUNCTION:
 * Re-initializes the packet by positioning the head to the desired
 * offset and setting the length to zero.
 *
 * @param packet the packet to empty
 * @param offset the offset that the head of the packet should be set
 * to. An application will be able to prepend and append data relative
 * to this offset.
 *
 * @return ANI_OK if the operation succeeded
 */
int
aniAsfPacketEmptyExplicit(tAniPacket *packet,
                       u_int offset)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(ANI_CHECK_RANGE(0, offset, packet->size));
    if (!ANI_CHECK_RANGE(0, offset, packet->size))
        return ANI_E_ILLEGAL_ARG;

    packet->head = packet->buf + offset;
    packet->tail = packet->head;
    packet->len = 0;

    return ANI_OK;
}


/**
 * aniAsfPacketRead
 *
 * FUNCTION:
 * Reads a message from an IPC object directly into the packet. If the
 * message contents are too large to fit in the packet, the remaining
 * bytes may be lost depending on the type of underlying FD. Also,
 * depending on the type of underlying FD, this call may block.
 *
 * @param packet the packet to read into
 * @param ipcConn the IPC obejct to read from
 *
 * @return the non-negative number of bytes that were read if the
 * operation succeeded
 */
int
aniAsfPacketRead(tAniPacket *packet, tAniIpc *ipcConn)
{
    int len;

    len = TAIL_SPACE(packet);
    len = aniAsfIpcRecv(ipcConn, (char *)packet->tail, len);
    if (len > 0) {
        packet->len += len;
        packet->tail += len;
    }
    return len;
}

/**
 * aniAsfPacketGetHdr
 *
 * FUNCTION:
 * Reads a tAniHdr out of the packet and returns it. The packet's head
 * is advanced and its length decremented by the appropriate
 * length. All network to host byte order translation is also taken
 * care of.
 *
 * @param packet the packet to read from
 * @param hdr the tAniHdr structure to fill
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketGetHdr(tAniPacket *packet, tAniHdr *hdr)
{
    int retVal = ANI_OK;

    retVal = aniAsfPacketPeekHdr(packet, hdr);
    if (retVal == ANI_OK)
        aniAsfPacketTruncateFromFront(packet, sizeof(tAniHdr));

    return retVal;
}

/**
 * aniAsfPacketPeekHdr
 *
 * FUNCTION:
 * Reads a tAniHdr out of the packet and returns it. The packet's head
 * and its length are unchanged.  All network to host byte order
 * translation is also taken care of.
 *
 * @param packet the packet to read from
 * @param hdr the tAniHdr structure to fill
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketPeekHdr(tAniPacket *packet, tAniHdr *hdr)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(hdr != NULL);
    if (hdr == NULL)
        return ANI_E_NULL_VALUE;

    if (packet->len < sizeof(tAniHdr)) {
        return ANI_E_SHORT_PACKET;
    }

    memcpy(hdr, packet->head, sizeof(tAniHdr));
    aniAsfNtohHdr((char *)hdr);

    if (hdr->length > packet->len)
        return ANI_E_SHORT_PACKET;

    return ANI_OK;
} /* end aniAsfPacketPeekHdr */

/**
 * aniAsfPacketPrependHdr
 *
 * FUNCTION:
 * Prepends a tAniHdr at the start of the packet.  All host to network
 * byte order translation is also taken care of.
 *
 * @param packet the packet to write to
 * @param msgType the message type to write as part of the header
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketPrependHdr(tAniPacket *packet, ANI_U16 msgType)
{
    int retVal;
    int length;

    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    length = sizeof(tAniHdr);
    assert(length >= 4);
    if (length < 4)
        return ANI_E_FAILED; // Hdr definition changed!!!

    length = 2 + 2 + packet->len;

    retVal = aniAsfPacketPrepend16(packet, length);
    if (retVal < 0)
        return retVal;

    retVal = aniAsfPacketPrepend16(packet, msgType);
    if (retVal < 0)
        return retVal;

    return ANI_OK;
}

/**
 * aniAsfPacketGet32
 *
 * FUNCTION:
 * Reads a ANI_U32 out of the packet and returns it. The packet's head
 * is advanced and its length decremented by the appropriate length.
 * All network to host byte order translation is also taken care of.
 *
 * @param packet the packet to read from
 * @param val the value to fill in
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketGet32(tAniPacket *packet, ANI_U32 *val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(val != NULL);
    if (val == NULL)
        return ANI_E_NULL_VALUE;

    if (packet->len < 4)
        return ANI_E_SHORT_PACKET;

    *val = aniAsfGet32((char *)packet->head);
    aniAsfPacketTruncateFromFront(packet, 4);

    return ANI_OK;
}

/**
 * aniAsfPacketAppend32
 *
 * FUNCTION:
 * Appends a ANI_U32 to the end of the packet.
 * All host to network byte order translation is also taken care of.
 *
 * @param packet the packet to write to
 * @param val the value to append
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketAppend32(tAniPacket *packet, ANI_U32 val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    if (TAIL_SPACE(packet) < 4)
        return ANI_E_FAILED;

    aniAsfPut32((char *)packet->tail, val);
    aniAsfPacketMoveRight(packet, 4);

    return ANI_OK;
}

/**
 * aniAsfPacketGet16
 *
 * FUNCTION:
 * Reads a ANI_U16 out of the packet and returns it. The packet's head
 * is advanced and its length decremented by the appropriate length.
 * All network to host byte order translation is also taken care of.
 *
 * @param packet the packet to read from
 * @param val the value to fill in
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketGet16(tAniPacket *packet, ANI_U16 *val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(val != NULL);
    if (val == NULL)
        return ANI_E_NULL_VALUE;

    if (packet->len < 2)
        return ANI_E_SHORT_PACKET;

    *val = aniAsfGet16((char *)packet->head);
    aniAsfPacketTruncateFromFront(packet, 2);

    return ANI_OK;
}

/**
 * aniAsfPacketPrepend16
 *
 * FUNCTION:
 * Prepends a ANI_U16 to the start of the packet.
 * All host to network byte order translation is also taken care of.
 *
 * @param packet the packet to write to
 * @param val the value to prepend
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketPrepend16(tAniPacket *packet, ANI_U16 val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(HEAD_SPACE(packet) >= 2);
    if (HEAD_SPACE(packet) < 2)
        return ANI_E_FAILED;

    aniAsfPacketMoveLeft(packet, 2);
    aniAsfPut16((char *)packet->head, val);

    return ANI_OK;
}

/**
 * aniAsfPacketAppend16
 *
 * FUNCTION:
 * Appends a ANI_U16 to the end of the packet.
 * All host to network byte order translation is also taken care of.
 *
 * @param packet the packet to write to
 * @param val the value to append
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketAppend16(tAniPacket *packet, ANI_U16 val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    if (TAIL_SPACE(packet) < 2)
        return ANI_E_FAILED;

    aniAsfPut16((char *)packet->tail, val);
    aniAsfPacketMoveRight(packet, 2);

    return ANI_OK;
}

/**
 * aniAsfPacketGet8
 *
 * FUNCTION:
 * Reads a ANI_U8 out of the packet and returns it. The packet's head
 * is advanced and its length decremented by the appropriate length.
 * All network to host byte order translation is also taken care of.
 *
 * @param packet the packet to read from
 * @param val the value to fill in
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketGet8(tAniPacket *packet, ANI_U8 *val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(val != NULL);
    if (val == NULL)
        return ANI_E_NULL_VALUE;

    if (packet->len < 1)
        return ANI_E_SHORT_PACKET;

    *val = *(packet->head);
    aniAsfPacketTruncateFromFront(packet, 1);

    return ANI_OK;
}

/**
 * aniAsfPacketPrepend8
 *
 * FUNCTION:
 * Prepends a ANI_U8 to the start of the packet.
 * All host to network byte order translation is also taken care of.
 *
 * @param packet the packet to read from
 * @param val the value to prepend
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketPrepend8(tAniPacket *packet, ANI_U8 val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    assert(HEAD_SPACE(packet) >= 1);
    if (HEAD_SPACE(packet) < 1)
        return ANI_E_FAILED;

    aniAsfPacketMoveLeft(packet, 1);
    *(packet->head) = val;

    return ANI_OK;
}

/**
 * aniAsfPacketAppend8
 *
 * FUNCTION:
 * Appends a ANI_U8 to the end of the packet.
 * All host to network byte order translation is also taken care of.
 *
 * @param packet the packet to write to
 * @param val the value to append
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketAppend8(tAniPacket *packet, ANI_U8 val)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    if (TAIL_SPACE(packet) < 1)
        return ANI_E_FAILED;

    *(packet->tail) = val;
    aniAsfPacketMoveRight(packet, 1);

    return ANI_OK;
}

/**
 * aniAsfPacketGetMac
 *
 * FUNCTION:
 * Returns a tAniMacAddr from the start of the packet.
 *
 * @param packet the packet to read from
 * @param macAddr the destination to copy the MAC address to
 *
 * @return ANI_OK if the operation succeeds. Also, the packet head
 * pointer is advanced past the MAC address.
 */
inline int
aniAsfPacketGetMac(tAniPacket *packet, tAniMacAddr macAddr)
{
    assert(packet != NULL);

    if (packet->len < sizeof(tAniMacAddr))
        return ANI_E_SHORT_PACKET;

    memcpy(macAddr, packet->head, sizeof(tAniMacAddr));

    packet->head += sizeof(tAniMacAddr);
    packet->len -= sizeof(tAniMacAddr);

    return ANI_OK;
}

/**
 * aniAsfPacketMoveLeft
 *
 * FUNCTION:
 * Pretends that a certain number of bytes have been prepended to the
 * packet, without actually copying any bytes in. The packet head and
 * length are appropriately changed. This function is useful while
 * interfacing with other libraries that only support byte array
 * manipulation.
 *
 * WARNING: 
 * Applications are discouraged from using this function
 * because correct usage is a two-step process - one: copy some bytes
 * to the packet's internal buffer, two: move head and length. This
 * violates the encapsulation the packet library aims to provide.
 *
 * @param packet the packet whose head and length needs to be modified
 * @param count the number of bytes to modify by
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketMoveLeft(tAniPacket *packet, u_int count)
{
    if (aniAsfPacketCanPrependBuffer(packet, count) != ANI_OK)
        return ANI_E_FAILED;

    packet->head -= count;
    packet->len += count;

    return ANI_OK;
}

/**
 * aniAsfPacketMoveRight
 *
 * FUNCTION:
 * Pretends that a certain number of bytes have been appended to the
 * packet, without actually copying any bytes in. The packet tail and
 * length are appropriately changed. This function is useful while
 * interfacing with other libraries that only support byte array
 * manipulation.
 *
 * WARNING: 
 * Applications are discouraged from using this function
 * because correct usage is a two-step process - one: copy some bytes
 * to the packet's internal buffer, two: move tail and length. This
 * violates the encapsulation the packet library aims to provide.
 *
 * @param packet the packet whose head and length needs to be modified
 * @param count the number of bytes to modify by
 *
 * @return ANI_OK if the operation succeeds
 */
int
aniAsfPacketMoveRight(tAniPacket *packet, u_int count)
{
    if (aniAsfPacketCanAppendBuffer(packet, count) != ANI_OK)
        return ANI_E_FAILED;

    packet->tail += count;
    packet->len += count;

    return ANI_OK;
}

/**
 * aniAsfPacketGetBytesFromTail
 *
 * FUNCTION:
 * Returns a pointer to the tail of the valid data stored 
 * in the packet.
 *
 * WARNING: 
 * Applications are discouraged from using this function
 * because correct usage is a three-step process - one: call this
 * routine to obtain a pointer to the current tail of the packet. 
 * two: treat this returned pointer like a simple array and copy 
 * some bytes to the packet's internal buffer, and finally 
 * three: move tail and length. This violates the encapsulation 
 * the packet library aims to provide.
 *
 * @param packet the packet whose bytes we need
 * @param rawBytesPtr the pointer that will be set the start of the
 * raw bytes.
 *
 * @return The non-negative number of bytes stored in the packet if
 * the operation succeeded. That is the same value as what would be
 * returned by aniAsfPacketGetLen.
 */
int
aniAsfPacketGetBytesFromTail(tAniPacket *packet, u_char **rawBytesPtr)
{
    assert(packet != NULL);
    if (packet == NULL)
        return ANI_E_NULL_VALUE;

    *rawBytesPtr = packet->tail;
    return 0; // The length of used bytes returned is zero
}

