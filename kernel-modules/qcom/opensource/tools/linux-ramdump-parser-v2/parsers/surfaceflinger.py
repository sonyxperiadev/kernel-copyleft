# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

from parser_util import register_parser, RamParser, cleanupString
from mmu import Armv8MMU
from print_out import print_out_str
import traceback
from utasklib import UTaskLib
from utasklib import ProcessNotFoundExcetion

class PixelFormat:

    PIXEL_FORMAT_UNKNOWN = 0,
    PIXEL_FORMAT_NONE = 0,
    PIXEL_FORMAT_CUSTOM = -4,
    PIXEL_FORMAT_TRANSLUCENT = -3,
    PIXEL_FORMAT_TRANSPARENT = -2,
    PIXEL_FORMAT_OPAQUE = -1,
    PIXEL_FORMAT_RGBA_8888 = 1  # 4x8-bit RGBA
    PIXEL_FORMAT_RGBX_8888 = 2  # 4x8-bit RGB0
    PIXEL_FORMAT_RGB_888 = 3  # 3x8-bit RGB
    PIXEL_FORMAT_RGB_565 = 4  # 16-bit RGB
    PIXEL_FORMAT_BGRA_8888 = 5  # 4x8-bit BGRA
    PIXEL_FORMAT_RGBA_5551 = 6  # 16-bit ARGB
    PIXEL_FORMAT_RGBA_4444 = 7  # 16-bit ARGB
    PIXEL_FORMAT_RGBA_FP16 = 22  # 64-bit RGBA
    PIXEL_FORMAT_RGBA_1010102 = 43  # 32-bit RGBA
    PIXEL_FORMAT_R_8 = 0x38

    HAL_PIXEL_FORMAT_ARGB_2101010 = 0x117
    HAL_PIXEL_FORMAT_RGBX_1010102 = 0x118
    HAL_PIXEL_FORMAT_XRGB_2101010 = 0x119
    HAL_PIXEL_FORMAT_BGRA_1010102 = 0x11A
    HAL_PIXEL_FORMAT_ABGR_2101010 = 0x11B
    HAL_PIXEL_FORMAT_BGRX_1010102 = 0x11C
    HAL_PIXEL_FORMAT_XBGR_2101010 = 0x11D
    HAL_PIXEL_FORMAT_YCbCr_420_P010 = 0x36
    HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC = 0x124
    HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS = 0x7FA30C0A
    HAL_PIXEL_FORMAT_NV12_ENCODEABLE = 0x102

    HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS = 0x7FA30C04
    HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED = 0x7FA30C03
    HAL_PIXEL_FORMAT_YCbCr_420_SP = 0x109
    HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO = 0x7FA30C01
    HAL_PIXEL_FORMAT_YCrCb_422_SP = 0x10B
    HAL_PIXEL_FORMAT_R_8 = 0x10D
    HAL_PIXEL_FORMAT_RG_88 = 0x10E
    HAL_PIXEL_FORMAT_YCbCr_444_SP = 0x10F
    HAL_PIXEL_FORMAT_YCrCb_444_SP = 0x110
    HAL_PIXEL_FORMAT_YCrCb_422_I = 0x111
    HAL_PIXEL_FORMAT_BGRX_8888 = 0x112
    HAL_PIXEL_FORMAT_NV21_ZSL = 0x113
    HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS = 0x114
    HAL_PIXEL_FORMAT_BGR_565 = 0x115
    HAL_PIXEL_FORMAT_RAW8 = 0x123
    HAL_PIXEL_FORMAT_NV12_HEIF = 0x116
    HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC = 0x7FA30C06
    HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC = 0x7FA30C09
    # Camera utils format
    HAL_PIXEL_FORMAT_CbYCrY_422_I = 0x120
    HAL_PIXEL_FORMAT_BGR_888 = 0x121
    HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX = 0x125
    HAL_PIXEL_FORMAT_NV12_UBWC_FLEX = 0x126
    HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX = 0x127
    HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH = 0x128
    HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH = 0x129
    HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH = 0x130

    HAL_PIXEL_FORMAT_RGBA_5551 = 6
    HAL_PIXEL_FORMAT_RGBA_4444 = 7
    # Khronos ASTC formats
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR = 0x93B0
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR = 0x93B1
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR = 0x93B2
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR = 0x93B3
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR = 0x93B4
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR = 0x93B5
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR = 0x93B6
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR = 0x93B7
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR = 0x93B8
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR = 0x93B9
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR = 0x93BA
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR = 0x93BB
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR = 0x93BC
    HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR = 0x93BD
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR = 0x93D0
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR = 0x93D1
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR = 0x93D2
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR = 0x93D3
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR = 0x93D4
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR = 0x93D5
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR = 0x93D6
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR = 0x93D7
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR = 0x93D8
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR = 0x93D9
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR = 0x93DA
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR = 0x93DB
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR = 0x93DC
    HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR = 0x93DD

    UNSPECIFIED = 0
    RGBA_8888 = 1
    RGBX_8888 = 2
    RGB_888 = 3
    RGB_565 = 4
    BGRA_8888 = 5

    YCBCR_422_SP = 0x10
    YCRCB_420_SP = 0x11
    YCBCR_422_I = 0x14

    RGBA_FP16 = 0x16
    RAW16 = 0x20
    BLOB = 0x21
    IMPLEMENTATION_DEFINED = 0x22
    YCBCR_420_888 = 0x23
    RAW_OPAQUE = 0x24
    RAW10 = 0x25
    RAW12 = 0x26
    RGBA_1010102 = 0x2B
    Y8 = 0x20203859
    Y16 = 0x20363159
    YV12 = 0x32315659
    DEPTH_16 = 0x30
    DEPTH_24 = 0x31
    DEPTH_24_STENCIL_8 = 0x32
    DEPTH_32F = 0x33
    DEPTH_32F_STENCIL_8 = 0x34
    STENCIL_8 = 0x35
    YCBCR_P010 = 0x36
    HSV_888 = 0x37
    R_8 = 0x38

class DrmFormat:
    def fourcc_code(a, b, c, d):
        return ((ord(d)) << 24) | ((ord(c)) << 16) | ((ord(b)) << 8) | (ord(a))

    def fourcc_mod_code(vendor, val):
        return (vendor << 56) | (val & 0x00ffffffffffffff)

    DRM_FORMAT_MOD_VENDOR_QCOM = 0x05
    DRM_FORMAT_ABGR8888 = fourcc_code('A', 'B', '2', '4')
    DRM_FORMAT_ABGR1555 = fourcc_code('A', 'B', '1', '5')
    DRM_FORMAT_ABGR4444 = fourcc_code('A', 'B', '1', '2')
    DRM_FORMAT_ABGR2101010 = fourcc_code('A', 'B', '3', '0')
    DRM_FORMAT_ABGR16161616F = fourcc_code('A', 'B', '4', 'H')
    DRM_FORMAT_ARGB8888 = fourcc_code('A', 'R', '2', '4')
    DRM_FORMAT_XBGR8888 = fourcc_code('X', 'B', '2', '4')
    DRM_FORMAT_XRGB8888 = fourcc_code('X', 'R', '2', '4')
    DRM_FORMAT_BGR888 = fourcc_code('B', 'G', '2', '4')
    DRM_FORMAT_BGR565 = fourcc_code('B', 'G', '1', '6')
    DRM_FORMAT_MOD_QCOM_COMPRESSED = fourcc_mod_code(DRM_FORMAT_MOD_VENDOR_QCOM, 1)

    DRM_FORMAT_BGRA1010102 = fourcc_code('B', 'A', '3', '0')
    DRM_FORMAT_XBGR2101010 = fourcc_code('X', 'B', '3', '0')
    DRM_FORMAT_BGRX1010102 = fourcc_code('B', 'X', '3', '0')
    DRM_FORMAT_ARGB2101010 = fourcc_code('A', 'R', '3', '0')
    DRM_FORMAT_RGBA1010102 = fourcc_code('R', 'A', '3', '0')
    DRM_FORMAT_XRGB2101010 = fourcc_code('X', 'R', '3', '0')
    DRM_FORMAT_RGBX1010102 = fourcc_code('R', 'X', '3', '0')

    DRM_FORMAT_NV12 = fourcc_code('N', 'V', '1', '2')
    DRM_FORMAT_MOD_QCOM_TILE = fourcc_mod_code(DRM_FORMAT_MOD_VENDOR_QCOM, 0x8)
    DRM_FORMAT_MOD_QCOM_DX = fourcc_mod_code(DRM_FORMAT_MOD_VENDOR_QCOM, 0x2)
    DRM_FORMAT_MOD_QCOM_TIGHT = fourcc_mod_code(DRM_FORMAT_MOD_VENDOR_QCOM, 0x4)
    DRM_FORMAT_NV21 = fourcc_code('N', 'V', '2', '1')
    DRM_FORMAT_NV16 = fourcc_code('N', 'V', '1', '6')
    DRM_FORMAT_NV61 = fourcc_code('N', 'V', '6', '1')
    DRM_FORMAT_NV24 = fourcc_code('N', 'V', '2', '4')
    DRM_FORMAT_NV42 = fourcc_code('N', 'V', '4', '2')
    DRM_FORMAT_YVU420 = fourcc_code('Y', 'V', '1', '2')

class ColorPrimaries():
    ColorPrimaries_BT709_5 = 1
    ColorPrimaries_BT470_6M = 4
    ColorPrimaries_BT601_6_625 = 5
    ColorPrimaries_BT601_6_525 = 6
    ColorPrimaries_SMPTE_240M = 7
    ColorPrimaries_GenericFilm = 8
    ColorPrimaries_BT2020 = 9
    ColorPrimaries_SMPTE_ST428 = 10
    ColorPrimaries_AdobeRGB = 11
    ColorPrimaries_DCIP3 = 12
    ColorPrimaries_EBU3213 = 22
    ColorPrimaries_Max = 0xff

class Dataspace():
    UNKNOWN = 0
    STANDARD_MASK = 63 << 16
    STANDARD_UNSPECIFIED = 0 << 16
    STANDARD_BT709 = 1 << 16
    STANDARD_BT601_625 = 2 << 16
    STANDARD_BT601_625_UNADJUSTED = 3 << 16
    STANDARD_BT601_525 = 4 << 16
    STANDARD_BT601_525_UNADJUSTED = 5 << 16
    STANDARD_BT2020 = 6 << 16
    STANDARD_BT2020_CONSTANT_LUMINANCE = 7 << 16
    STANDARD_BT470M = 8 << 16
    STANDARD_FILM = 9 << 16
    STANDARD_DCI_P3 = 10 << 16
    STANDARD_ADOBE_RGB = 11 << 16
    TRANSFER_MASK = 31 << 22
    TRANSFER_UNSPECIFIED = 0 << 22
    TRANSFER_LINEAR = 1 << 22
    TRANSFER_SRGB = 2 << 22
    TRANSFER_SMPTE_170M = 3 << 22
    TRANSFER_GAMMA2_2 = 4 << 22
    TRANSFER_GAMMA2_6 = 5 << 22
    TRANSFER_GAMMA2_8 = 6 << 22
    TRANSFER_ST2084 = 7 << 22
    TRANSFER_HLG = 8 << 22
    RANGE_MASK = 7 << 27
    RANGE_UNSPECIFIED = 0 << 27
    RANGE_FULL = 1 << 27
    RANGE_LIMITED = 2 << 27
    RANGE_EXTENDED = 3 << 27
    SCRGB_LINEAR = 406913024
    SRGB = 142671872
    SCRGB = 411107328
    DISPLAY_P3 = 143261696
    BT2020_PQ = 163971072
    BT2020_ITU_PQ = 298188800
    ADOBE_RGB = 151715840
    JFIF = 146931712
    BT601_625 = 281149440
    BT601_525 = 281280512
    BT2020 = 147193856
    BT709 = 281083904
    DCI_P3 = 155844608
    SRGB_LINEAR = 138477568
    BT2020_HLG = 168165376
    BT2020_ITU_HLG = 302383104
    DEPTH = 4096

class GammaTransfer():
    Transfer_sRGB = 1
    Transfer_Gamma2_2 = 4
    Transfer_Gamma2_8 = 5
    Transfer_SMPTE_170M = 6
    Transfer_SMPTE_240M = 7
    Transfer_Linear = 8
    Transfer_Log = 9
    Transfer_Log_Sqrt = 10
    Transfer_XvYCC = 11
    Transfer_BT1361 = 12
    Transfer_sYCC = 13
    Transfer_BT2020_2_1 = 14
    Transfer_BT2020_2_2 = 15
    Transfer_SMPTE_ST2084 = 16
    Transfer_ST_428 = 17
    Transfer_HLG = 18
    Transfer_Max = 0xff

class ColorRange():
    Range_Limited = 0
    Range_Full = 1
    Range_Extended = 2
    Range_Max = 0xff

class QtigrallocFlags():
    PRIV_FLAGS_USES_ION = 0x00000008
    PRIV_FLAGS_NEEDS_FLUSH = 0x00000020
    PRIV_FLAGS_NON_CPU_WRITER = 0x00000080
    PRIV_FLAGS_CACHED = 0x00000200
    PRIV_FLAGS_SECURE_BUFFER = 0x00000400
    PRIV_FLAGS_VIDEO_ENCODER = 0x00010000
    PRIV_FLAGS_CAMERA_WRITE = 0x00020000
    PRIV_FLAGS_CAMERA_READ = 0x00040000
    PRIV_FLAGS_HW_TEXTURE = 0x00100000
    PRIV_FLAGS_SECURE_DISPLAY = 0x01000000
    PRIV_FLAGS_TILE_RENDERED = 0x02000000
    PRIV_FLAGS_UBWC_ALIGNED = 0x08000000
    PRIV_FLAGS_UBWC_ALIGNED_PI = 0x40000000

class ExtendableType:
    VENDOR_QTI = "QTI"
    GRALLOC4_STANDARD_CHROMA_SITING = "android.hardware.graphics.common.ChromaSiting"

    GRALLOC4_STANDARD_METADATA_TYPE = "android.hardware.graphics.common.StandardMetadataType"
    GRALLOC4_STANDARD_CHROMA_SITING = "android.hardware.graphics.common.ChromaSiting"
    GRALLOC4_STANDARD_COMPRESSION = "android.hardware.graphics.common.Compression"
    GRALLOC4_STANDARD_INTERLACED  = "android.hardware.graphics.common.Interlaced"
    GRALLOC4_STANDARD_PLANE_LAYOUT_COMPONENT_TYPE = "android.hardware.graphics.common.PlaneLayoutComponentType"

    COMPRESSION_QTI_UBWC = 20001
    INTERLACED_QTI = 20002
    NONE = 0

    def __init__(self, name, value):
        self.name = name
        self.value = value

class PrivateHandleT:
    unaligned_width = 0
    unaligned_height = 0
    width = 0
    height = 0
    usage = 0
    format = 0
    flags = 0
    buffer_id = 0
    allocationSizeKiB = 0
    pixelFormatRequested = 0,
    dataspace = 0
    color_primaries = 0
    color_range = 0
    color_transfer = 0
    protectedContent = 0

class PlaneComponent:
    PLANE_COMPONENT_Y = 1 << 0
    PLANE_COMPONENT_Cb = 1 << 1
    PLANE_COMPONENT_Cr = 1 << 2
    PLANE_COMPONENT_R = 1 << 10
    PLANE_COMPONENT_G = 1 << 11
    PLANE_COMPONENT_B = 1 << 12
    PLANE_COMPONENT_A = 1 << 20
    PLANE_COMPONENT_RAW = 1 << 30
    PLANE_COMPONENT_META = 1 << 31


class BufferType:
    BUFFER_TYPE_UI = 0
    BUFFER_TYPE_VIDEO = 1

class BufferUsage:
    CPU_READ_MASK = 0xf
    CPU_READ_NEVER = 0
    CPU_READ_RARELY = 2
    CPU_READ_OFTEN = 3
    CPU_WRITE_MASK = 0xf << 4
    CPU_WRITE_NEVER = 0 << 4
    CPU_WRITE_RARELY = 2 << 4
    CPU_WRITE_OFTEN = 3 << 4
    GPU_TEXTURE = 1 << 8
    GPU_RENDER_TARGET = 1 << 9
    COMPOSER_OVERLAY = 1 << 11
    COMPOSER_CLIENT_TARGET = 1 << 12
    PROTECTED = 1 << 14
    COMPOSER_CURSOR = 1 << 15
    VIDEO_ENCODER = 1 << 16
    CAMERA_OUTPUT = 1 << 17
    CAMERA_INPUT = 1 << 18
    RENDERSCRIPT = 1 << 20
    VIDEO_DECODER = 1 << 22
    SENSOR_DIRECT_DATA = 1 << 23
    GPU_DATA_BUFFER = 1 << 24
    GPU_CUBE_MAP = 1 << 25
    GPU_MIPMAP_COMPLETE = 1 << 26
    HW_IMAGE_ENCODER = 1 << 27
    FRONT_BUFFER = 1 << 32
    VENDOR_MASK = 0xf << 28
    VENDOR_MASK_HI = (1 * 0xffff) << 48

class GrallocUsage:
    GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY = 1 << 31
    GRALLOC_USAGE_PRIVATE_CDSP = 1 << 50
    GRALLOC_USAGE_PRIVATE_WFD = 1 << 51
    GRALLOC_USAGE_PRIVATE_VIDEO_HW = 1 << 52
    ONLY_GPU_USAGE_MASK = \
    (BufferUsage.GPU_TEXTURE | BufferUsage.GPU_RENDER_TARGET | BufferUsage.GPU_CUBE_MAP |
        BufferUsage.GPU_MIPMAP_COMPLETE | BufferUsage.GPU_DATA_BUFFER | BufferUsage.RENDERSCRIPT)

    NON_GPU_USAGE_MASK = \
    (BufferUsage.COMPOSER_CLIENT_TARGET | BufferUsage.COMPOSER_OVERLAY |
        BufferUsage.COMPOSER_CURSOR | BufferUsage.VIDEO_ENCODER | BufferUsage.CAMERA_OUTPUT |
        BufferUsage.CAMERA_INPUT | BufferUsage.VIDEO_DECODER | GRALLOC_USAGE_PRIVATE_CDSP |
        GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY | GRALLOC_USAGE_PRIVATE_VIDEO_HW)

@register_parser('--sf', 'Extract the content of dumpsys SurfaceFlinger --allocated-buffers ')
class Surfaceflinger(RamParser):

    def __init__(self, *args):
        super(Surfaceflinger, self).__init__(*args)

    def read_bytes(self, addr, len=8):
        return UTaskLib.read_bytes(self.ramdump, self.taskinfo.mmu, addr, len)

    def read_binary(self, addr, len):
        return UTaskLib.read_binary(self.ramdump, self.taskinfo.mmu, addr, len)

    def find_bss_addrs(self, shared_lib_name):
        """ find vma list of bss section """
        bss_vms_list = []
        # bss section is after data section
        for index, vma in enumerate(self.taskinfo.vmalist):
            tmpstartVm = vma.vm_start
            tmpendVm   = vma.vm_end
            # rw flags
            if vma.file_name == shared_lib_name and vma.flags & 0b11 == 0b11:
                bss_vms_list.append([tmpstartVm, tmpendVm])
                # anon page
                vma_next = self.taskinfo.vmalist[index + 1]
                if not vma_next.file_name and vma_next.flags & 0b11 == 0b11:
                    bss_vms_list.append([vma_next.vm_start, vma_next.vm_end])
        return bss_vms_list

    def dump(self):
        self.output = self.ramdump.open_file("surfaceflinger.txt")
        self.dump_GraphicBufferAllocator()
        self.dumpDebugInfo()

    def dump_GraphicBufferAllocator(self):
        self.output.write("GraphicBufferAllocator buffers:\n")
        self.output.write("%10s | %11s | %18s | %s | %8s | %10s | %s\n" % ("Handle", "Size",
                  "W (Stride) x H", "Layers", "Format", "Usage", "Requestor"))

        mStroage_addr = self.read_bytes(self.sAllocList_addr + 8, 8)
        mCount = self.read_bytes(self.sAllocList_addr +16, 8)
        mFlags = self.read_bytes(self.sAllocList_addr +24, 4)
        mItemSize = self.read_bytes(self.sAllocList_addr +32, 8)
        print_out_str("mStroage_addr=0x%x mCount=%d mFlags=0x%x mItemSize=%d" % (mStroage_addr, mCount, mFlags, mItemSize))
        i= 0
        total = 0
        while(i < mCount):
            handle_ptr = self.read_bytes(mStroage_addr, 8) #8byte
            width = self.read_bytes(mStroage_addr + 8, 4)
            height = self.read_bytes(mStroage_addr + 12, 4)
            stride = self.read_bytes(mStroage_addr + 16, 4)
            format = self.read_bytes(mStroage_addr + 20, 4)
            layerCount = self.read_bytes(mStroage_addr + 24, 8)
            usage = self.read_bytes(mStroage_addr + 32, 8)
            size = self.read_bytes(mStroage_addr + 40, 8)
            requestorName = cleanupString(
                self.read_binary(mStroage_addr + 48, 24).decode('ascii', 'ignore')) # 24 byte
            ostr = "0x%10x | %7.2f KiB | %4u (%4u) x %4u | %6u | %8X | 0x%8x | %s\n" % \
                        (handle_ptr, size / 1024.0, width, stride, height,
                        layerCount, format, usage, requestorName)
            self.output.write(ostr)
            total += size
            mStroage_addr += 72
            i += 1

        print_out_str("Total allocated by GraphicBufferAllocator (estimate): %.2f KB\n" % (total / 1024.0))
        self.output.write("Total allocated by GraphicBufferAllocator (estimate): %.2f KB\n" % (total / 1024.0))

    def is_valid_useraddr(self, addr):
        ''' is valid user space address'''
        return ((addr >>48) & 0xff) == 0 if addr else False

    def is_sAllocList_addr(self, addr):
        mStroage_addr = self.read_bytes(addr + 8, 8)
        mCount = self.read_bytes(addr +16, 8)
        mFlags = self.read_bytes(addr +24, 4)
        mItemSize = self.read_bytes(addr +32, 8)
        if mItemSize != 72 or mCount > 100 or mCount <= 0:
            return False
        i= 0
        while(i < mCount):
            if not self.is_valid_useraddr(mStroage_addr):
                return False

            handle_ptr = self.read_bytes(mStroage_addr, 8) #8byte
            if not self.is_valid_useraddr(handle_ptr):
                return False

            handle_ptr = self.read_bytes(mStroage_addr, 8)  # 8byte
            request_data = self.read_binary(mStroage_addr + 48, 24)
            if request_data:
                requestorName = cleanupString(request_data.decode('ascii', 'ignore'))
                print_out_str("requestorName",requestorName)
            else:
                return False
            i += 1
            mStroage_addr = mStroage_addr + 72
        return True

    def find_sAllocList_addr(self, vma):
        if not vma or len(vma) != 2:
            print_out_str("invalid bss range", vma)
            return None
        vma_start = vma[0]
        vma_end = vma[1]

        addr = vma_start
        while addr < vma_end:
            isAllocList = self.is_sAllocList_addr(addr)
            if isAllocList:
                return addr
            addr = addr + 4
        return None

    def GetDRMFormat(self, format, flags):
        compressed = True if (int(flags) & QtigrallocFlags.PRIV_FLAGS_UBWC_ALIGNED) else False
        drm_format = 0
        drm_format_modifier = 0

        if format == PixelFormat.RGBA_8888:
            drm_format = DrmFormat.DRM_FORMAT_ABGR8888
        elif format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_5551:
            drm_format = DrmFormat.DRM_FORMAT_ABGR1555
        elif format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_4444:
            drm_format = DrmFormat.DRM_FORMAT_ABGR4444
        elif format == PixelFormat.BGRA_8888:
            drm_format = DrmFormat.DRM_FORMAT_ARGB8888
        elif format == PixelFormat.RGBX_8888:
            drm_format = DrmFormat.DRM_FORMAT_XBGR8888
            if compressed:
                drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED
        elif format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_8888:
            drm_format = DrmFormat.DRM_FORMAT_XRGB8888
        elif format == PixelFormat.RGB_888:
            drm_format = DrmFormat.DRM_FORMAT_BGR888

        elif format == PixelFormat.RGB_565:
            drm_format = DrmFormat.DRM_FORMAT_BGR565
        elif format == PixelFormat.HAL_PIXEL_FORMAT_BGR_565:
            drm_format = DrmFormat.DRM_FORMAT_BGR565
            if compressed:
                drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED
        elif format == PixelFormat.RGBA_1010102:
            drm_format = DrmFormat.DRM_FORMAT_ABGR2101010;
            if compressed:
                drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED
        elif format == PixelFormat.HAL_PIXEL_FORMAT_ARGB_2101010:
            drm_format = DrmFormat.DRM_FORMAT_BGRA1010102
        elif format == PixelFormat.HAL_PIXEL_FORMAT_RGBX_1010102:
            drm_format = DrmFormat.DRM_FORMAT_XBGR2101010
            if compressed:
               drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED
        elif format == PixelFormat.HAL_PIXEL_FORMAT_XRGB_2101010:
            drm_format = DrmFormat.DRM_FORMAT_BGRX1010102

        elif format == PixelFormat.HAL_PIXEL_FORMAT_BGRA_1010102:
            drm_format = DrmFormat.DRM_FORMAT_ARGB2101010
        elif format == PixelFormat.HAL_PIXEL_FORMAT_ABGR_2101010:
            drm_format = DrmFormat.DRM_FORMAT_RGBA1010102

        elif format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_1010102:
            drm_format = DrmFormat.DRM_FORMAT_XRGB2101010

        elif format == PixelFormat.HAL_PIXEL_FORMAT_XBGR_2101010:
            drm_format = DrmFormat.DRM_FORMAT_RGBX1010102

        elif format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS or \
                format == PixelFormat.HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
            drm_format = DrmFormat.DRM_FORMAT_NV12
        elif any ([format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC,
                    format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX,
                    format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH,
                    format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH,
                    format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH]):
            drm_format = DrmFormat.DRM_FORMAT_NV12
            drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED \
                    if compressed else DrmFormat.DRM_FORMAT_MOD_QCOM_TILE
        elif format == PixelFormat.YCRCB_420_SP:
            drm_format = DrmFormat.DRM_FORMAT_NV21
        elif format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
            drm_format = DrmFormat.DRM_FORMAT_NV21

        elif format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010 or \
            format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
            drm_format = DrmFormat.DRM_FORMAT_NV12
            drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_DX

        elif format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
            drm_format = DrmFormat.DRM_FORMAT_NV12
            if compressed:
                drm_format_modifier = \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED | \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_DX
            else:
                drm_format_modifier = DrmFormat.DRM_FORMAT_MOD_QCOM_TILE | \
                                      DrmFormat.DRM_FORMAT_MOD_QCOM_DX


        elif format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
            drm_format = DrmFormat.DRM_FORMAT_NV12
            if compressed:
                drm_format_modifier = \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_COMPRESSED | \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_DX | \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_TIGHT
            else:
                drm_format_modifier = \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_TILE | \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_DX | \
                    DrmFormat.DRM_FORMAT_MOD_QCOM_TIGHT


        elif format == PixelFormat.YCBCR_422_SP:
            drm_format = DrmFormat.DRM_FORMAT_NV16
        elif format == PixelFormat.YV12:
            drm_format = DrmFormat.DRM_FORMAT_YVU420
        elif format == PixelFormat.RGBA_FP16:
            drm_format = DrmFormat.DRM_FORMAT_ABGR16161616F
        else:
            pass
            #print("GetDRMFormat: Unsupported format %d" % format)
        return drm_format, drm_format_modifier

    def ColorMetadataToDataspace(self, private_handle_t, ColorMetaData_color_addr):
        color_primaries = self.read_bytes(ColorMetaData_color_addr, 4)
        if not color_primaries:
            color_primaries = 0

        color_range = self.read_bytes(ColorMetaData_color_addr + 4, 4)
        if not color_range:
            color_range = 0

        color_transfer = self.read_bytes(ColorMetaData_color_addr + 8, 4)
        if not color_transfer:
            color_transfer = 0

        private_handle_t.color_primaries = color_primaries
        private_handle_t.color_range = color_range
        private_handle_t.color_transfer = color_transfer

        primaries = None

        if color_primaries == ColorPrimaries.ColorPrimaries_BT709_5:
            primaries = Dataspace.STANDARD_BT709
        elif color_primaries == ColorPrimaries.ColorPrimaries_BT470_6M:
            primaries = Dataspace.STANDARD_BT470M
        elif color_primaries == ColorPrimaries.ColorPrimaries_BT601_6_625:
            primaries = Dataspace.STANDARD_BT601_625
        elif color_primaries == ColorPrimaries.ColorPrimaries_BT601_6_525:
            primaries = Dataspace.STANDARD_BT601_525
        elif color_primaries == ColorPrimaries.ColorPrimaries_GenericFilm:
            primaries = Dataspace.STANDARD_FILM
        elif color_primaries == ColorPrimaries.ColorPrimaries_BT2020:
            primaries = Dataspace.STANDARD_BT2020
        elif color_primaries == ColorPrimaries.ColorPrimaries_AdobeRGB:
            primaries = Dataspace.STANDARD_ADOBE_RGB
        elif color_primaries == ColorPrimaries.ColorPrimaries_DCIP3:
            primaries = Dataspace.STANDARD_DCI_P3
        else:
            #print("unsupported color_primaries 0x%x " % color_primaries)
            return None
        if color_transfer == GammaTransfer.Transfer_sRGB:
            transfer = Dataspace.TRANSFER_SRGB
        elif color_transfer == GammaTransfer.Transfer_Gamma2_2:
            transfer = Dataspace.TRANSFER_GAMMA2_2

        elif color_transfer == GammaTransfer.Transfer_Gamma2_8:
            transfer = Dataspace.TRANSFER_GAMMA2_8

        elif color_transfer == GammaTransfer.Transfer_SMPTE_170M:
            transfer = Dataspace.TRANSFER_SMPTE_170M
        elif color_transfer == GammaTransfer.Transfer_Linear:
            transfer = Dataspace.TRANSFER_LINEAR

        elif color_transfer == GammaTransfer.Transfer_HLG:
            transfer = Dataspace.Transfer_HLG
        else:
            #print("unsupported color_transfer 0x%x" % color_transfer)
            return None
        if color_range == ColorRange.Range_Full:
            range = Dataspace.RANGE_FULL
        elif color_range == ColorRange.Range_Limited:
            range = Dataspace.RANGE_LIMITED

        elif color_range == ColorRange.Range_Extended:
            range = Dataspace.RANGE_EXTENDED
        else:
            #print("unsupported color_range 0x%x" % color_range)
            return None
        return primaries | transfer | range

    def bufferDumpHelper(self, handle_addr):
        pht = PrivateHandleT()
        private_handle_t_id_oft = 0x38
        private_handle_t_base_metadata_oft = 0x5c
        pht.buffer_id = self.read_bytes(handle_addr + private_handle_t_id_oft, 4)
        pht.base_metadata_addr = self.read_bytes(handle_addr + private_handle_t_base_metadata_oft)
        if not pht.base_metadata_addr:
            return 0
        MetaData_t_name_oft = 0x60c0
        name = cleanupString(
            self.read_binary(pht.base_metadata_addr + MetaData_t_name_oft, 256).decode('ascii', 'ignore'))

        pht.width = self.read_bytes(handle_addr + 0x1c, 4)
        pht.height = self.read_bytes(handle_addr + 0x20, 4)
        pht.unaligned_width = self.read_bytes(handle_addr + 0x24, 4)
        pht.unaligned_height = self.read_bytes(handle_addr + 0x28, 4)
        pht.layer_count = self.read_bytes(handle_addr + 0x34, 4)
        pht.allocate_size = self.read_bytes(handle_addr + 0x48, 4)
        pht.pixelFormatRequested = self.read_bytes(handle_addr + 0x2c, 4)
        pht.flags = self.read_bytes(handle_addr + 0x18, 4)
        pht.format = self.read_bytes(handle_addr + 0x2c, 4)
        pixelFormatFourCC, pixelFormatModifier = self.GetDRMFormat(pht.pixelFormatRequested, pht.flags)
        pht.usage = self.read_bytes(handle_addr + 0x40)

        ColorMetaData_color_addr = pht.base_metadata_addr + 0x28
        dataspace = self.ColorMetadataToDataspace(pht, ColorMetaData_color_addr)
        if not dataspace:
            dataspace = 0
        pht.allocationSize = self.read_bytes(handle_addr + 0x48, 4)
        pht.protectedContent = 1 if (pht.flags & QtigrallocFlags.PRIV_FLAGS_SECURE_BUFFER) else 0

        if pht.flags & QtigrallocFlags.PRIV_FLAGS_UBWC_ALIGNED or \
                pht.flags & QtigrallocFlags.PRIV_FLAGS_UBWC_ALIGNED_PI:
            compression = ExtendableType(ExtendableType.VENDOR_QTI, ExtendableType.COMPRESSION_QTI_UBWC)
        else:
            compression = ExtendableType(ExtendableType.GRALLOC4_STANDARD_COMPRESSION, ExtendableType.NONE)

        MetaData_t_interlaced_oft = 4
        interlaced_int = self.read_bytes(pht.base_metadata_addr + MetaData_t_interlaced_oft, 4)

        if interlaced_int and interlaced_int > 0:
            interlaced = ExtendableType(ExtendableType.VENDOR_QTI, ExtendableType.INTERLACED_QTI)
        else:
            interlaced = ExtendableType(ExtendableType.GRALLOC4_STANDARD_INTERLACED, ExtendableType.NONE)

        chromaSiting = ExtendableType(ExtendableType.GRALLOC4_STANDARD_CHROMA_SITING, ExtendableType.NONE)

        allocationSizeKiB = round(pht.allocationSize / 1024.0, 2)
        isCompressed = not self.isStandardCompression(compression) or\
                       (self.getStandardCompressionValue(compression) != ExtendableType.NONE)

        outStr = "+ name:{}, id:{}, size:{}0KiB, w/h:{}x{}, usage: {}," \
                 "req fmt:{}, fourcc/mod: {}/{}, dataspace: {}, compressed: {}\n"\
            .format(name,
                    pht.buffer_id,
                    allocationSizeKiB,
                    pht.unaligned_width,
                    pht.unaligned_height,
                    hex(pht.usage),
                    pht.pixelFormatRequested,
                    pixelFormatFourCC,
                    pixelFormatModifier,
                    hex(dataspace),
                    isCompressed)
        self.output.write(outStr)
        self.GetPlaneLayout(pht)
        return pht.allocate_size

    def GetPlaneLayout(self, p_handle_t):
        if self.IsUncompressedRGBFormat(p_handle_t.format)\
                or self.IsCompressedRGBFormat(p_handle_t.format):
            self.GetRGBPlaneInfo(p_handle_t)
        elif self.IsYuvFormat(p_handle_t.format):
            self.output.write("not support yuv format\n")
        else:
            self.output.write("Bad buffer with format {}\n".format(p_handle_t.format))

    def IsUncompressedRGBFormat(self, format):
        return any([format == PixelFormat.RGBA_8888,
            format == PixelFormat.RGBX_8888,
            format == PixelFormat.RGB_888,
            format == PixelFormat.RGB_565,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGR_565,
            format == PixelFormat.BGRA_8888,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_5551,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_4444,
            format == PixelFormat.HAL_PIXEL_FORMAT_R_8,
            format == PixelFormat.R_8,
            format == PixelFormat.HAL_PIXEL_FORMAT_RG_88,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_8888,
            format == PixelFormat.RGBA_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_ARGB_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBX_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_XRGB_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRA_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_ABGR_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_XBGR_2101010,
            format == PixelFormat.RGBA_FP16,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGR_888])

    def GetRGBPlaneInfo(self, p_handle_t):
        component = PlaneComponent.PLANE_COMPONENT_R | \
                    PlaneComponent.PLANE_COMPONENT_G | \
                    PlaneComponent.PLANE_COMPONENT_B

        if self.HasAlphaComponent(p_handle_t.format):
            component |= PlaneComponent.PLANE_COMPONENT_A

        step = self.GetBpp(p_handle_t.format)

        h_subsampling = 0
        v_subsampling = 0
        stride = p_handle_t.width
        stride_bytes = p_handle_t.width * step
        scanlines = p_handle_t.height
        self.output.write("\tplanes: ")
        types = self.grallocToStandardPlaneLayoutComponentType(component)
        outstr = ""
        for i, type in enumerate(types):
            outstr += type
            if i != (len(types) -1):
                outstr += "/"
            else:
                outstr += ":\t"
        self.output.write(outstr)

        widthInSamples = p_handle_t.unaligned_width >> h_subsampling
        heightInSamples = p_handle_t.unaligned_height >> v_subsampling
        strideInBytes = stride_bytes
        totalSizeInBytes = p_handle_t.allocationSize
        outstr = " w/h:{}x{}, stride:{} bytes, size:{}\n".format(
            widthInSamples, heightInSamples, strideInBytes, totalSizeInBytes)
        self.output.write(outstr)

    def grallocToStandardPlaneLayoutComponentType(self, component):
        types=[]
        if (component & PlaneComponent.PLANE_COMPONENT_Y):
            types.append("Y")
        if (component & PlaneComponent.PLANE_COMPONENT_Cb):
            types.append("Cb")
        if (component & PlaneComponent.PLANE_COMPONENT_Cr):
            types.append("Cr")
        if (component & PlaneComponent.PLANE_COMPONENT_R):
            types.append("R")
        if (component & PlaneComponent.PLANE_COMPONENT_G):
            types.append("G")
        if (component & PlaneComponent.PLANE_COMPONENT_B):
            types.append("B")
        if (component & PlaneComponent.PLANE_COMPONENT_A):
            types.append("A")
        if (component & PlaneComponent.PLANE_COMPONENT_RAW):
            types.append("RAW")
        if (component & PlaneComponent.PLANE_COMPONENT_META):
            types.append("{}#{}".format(ExtendableType.VENDOR_QTI, PlaneComponent.PLANE_COMPONENT_META))
        return types

    def GetBppForUncompressedRGB(self, format):
        bpp = 0
        if format == PixelFormat.RGBA_FP16:
            bpp = 8
        elif any([
            format == PixelFormat.RGBA_8888,
            format == PixelFormat.RGBX_8888,
            format == PixelFormat.BGRA_8888,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_8888,
            format == PixelFormat.RGBA_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_ARGB_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBX_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_XRGB_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRA_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_ABGR_2101010,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGRX_1010102,
            format == PixelFormat.HAL_PIXEL_FORMAT_XBGR_2101010]):
            bpp = 4

        elif any([
            format == PixelFormat.RGB_888,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGR_888]):
            bpp = 3
        elif any([
            format == PixelFormat.RGB_565,
            format == PixelFormat.HAL_PIXEL_FORMAT_BGR_565,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_5551,
            format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_4444,
            format == PixelFormat.HAL_PIXEL_FORMAT_RG_88]):
            bpp = 2
        elif any([
            format == PixelFormat.HAL_PIXEL_FORMAT_R_8,
            format == PixelFormat.R_8]):
            bpp = 1
        else:
            #print("Error , GetBppForUncompressedRGB New format request = 0x%x" % format)
            pass
        return bpp

    def GetBpp(self, format):
        if (self.IsUncompressedRGBFormat(format)):
            return self.GetBppForUncompressedRGB(format)

        if any([
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_RAW8,
            format == PixelFormat.Y8]):
            return 1
        elif any([
            format == PixelFormat.RAW16,
            format == PixelFormat.Y16,
            format == PixelFormat.YCBCR_422_SP,
            format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_422_SP,
            format == PixelFormat.YCBCR_422_I,
            format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_422_I,
            format == PixelFormat.HAL_PIXEL_FORMAT_CbYCrY_422_I]):
          return 2
        elif any([
            format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS,
            format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010]):
            return 3
        return -1

    def GetBufferType(self, inputFormat):
      return BufferType.BUFFER_TYPE_VIDEO if self.IsYuvFormat(inputFormat) else BufferType.BUFFER_TYPE_UI

    def IsYuvFormat(self, format):
        return any([format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_SP,
        format == PixelFormat.YCBCR_422_SP,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_ENCODEABLE,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC,
        format == PixelFormat.YCRCB_420_SP,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_422_SP,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV21_ZSL,
        format == PixelFormat.RAW16,
        format == PixelFormat.Y16,
        format == PixelFormat.RAW12,
        format == PixelFormat.RAW10,
        format == PixelFormat.HAL_PIXEL_FORMAT_RAW8,
        format == PixelFormat.YV12,
        format == PixelFormat.Y8,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC,
        format == PixelFormat.HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS,
        format == PixelFormat.BLOB,
        format == PixelFormat.RAW_OPAQUE,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_HEIF,
        format == PixelFormat.HAL_PIXEL_FORMAT_CbYCrY_422_I,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH,
        format == PixelFormat.HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH,
        format == PixelFormat.HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX])

    def HasAlphaComponent(self, format):
        return any([format == PixelFormat.RGBA_8888,
        format == PixelFormat.BGRA_8888,
        format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_5551,
        format == PixelFormat.HAL_PIXEL_FORMAT_RGBA_4444,
        format == PixelFormat.RGBA_1010102,
        format == PixelFormat.HAL_PIXEL_FORMAT_ARGB_2101010,
        format == PixelFormat.HAL_PIXEL_FORMAT_BGRA_1010102,
        format == PixelFormat.HAL_PIXEL_FORMAT_ABGR_2101010,
        format == PixelFormat.RGBA_FP16])

    def IsCompressedRGBFormat(self, format):
        return any([
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR,
            format == PixelFormat.HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR])

    def getStandardCompressionValue(self, compression):
        return compression.value

    #ExtendableType compression
    def isStandardCompression(self, compression):
        return compression.name == ExtendableType.GRALLOC4_STANDARD_COMPRESSION

    def dumpDebugInfo(self):
        GraphicBufferAllocator_sInstance_addr = \
            self.read_bytes(self.sAllocList_addr - 0x30)

        GraphicBufferAllocator_mAllocator_oft = 8
        Gralloc4Allocator_mMapper_oft = 8
        Gralloc4Mapper_mMapper_oft = 8
        GraphicBufferAllocator_mAllocator_addr = \
            self.read_bytes(GraphicBufferAllocator_sInstance_addr + GraphicBufferAllocator_mAllocator_oft)

        Gralloc4Allocator_mMapper_addr = \
            self.read_bytes(GraphicBufferAllocator_mAllocator_addr + Gralloc4Allocator_mMapper_oft)

        Gralloc4Mapper_mMapper_addr = \
            self.read_bytes(Gralloc4Allocator_mMapper_addr + Gralloc4Mapper_mMapper_oft)

        BsMapper_mImpl_oft = 0x60
        BsMapper_mImpl = self.read_bytes(Gralloc4Mapper_mMapper_addr + BsMapper_mImpl_oft)

        QtiMapper_buf_mgr_oft = 0x10
        QtiMapper_buf_mgr_addr = self.read_bytes(BsMapper_mImpl + QtiMapper_buf_mgr_oft)

        BufferManagr_handles_map_oft = 0x30
        BufferManagr_handles_map_addr = QtiMapper_buf_mgr_addr + BufferManagr_handles_map_oft
        first_item_addr = self.read_bytes(BufferManagr_handles_map_addr + 16)
        map_size = self.read_bytes(BufferManagr_handles_map_addr + 24)

        idx = 0
        next_item_addr = first_item_addr
        self.output.write("Imported gralloc buffers:\n")
        totalAllocationSize = 0
        while idx < map_size:
            handle_addr = self.read_bytes(next_item_addr + 0x10) #key
            value_addr = self.read_bytes(next_item_addr + 0x18) #value
            allocationSize = self.bufferDumpHelper(handle_addr)
            totalAllocationSize += allocationSize
            idx += 1
            next_item_addr = self.read_bytes(next_item_addr)
        totalAllocationSizeKiB = totalAllocationSize / 1024
        print_out_str("Total imported by gralloc: {}KiB\n".format(totalAllocationSizeKiB))
        self.output.write("Total imported by gralloc: {}KiB\n".format(totalAllocationSizeKiB))

    def parse(self):
        try:
            try:
                self.taskinfo = UTaskLib(self.ramdump).get_utask_info("surfaceflinger")
            except ProcessNotFoundExcetion:
                print_out_str("surfaceflinger process is not started")
                return

            bss_addrs = self.find_bss_addrs("libui.so")
            if not bss_addrs:
                print_out_str("can't find vma of libui.so")
                return

            self.sAllocList_addr = None
            for bss in bss_addrs:
                addr = self.find_sAllocList_addr(bss)
                if addr:
                    self.sAllocList_addr = addr
                    print_out_str("found sAllocList_addr 0x%x " % self.sAllocList_addr)
                    break

            if not self.sAllocList_addr:
                print_out_str("can't find sAllocList address")
                return

            self.dump()
        except Exception as result:
            print_out_str(str(result))
            traceback.print_exc()
