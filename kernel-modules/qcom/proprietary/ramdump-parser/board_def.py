"""
Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
Copyright (c) 2019-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
"""

from boards import Board

class Board8960(Board):
    def __init__(self, socid, board_num, phys_offset=0x80200000, ram_start=0x80000000):
        super(Board8960, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'KRAIT'
        self.ram_start = ram_start
        self.imem_start = 0x2a03f000
        self.smem_addr = 0x0
        self.phys_offset = phys_offset
        self.wdog_addr = 0x2a03f658
        self.imem_file_name = 'IMEM_C.BIN'

class Board8625(Board):
    def __init__(self, socid, board_num):
        super(Board8625, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'SCORPION'
        self.ram_start = 0
        self.imem_start = 0x0
        self.smem_addr = 0x00100000
        self.phys_offset = 0x00200000

class Board9615(Board):
    def __init__(self, socid):
        super(Board9615, self).__init__()
        self.socid = socid
        self.board_num = "9615"
        self.cpu = 'CORTEXA5'
        self.ram_start = 0x40000000
        self.imem_start = 0
        self.smem_addr = 0x0
        self.phys_offset = 0x40800000

class Board8974(Board):
    def __init__(self, socid, board_num="8974"):
        super(Board8974, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'KRAIT'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0xfa00000
        self.phys_offset = 0x0
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board9625(Board):
    def __init__(self, socid):
        super(Board9625, self).__init__()
        self.socid = socid
        self.board_num = "9625"
        self.cpu = 'CORTEXA5'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0x0
        self.phys_offset = 0x200000
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8626(Board):
    def __init__(self, socid, board_num="8626"):
        super(Board8626, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0x0fa00000
        self.phys_offset = 0x0
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8026LW(Board):
    def __init__(self, socid, board_num="8026"):
        super(Board8026LW, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0x03000000
        self.phys_offset = 0x0
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8610(Board):
    def __init__(self, socid, board_num="8610"):
        super(Board8610, self).__init__()
        self.socid = socid
        self.board_num = board_num
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0x0d900000
        self.phys_offset = 0x0
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board9635(Board):
    def __init__(self, socid):
        super(Board9635, self).__init__()
        self.socid = socid
        self.board_num = "9635"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x0
        self.imem_start = 0xfe800000
        self.smem_addr = 0x1100000
        self.phys_offset = 0
        self.wdog_addr = 0xfe805658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8916(Board):
    def __init__(self, socid, smem_addr):
        super(Board8916, self).__init__()
        self.socid = socid
        self.board_num = "8916"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        #self.ram_start = 0x0
        self.smem_addr = smem_addr
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8939(Board):
    def __init__(self, socid, smem_addr):
        super(Board8939, self).__init__()
        self.socid = socid
        self.board_num = "8939"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = smem_addr
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8936(Board):
    def __init__(self, socid):
        super(Board8936, self).__init__()
        self.socid = socid
        self.board_num = "8936"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8994(Board):
    def __init__(self, socid):
        super(Board8994, self).__init__()
        self.socid = socid
        self.board_num = "8994"
        self.cpu = 'CORTEXA57A53'
        self.ram_start = 0x0
        self.smem_addr = 0x6a00000
        self.phys_offset = 0x0
        self.imem_start = 0xfe800000
        self.wdog_addr = 0xfe87f658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8909(Board):
    def __init__(self, socid):
        super(Board8909, self).__init__()
        self.socid = socid
        self.board_num = "8909"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7d00000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8908(Board):
    def __init__(self, socid):
        super(Board8908, self).__init__()
        self.socid = socid
        self.board_num = "8908"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7d00000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board9640(Board):
    def __init__(self, socid):
        super(Board9640, self).__init__()
        self.socid = socid
        self.board_num = "9640"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e80000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8992(Board):
    def __init__(self, socid):
        super(Board8992, self).__init__()
        self.socid = socid
        self.board_num = "8992"
        self.cpu = 'CORTEXA57A53'
        self.ram_start = 0x0
        self.smem_addr = 0x6a00000
        self.phys_offset = 0x0
        self.imem_start = 0xfe800000
        self.wdog_addr = 0xfe87f658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8929(Board):
    def __init__(self, socid, smem_addr):
        super(Board8929, self).__init__()
        self.socid = socid
        self.board_num = "8929"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = smem_addr
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658

class Board8996(Board):
    def __init__(self, socid):
        super(Board8996, self).__init__()
        self.socid = socid
        self.board_num = "8996"
        self.cpu = 'HYDRA'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.phys_offset = 0x80000000
        self.imem_start = 0x6680000
        self.wdog_addr = 0x66BF658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8952(Board):
    def __init__(self, socid):
        super(Board8952, self).__init__()
        self.socid = socid
        self.board_num = "8952"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8976(Board):
    def __init__(self, socid):
        super(Board8976, self).__init__()
        self.socid = socid
        self.board_num = "8976"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x20000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board9607(Board):
    def __init__(self, socid):
        super(Board9607, self).__init__()
        self.socid = socid
        self.board_num = "9607"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7d00000
        self.phys_offset = 0x80000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'


class Board8937(Board):
    def __init__(self, socid):
        super(Board8937, self).__init__()
        self.socid = socid
        self.board_num = "8937"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8940(Board):
     def __init__(self, socid):
         super(Board8940, self).__init__()
         self.socid = socid
         self.board_num = "8940"
         self.cpu = 'CORTEXA53'
         self.ram_start = 0x80000000
         self.smem_addr = 0x6300000
         self.phys_offset = 0x40000000
         self.imem_start = 0x8600000
         self.wdog_addr = 0x8600658
         self.imem_file_name = 'OCIMEM.BIN'

class Board8953(Board):
    def __init__(self, socid):
        super(Board8953, self).__init__()
        self.socid = socid
        self.board_num = "8953"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board450(Board):
    def __init__(self, socid):
        super(Board450, self).__init__()
        self.socid = socid
        self.board_num = "450"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.kaslr_addr = 0x86006d0
        self.imem_file_name = 'OCIMEM.BIN'

class Board632(Board):
    def __init__(self, socid):
        super(Board632, self).__init__()
        self.socid = socid
        self.board_num = "632"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board439(Board):
    def __init__(self, socid):
        super(Board439, self).__init__()
        self.socid = socid
        self.board_num = "sdm439"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board429(Board):
    def __init__(self, socid):
        super(Board429, self).__init__()
        self.socid = socid
        self.board_num = "sdm429"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8917(Board):
    def __init__(self, socid):
        super(Board8917, self).__init__()
        self.socid = socid
        self.board_num = "8917"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8920(Board):
     def __init__(self, socid):
         super(Board8920, self).__init__()
         self.socid = socid
         self.board_num = "8920"
         self.cpu = 'CORTEXA53'
         self.ram_start = 0x80000000
         self.smem_addr = 0x6300000
         self.phys_offset = 0x40000000
         self.imem_start = 0x8600000
         self.wdog_addr = 0x8600658
         self.imem_file_name = 'OCIMEM.BIN'

class BoardCalifornium(Board):
    def __init__(self, socid):
        super(BoardCalifornium, self).__init__()
        self.socid = socid
        self.board_num = "californium"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e80000
        self.phys_offset = 0x80000000
        self.imem_start = 0x08600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSdxnightjar(Board):
    def __init__(self, socid):
        super(BoardSdxnightjar, self).__init__()
        self.socid = socid
        self.board_num = "sdxnightjar"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e80000
        self.phys_offset = 0x80000000
        self.imem_start = 0x08600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardCobalt(Board):
    def __init__(self, socid):
        super(BoardCobalt, self).__init__()
        self.socid = socid
        self.board_num = "cobalt"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSDM845(Board):
    def __init__(self, socid):
        super(BoardSDM845, self).__init__()
        self.socid = socid
        self.board_num = "sdm845"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSDM710(Board):
    def __init__(self, socid):
        super(BoardSDM710, self).__init__()
        self.socid = socid
        self.board_num = "sdm710"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardTrinket(Board):
    def __init__(self, socid):
        super(BoardTrinket, self).__init__()
        self.socid = socid
        self.board_num = "trinket"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x40000000
        self.imem_start = 0x0c100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0c125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardQCS605(Board):
    def __init__(self, socid):
        super(BoardQCS605, self).__init__()
        self.socid = socid
        self.board_num = "qcs605"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardQCS405(Board):
    def __init__(self, socid):
        super(BoardQCS405, self).__init__()
        self.socid = socid
        self.board_num = "qcs405"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.kaslr_addr = 0x86006d0
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardQCS403(Board):
    def __init__(self, socid):
        super(BoardQCS403, self).__init__()
        self.socid = socid
        self.board_num = "qcs403"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6300000
        self.phys_offset = 0x40000000
        self.imem_start = 0x8600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class Board8998(Board):
    def __init__(self, socid):
        super(Board8998, self).__init__()
        self.socid = socid
        self.board_num = "8998"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class Board660(Board):
    def __init__(self, socid):
        super(Board660, self).__init__()
        self.socid = socid
        self.board_num = "660"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6006ec0
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class Board630(Board):
    def __init__(self, socid):
        super(Board630, self).__init__()
        self.socid = socid
        self.board_num = "630"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSDX20(Board):
    def __init__(self, socid):
        super(BoardSDX20, self).__init__()
        self.socid = socid
        self.board_num = "SDX20"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e80000
        self.phys_offset = 0x80000000
        self.imem_start = 0x08600000
        self.wdog_addr = 0x8600658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSdxbaagha(Board):
    def __init__(self, socid):
        super(BoardSdxbaagha, self).__init__()
        self.socid = socid
        self.board_num = "sdxbaagha"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x2280000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.wdog_addr = 0x14693658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardMsmnile(Board):
    def __init__(self, socid):
        super(BoardMsmnile, self).__init__()
        self.socid = socid
        self.board_num = "msmnile"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSteppe(Board):
    def __init__(self, socid):
        super(BoardSteppe, self).__init__()
        self.socid = socid
        self.board_num = "steppe"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardSdmshrike(Board):
    def __init__(self, socid):
        super(BoardSdmshrike, self).__init__()
        self.socid = socid
        self.board_num = "sdmshrike"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardLito(Board):
    def __init__(self, socid):
        super(BoardLito, self).__init__()
        self.socid = socid
        self.board_num = "lito"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA2400000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146ab6d0
        self.wdog_addr = 0x146ab658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardAtoll(Board):
    def __init__(self, socid):
        super(BoardAtoll, self).__init__()
        self.socid = socid
        self.board_num = "atoll"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA1200000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardLagoon(Board):
    def __init__(self, socid):
        super(BoardLagoon, self).__init__()
        self.socid = socid
        self.board_num = "lagoon"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA2400000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardHoli(Board):
    def __init__(self, socid):
        super(BoardHoli, self).__init__()
        self.socid = socid
        self.board_num = "holi"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA0000000
        self.imem_start = 0x0C100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0C125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardBlair(Board):
    def __init__(self, socid):
        super(BoardBlair, self).__init__()
        self.socid = socid
        self.board_num = "blair"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA0000000
        self.imem_start = 0x0C100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0C125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardPoorwills(Board):
    def __init__(self, socid):
        super(BoardPoorwills, self).__init__()
        self.socid = socid
        self.board_num = "poorwills"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0xFE40000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.wdog_addr =  0x14680658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardPrairie(Board):
    def __init__(self, socid):
        super(BoardPrairie, self).__init__()
        self.socid = socid
        self.board_num = "sdxprairie"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0xFE40000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.wdog_addr =  0x14680658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardLemur(Board):
    def __init__(self, socid):
        super(BoardLemur, self).__init__()
        self.socid = socid
        self.board_num = "sdxlemur"
        self.cpu = 'CORTEXA7'
        self.ram_start = 0x80000000
        self.smem_addr = 0x8FE20000
        self.phys_offset = 0x80000000
        self.imem_start = 0x14680000
        self.wdog_addr = 0x1468F658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardKona(Board):
    def __init__(self, socid):
        super(BoardKona, self).__init__()
        self.socid = socid
        self.board_num = "kona"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardBengal(Board):
    def __init__(self, socid):
        super(BoardBengal, self).__init__()
        self.socid = socid
        self.board_num = "bengal"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x40000000
        self.imem_start = 0x0c100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0c125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardKhaje(Board):
    def __init__(self, socid):
        super(BoardKhaje, self).__init__()
        self.socid = socid
        self.board_num = "khaje"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x40000000
        self.imem_start = 0x0c100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0c125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardLahaina(Board):
    def __init__(self, socid):
        super(BoardLahaina, self).__init__()
        self.socid = socid
        self.board_num = "lahaina"
        self.cpu = 'CORTEXA75'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.hyp_diag_addr = 0x146BFB30
        self.rm_debug_addr =  0x146BFBEC
        self.imem_file_name = 'OCIMEM.BIN'

class BoardDirewolf(Board):
    def __init__(self, socid):
        super(BoardDirewolf, self).__init__()
        self.socid = socid
        self.board_num = "direwolf"
        self.cpu = 'CORTEXA75'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.hyp_diag_addr = 0x146BFB30
        self.rm_debug_addr =  0x146BFBEC
        self.imem_file_name = 'OCIMEM.BIN'

class BoardWaipio(Board):
    def __init__(self, socid):
        super(BoardWaipio, self).__init__()
        self.socid = socid
        self.board_num = "taro"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardWaipioSVM(Board):
    def __init__(self, socid):
        super(BoardWaipioSVM, self).__init__()
        self.socid = socid
        self.board_num = "tarosvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardDiwali(Board):
    def __init__(self, socid):
        super(BoardDiwali, self).__init__()
        self.socid = socid
        self.board_num = "diwali"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardDiwaliSVM(Board):
    def __init__(self, socid):
        super(BoardDiwaliSVM, self).__init__()
        self.socid = socid
        self.board_num = "diwalisvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardCape(Board):
    def __init__(self, socid):
        super(BoardCape, self).__init__()
        self.socid = socid
        self.board_num = "cape"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardCapeSVM(Board):
    def __init__(self, socid):
        super(BoardCapeSVM, self).__init__()
        self.socid = socid
        self.board_num = "capesvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardUkee(Board):
    def __init__(self, socid):
        super(BoardUkee, self).__init__()
        self.socid = socid
        self.board_num = "ukee"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardUkeeSVM(Board):
    def __init__(self, socid):
        super(BoardUkeeSVM, self).__init__()
        self.socid = socid
        self.board_num = "ukeesvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardLahainaSVM(Board):
    def __init__(self, socid):
        super(BoardLahainaSVM, self).__init__()
        self.socid = socid
        self.board_num = "lahainasvm"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071c0
        self.phys_offset = 0xD0780000
        #self.phys_offset = 0xD0800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146bf6d0
        self.wdog_addr = 0x146BF658
        self.svm_kaslr_offset = 0x180000
        self.hyp_diag_addr = 0x146BFB30
        self.rm_debug_addr =  0x146BFBEC
        self.imem_file_name = 'OCIMEM.BIN'

class BoardKailua(Board):
    def __init__(self, socid):
        super(BoardKailua, self).__init__()
        self.socid = socid
        self.board_num = "kalama"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardKailuaSVM(Board):
    def __init__(self, socid):
        super(BoardKailuaSVM, self).__init__()
        self.socid = socid
        self.board_num = "kalamasvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xF3800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardKailuaOEMVM(Board):
    def __init__(self, socid):
        super(BoardKailuaOEMVM, self).__init__()
        self.socid = socid
        self.board_num = "kalamaoemvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xF7C00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardSC7180(Board):
    def __init__(self, socid):
        super(BoardSC7180, self).__init__()
        self.socid = socid
        self.board_num = "sc7180"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x907210
        self.phys_offset = 0x9b400000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardScuba(Board):
    def __init__(self, socid):
        super(BoardScuba, self).__init__()
        self.socid = socid
        self.board_num = "scuba"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x40000000
        self.imem_start = 0x0c100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0c125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardMonaco(Board):
    def __init__(self, socid):
        super(BoardMonaco, self).__init__()
        self.socid = socid
        self.board_num = "monaco"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x40000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0x40000000
        self.imem_start = 0x0c100000
        self.kaslr_addr = 0x0c1256d0
        self.wdog_addr = 0x0c125658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardShima(Board):
    def __init__(self, socid):
        super(BoardShima, self).__init__()
        self.socid = socid
        self.board_num = "shima"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071c0
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardShimaSVM(Board):
    def __init__(self, socid):
        super(BoardShimaSVM, self).__init__()
        self.socid = socid
        self.board_num = "shimasvm"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071c0
        self.phys_offset = 0xD0780000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardYupik(Board):
    def __init__(self, socid):
        super(BoardYupik, self).__init__()
        self.socid = socid
        self.board_num = "yupik"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071c0
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardYupikSVM(Board):
    def __init__(self, socid):
        super(BoardYupikSVM, self).__init__()
        self.socid = socid
        self.board_num = "yupiksvm"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071c0
        self.phys_offset = 0xD0780000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardNeo(Board):
    def __init__(self, socid):
        super(BoardNeo, self).__init__()
        self.socid = socid
        self.board_num = "neo"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146AA658
        self.imem_file_name = 'OCIMEM.BIN'

class BoardParrot(Board):
    def __init__(self, socid):
        super(BoardParrot, self).__init__()
        self.socid = socid
        self.board_num = "parrot"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr = 0x146AABEC

class BoardParrotSVM(Board):
    def __init__(self, socid):
        super(BoardParrotSVM, self).__init__()
        self.socid = socid
        self.board_num = "parrotsvm"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr = 0x146AABEC

class BoardRavelin(Board):
    def __init__(self, socid):
        super(BoardRavelin, self).__init__()
        self.socid = socid
        self.board_num = "ravelin"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr = 0x146AABEC

class BoardRavelinSVM(Board):
    def __init__(self, socid):
        super(BoardRavelinSVM, self).__init__()
        self.socid = socid
        self.board_num = "ravelinsvm"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xe0b00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr = 0x146AABEC

class BoardLassen(Board):
    def __init__(self, socid):
        super(BoardLassen, self).__init__()
        self.socid = socid
        self.board_num = "cinder"
        self.cpu = 'CORTEXA55'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9083e0
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.wdog_addr = 0x14680658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardAnorak(Board):
    def __init__(self, socid):
        super(BoardAnorak, self).__init__()
        self.socid = socid
        self.board_num = "anorak"
        self.cpu = 'CORTEXA53'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D083E0
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146AA658
        self.imem_file_name = 'OCIMEM.BIN'

class Boardsa8540(Board):
    def __init__(self, socid):
        super(Boardsa8540, self).__init__()
        self.socid = socid
        self.board_num = "sa8540"
        self.cpu = 'ARMv8-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x6000000
        self.smem_addr_buildinfo = 0x6007210
        self.phys_offset = 0xA0000000
        self.imem_start = 0x14680000
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'
        self.kaslr_addr = 0x80880000

class BoardPinn(Board):
    def __init__(self, socid):
        super(BoardPinn, self).__init__()
        self.socid = socid
        self.board_num = "sdxpinn"
        self.cpu = 'CORTEXA55'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e20000
        self.smem_addr_buildinfo = 0x7e253f8
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.wdog_addr = 0x14680658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardPinnSVM(Board):
    def __init__(self, socid):
        super(BoardPinnSVM, self).__init__()
        self.socid = socid
        self.board_num = "sdxpinnsvm"
        self.cpu = 'CORTEXA55'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e20000
        self.smem_addr_buildinfo = 0x7e253f8
        self.phys_offset = 0xB0000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.wdog_addr = 0x14680658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardPinnFOTAVM(Board):
    def __init__(self, socid):
        super(BoardPinnFOTAVM, self).__init__()
        self.socid = socid
        self.board_num = "sdxpinnfotavm"
        self.cpu = 'CORTEXA55'
        self.ram_start = 0x80000000
        self.smem_addr = 0x7e20000
        self.smem_addr_buildinfo = 0x7e253f8
        self.phys_offset = 0xB8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.wdog_addr = 0x14680658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000
        self.hyp_diag_addr = 0x146AAB30
        self.rm_debug_addr =  0x146AABEC

class BoardPineapple(Board):
    def __init__(self, socid):
        super(BoardPineapple, self).__init__()
        self.socid = socid
        self.board_num = "pineapple"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        #self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardPineappleSVM(Board):
    def __init__(self, socid):
        super(BoardPineappleSVM, self).__init__()
        self.socid = socid
        self.board_num = "pineapplesvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf3800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        #self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardPineappleOEMVM(Board):
    def __init__(self, socid):
        super(BoardPineappleOEMVM, self).__init__()
        self.socid = socid
        self.board_num = "pineappleoemvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf7c00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        #self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardCliffs(Board):
    def __init__(self, socid):
        super(BoardCliffs, self).__init__()
        self.socid = socid
        self.board_num = "cliffs"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardCliffsSVM(Board):
    def __init__(self, socid):
        super(BoardCliffsSVM, self).__init__()
        self.socid = socid
        self.board_num = "cliffssvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf3800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardCliffsOEMVM(Board):
    def __init__(self, socid):
        super(BoardCliffsOEMVM, self).__init__()
        self.socid = socid
        self.board_num = "cliffsoemvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf7c00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class Boardlemans(Board):
    def __init__(self, socid):
        super(Boardlemans, self).__init__()
        self.socid = socid
        self.board_num = "lemans"
        self.cpu = 'CORTEXA78'
        self.ram_start = 0x80000000
        self.smem_addr = 0x900000
        self.smem_addr_buildinfo = 0x9071e0
        self.phys_offset = 0xB0000000
        self.imem_start = 0x14680000
        self.wdog_addr = 0x146BF658
        self.imem_file_name = 'OCIMEM.BIN'
        self.kaslr_addr = 0x146D86d0
        self.imem_offset_memdump_table = 0x2a010
        self.hyp_diag_addr = 0x146D8B30

class BoardCrow(Board):
    def __init__(self, socid):
        super(BoardCrow, self).__init__()
        self.socid = socid
        self.board_num = "crow"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name ='OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardCrowSVM(Board):
    def __init__(self, socid):
        super(BoardCrowSVM, self).__init__()
        self.socid = socid
        self.board_num = "crowsvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xF3800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name ='OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardCrowOEMVM(Board):
    def __init__(self, socid):
        super(BoardCrowOEMVM, self).__init__()
        self.socid = socid
        self.board_num = "crowoemvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xF7C00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146aa6d0
        self.wdog_addr = 0x146aa658
        self.imem_offset_memdump_table = 0x2a010
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardSun(Board):
    def __init__(self, socid):
        super(BoardSun, self).__init__()
        self.socid = socid
        self.board_num = "sun"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xA8000000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardSunSVM(Board):
    def __init__(self, socid):
        super(BoardSunSVM, self).__init__()
        self.socid = socid
        self.board_num = "sunsvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf3800000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

class BoardSunOEMVM(Board):
    def __init__(self, socid):
        super(BoardSunOEMVM, self).__init__()
        self.socid = socid
        self.board_num = "sunoemvm"
        self.cpu = 'ARMV9-A'
        self.ram_start = 0x80000000
        self.smem_addr = 0x1D00000
        self.smem_addr_buildinfo = 0x1D08408
        self.phys_offset = 0xf7c00000
        self.imem_start = 0x14680000
        self.kaslr_addr = 0x146806d0
        self.imem_offset_memdump_table = 0x10
        self.imem_file_name = 'OCIMEM.BIN'
        self.tbi_mask = 0x4000000000

Board9640(socid=234)
Board9640(socid=235)
Board9640(socid=236)
Board9640(socid=237)
Board9640(socid=238)

Board8916(socid=206, smem_addr=0xe200000)
Board8916(socid=206, smem_addr=0x6300000)

Board8939(socid=239, smem_addr=0xe200000)
Board8939(socid=241, smem_addr=0xe200000)
Board8939(socid=239, smem_addr=0x6300000)
Board8939(socid=241, smem_addr=0x6300000)

Board8936(socid=233)
Board8936(socid=240)
Board8936(socid=242)
Board8936(socid=243)

Board8909(socid=245)
Board8909(socid=258)
Board8909(socid=265)

Board8908(socid=259)

Board8929(socid=268, smem_addr=0xe200000)
Board8929(socid=269, smem_addr=0xe200000)
Board8929(socid=270, smem_addr=0xe200000)
Board8929(socid=271, smem_addr=0x6300000)

Board8974(socid=126)
Board8974(socid=184)
Board8974(socid=185)
Board8974(socid=186)
Board8974(socid=208)
Board8974(socid=211)
Board8974(socid=214)
Board8974(socid=217)
Board8974(socid=209)
Board8974(socid=212)
Board8974(socid=215)
Board8974(socid=218)
Board8974(socid=194)
Board8974(socid=210)
Board8974(socid=213)
Board8974(socid=216)

Board9625(socid=134)
Board9625(socid=148)
Board9625(socid=149)
Board9625(socid=150)
Board9625(socid=151)
Board9625(socid=152)
Board9625(socid=173)
Board9625(socid=174)
Board9625(socid=175)


Board8626(socid=145)
Board8626(socid=158)
Board8626(socid=159)
Board8626(socid=198)
Board8626(socid=199)
Board8626(socid=200)
Board8626(socid=205)
Board8626(socid=219)
Board8626(socid=220)
Board8626(socid=222)
Board8626(socid=223)
Board8626(socid=224)

Board8026LW(socid=145)
Board8026LW(socid=158)
Board8026LW(socid=159)
Board8026LW(socid=198)
Board8026LW(socid=199)
Board8026LW(socid=200)
Board8026LW(socid=205)
Board8026LW(socid=219)
Board8026LW(socid=220)
Board8026LW(socid=222)
Board8026LW(socid=223)
Board8026LW(socid=224)

Board8610(socid=147)
Board8610(socid=161)
Board8610(socid=162)
Board8610(socid=163)
Board8610(socid=164)
Board8610(socid=165)
Board8610(socid=166)

Board8974(socid=178, board_num="8084")

Board9635(socid=187)
Board9635(socid=227)
Board9635(socid=228)
Board9635(socid=229)
Board9635(socid=230)
Board9635(socid=231)

Board8960(socid=87, board_num="8960")
Board8960(socid=122, board_num="8960")
Board8960(socid=123, board_num="8260")
Board8960(socid=124, board_num="8060")

Board8960(socid=244, board_num="8064", phys_offset=0x40200000,
                        ram_start=0x40000000)
Board8960(socid=109, board_num="8064")
Board8960(socid=130, board_num="8064")
Board8960(socid=153, board_num="8064")

Board8960(socid=116, board_num="8930")
Board8960(socid=117, board_num="8930")
Board8960(socid=118, board_num="8930")
Board8960(socid=119, board_num="8930")
Board8960(socid=154, board_num="8930")
Board8960(socid=155, board_num="8930")
Board8960(socid=156, board_num="8930")
Board8960(socid=157, board_num="8930")
Board8960(socid=160, board_num="8930")

Board8960(socid=120, board_num="8627")
Board8960(socid=121, board_num="8627")
Board8960(socid=138, board_num="8960")
Board8960(socid=139, board_num="8960")
Board8960(socid=140, board_num="8960")
Board8960(socid=141, board_num="8960")
Board8960(socid=142, board_num="8930")
Board8960(socid=143, board_num="8630")
Board8960(socid=144, board_num="8630")

Board9615(socid=104)
Board9615(socid=105)
Board9615(socid=106)
Board9615(socid=107)

Board8625(socid=88, board_num="8625")
Board8625(socid=89, board_num="8625")
Board8625(socid=96, board_num="8625")
Board8625(socid=90, board_num="8625")
Board8625(socid=91, board_num="8625")
Board8625(socid=92, board_num="8625")
Board8625(socid=97, board_num="8625")
Board8625(socid=98, board_num="8625")
Board8625(socid=99, board_num="8625")
Board8625(socid=100, board_num="8625")
Board8625(socid=101, board_num="8625")
Board8625(socid=102, board_num="8625")
Board8625(socid=103, board_num="8625")
Board8625(socid=127, board_num="8625")
Board8625(socid=128, board_num="8625")
Board8625(socid=129, board_num="8625")
Board8625(socid=131, board_num="8625")
Board8625(socid=132, board_num="8625")
Board8625(socid=133, board_num="8625")
Board8625(socid=135, board_num="8625")

Board8994(socid=207)

Board8992(socid=251)
Board8992(socid=252)

Board8996(socid=246)
Board8996(socid=291)
Board8996(socid=315)
Board8996(socid=316)

Board8952(socid=264)

Board8976(socid=266)
Board8976(socid=274)
Board8976(socid=277)
Board8976(socid=278)

Board9607(socid=290)
Board9607(socid=296)
Board9607(socid=297)
Board9607(socid=298)
Board9607(socid=299)

Board8937(socid=294)
Board8937(socid=295)

Board8940(socid=313)

Board8953(socid=293)
Board8953(socid=304)
Board450(socid=338)
Board632(socid=349)
Board632(socid=350)

Board8917(socid=303)
Board8917(socid=307)
Board8917(socid=308)
Board8917(socid=309)
Board8917(socid=386)

Board8920(socid=320)

BoardCalifornium(socid=279)

BoardCobalt(socid=292)
Board8998(socid=292)

Board660(socid=317)
Board660(socid=324)
Board660(socid=325)
Board660(socid=326)

Board630(socid=318)
Board630(socid=327)

BoardSDM845(socid=321)
BoardMsmnile(socid=339)
BoardSDX20(socid=333)

BoardSdxbaagha(socid=570)
BoardSdxbaagha(socid=571)

BoardSteppe(socid=355)
BoardSteppe(socid=369)
BoardSteppe(socid=365)
BoardSteppe(socid=366)

BoardSdmshrike(socid=405)
BoardSdmshrike(socid=340)

BoardLito(socid=400)
BoardLito(socid=440)
BoardLito(socid=476)

BoardAtoll(socid=407)

BoardLagoon(socid=434)

BoardHoli(socid=454)
BoardBlair(socid=507)

BoardSDM710(socid=336)
BoardSDM710(socid=337)
BoardSDM710(socid=360)

BoardQCS605(socid=347)
BoardQCS405(socid=352)
BoardQCS403(socid=373)

BoardTrinket(socid=394)

BoardPoorwills(socid=334)
BoardPoorwills(socid=335)

BoardPrairie(socid=357)
BoardPrairie(socid=368)

BoardLemur(socid=458)

BoardPinn(socid=556)
BoardPinn(socid=580)
BoardPinnSVM(socid=558)
BoardPinnFOTAVM(socid=558)

Board439(socid=353)
Board439(socid=363)

Board429(socid=354)
Board429(socid=364)

BoardKona(socid=356)
BoardKona(socid=496)

BoardBengal(socid=417)
BoardBengal(socid=444)
BoardBengal(socid=445)
BoardSC7180(socid=407)

BoardKhaje(socid=518)

BoardLahaina(socid=415)
BoardLahainaSVM(socid=415)

BoardWaipio(socid=457)
BoardWaipioSVM(socid=457)

BoardDirewolf(socid=461)

BoardCape(socid=530)
BoardCapeSVM(socid=530)

BoardUkee(socid=591)
BoardUkeeSVM(socid=591)

BoardDiwali(socid=506)
BoardDiwaliSVM(socid=506)

BoardScuba(socid=441)
BoardMonaco(socid=486)
BoardMonaco(socid=517)

BoardShima(socid=450)
BoardShimaSVM(socid=450)
BoardYupik(socid=475)
BoardYupikSVM(socid=475)
BoardYupik(socid=499)
BoardSdxnightjar(socid=484)

BoardKailua(socid=519)
BoardKailuaSVM(socid=519)
BoardKailuaOEMVM(socid=519)

BoardNeo(socid=525)
BoardNeo(socid=554)

BoardParrot(socid=537)
BoardParrotSVM(socid=537)

BoardParrot(socid=613)
BoardParrotSVM(socid=613)

BoardRavelin(socid=568)
BoardRavelinSVM(socid=568)

BoardRavelin(socid=602)
BoardRavelinSVM(socid=602)

BoardLassen(socid=539)
BoardLassen(socid=545)
BoardAnorak(socid=549)

Boardsa8540(socid=461)
Boardsa8540(socid=460)

Boardlemans(socid=532)
Boardlemans(socid=533)
Boardlemans(socid=534)
Boardlemans(socid=535)

BoardPineapple(socid=557)
BoardPineappleSVM(socid=557)
BoardPineappleOEMVM(socid=557)
BoardPineapple(socid=577)
BoardPineappleSVM(socid=577)
BoardPineappleOEMVM(socid=577)

BoardCrow(socid=608)
BoardCrowSVM(socid=608)
BoardCrowOEMVM(socid=608)

BoardCliffs(socid=614)
BoardCliffsSVM(socid=614)
BoardCliffsOEMVM(socid=614)
BoardCliffs(socid=632)
BoardCliffsSVM(socid=632)
BoardCliffsOEMVM(socid=632)

BoardSun(socid=618)
BoardSunSVM(socid=618)
BoardSunOEMVM(socid=618)
