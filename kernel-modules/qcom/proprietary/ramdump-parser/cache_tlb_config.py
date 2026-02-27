"""
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
"""

from cachedumplib import lookuptable as cache_table
from tlbdumplib import lookuptable as tlb_table
from cachedumplib import CacheDumpType_v3
from tlbdumplib import TlbDumpType_v4
from cachedumplib import LLC_SYSTEM_CACHE_KRYO3XX
from cachedumplib import L1_DCache_KRYO4XX_GOLD, L1_ICache_KRYO4XX_GOLD
from cachedumplib import L1_DCache_KRYO4XX_SILVER, L1_ICache_KRYO4XX_SILVER
from cachedumplib import L2_Cache_KRYO4XX_GOLD
from cachedumplib import L1_DCache_KRYO2XX_SILVER, L1_ICache_KYRO2XX_SILVER
from cachedumplib import L1_DCache_KRYO2XX_GOLD, L1_ICache_KRYO2XX_GOLD
from tlbdumplib import L1_DTLB_KRYO4XX_GOLD, L1_ITLB_KRYO4XX_GOLD
from tlbdumplib import L2_TLB_KRYO4XX_GOLD
from tlbdumplib import L2_TLB_KRYO4XX_SILVER
from tlbdumplib import L2_TLB_KRYOBXX_SILVER
from tlbdumplib import L1_TLB_KRYO2XX_GOLD

class L2_VICTIM_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L2_VICTIM_Matterhorn_ELP, self).__init__()
        self.NumSets = 0x200
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn_ELP"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        
class L2_VICTIM_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L2_VICTIM_Matterhorn, self).__init__()
        self.NumSets = 0x200
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
        
class L1_BIM_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L1_BIM_Matterhorn_ELP, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn_ELP"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        
class L1_BIM_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L1_BIM_Matterhorn, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L1_GHB_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L1_GHB_Matterhorn_ELP, self).__init__()
        self.NumSets = 0x400
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn_ELP"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        
class L1_GHB_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L1_GHB_Matterhorn, self).__init__()
        self.NumSets = 0x400
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L1_BTB_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L1_BTB_Matterhorn_ELP, self).__init__()
        self.NumSets = 0x1000
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn_ELP"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        
class L1_BTB_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L1_BTB_Matterhorn, self).__init__()
        self.NumSets = 0x1000
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L0_MOP_CA_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L0_MOP_CA_Matterhorn_ELP, self).__init__()
        self.NumSets = 0xC00
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn_ELP"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        
class L0_MOP_CA_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L0_MOP_CA_Matterhorn, self).__init__()
        self.NumSets = 0x600
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Matterhorn"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
        
class L2_DC_MTE_Klein(CacheDumpType_v3):
    def __init__(self):
        super(L2_DC_MTE_Klein, self).__init__()
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Klein"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L1_DC_MTE_Klein(CacheDumpType_v3):
    def __init__(self):
        super(L1_DC_MTE_Klein, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Klein"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
        
class L1_DC_DIRTY_Klein(CacheDumpType_v3):
    def __init__(self):
        super(L1_DC_DIRTY_Klein, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Klein"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
        
class L1_DCache_Klein_SILVER(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_Klein_SILVER, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Klein"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
 
class L1_ICache_Klein_SILVER(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_Klein_SILVER, self).__init__()
        self.cpu_name = "Klein"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 4
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)

        
class L1_ICache_KRYO5XX_SILVER(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_KRYO5XX_SILVER, self).__init__()
        self.cpu_name = "Kryo5Silver"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 4
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)

class L1_DCache_KRYO5XX_SILVER(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_KRYO5XX_SILVER, self).__init__()
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 2
        self.RegSize = 4
        self.cpu_name = "Kryo5Silver"
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)


class L1_ICache_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_Matterhorn, self).__init__()
        self.cpu_name = "Matterhorn"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.datasubcache = []
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L0_MOP", 1024, 1, 16])
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)
        
# Cortex_A78 == Hercules == Gold cores
class L1_ICache_HERCULES(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_HERCULES, self).__init__()
        self.cpu_name = "Cortex_A78"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.datasubcache = []
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L0_MOP", 1024, 1, 16])
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)

class L2_Cache_Matterhorn(CacheDumpType_v3):
    def __init__(self, numsets):
        super(L2_Cache_Matterhorn, self).__init__()
        self.cpu_name = "Matterhorn"
        self.NumSets = numsets
        self.NumWays = 8
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]

        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, tagfile_name)


class L2_Cache_HERCULES(CacheDumpType_v3):
    def __init__(self, numsets):
        super(L2_Cache_HERCULES, self).__init__()
        self.cpu_name = "Cortex_A78"
        self.NumSets = numsets
        self.NumWays = 8
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]

        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, tagfile_name)

class L1_ITLB_Matterhorn(TlbDumpType_v4):
    def __init__(self):
        super(L1_ITLB_Matterhorn, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn"
        self.NumWays = 1
        self.NumSets = 32
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB_DATA", 0,\
                            datafile_name)
                            
class L1_ITLB_HERCULES(TlbDumpType_v4):
    def __init__(self):
        super(L1_ITLB_HERCULES, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_A78"
        self.NumWays = 1
        self.NumSets = 32
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB_DATA", 0,\
                            datafile_name)

class L1_DTLB_Matterhorn(TlbDumpType_v4):
    def __init__(self):
        super(L1_DTLB_Matterhorn, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn"
        self.NumWays = 1
        self.NumSets = 32
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB_DATA", 0,\
                            datafile_name)
                            
class L1_DTLB_HERCULES(TlbDumpType_v4):
    def __init__(self):
        super(L1_DTLB_HERCULES, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_A78"
        self.NumWays = 1
        self.NumSets = 32
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB_DATA", 0,\
                            datafile_name)

class L2_TLB_Matterhorn(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_Matterhorn, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn"
        #num sets and num ways can be found in json/ directory within this
        #cpu's json file
        self.NumSets = 256
        self.NumWays = 4
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, datafile_name)
        
class L2_TLB_HERCULES(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_HERCULES, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_A78"
        #num sets and num ways can be found in json/ directory within this
        #cpu's json file
        self.NumSets = 256
        self.NumWays = 4
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, datafile_name)

class L1_ICache_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_Matterhorn_ELP, self).__init__()
        self.cpu_name = "Matterhorn_ELP"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.datasubcache = []
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L0_MOP", 2048, 1, 16])
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)
# Cortex_X1 == Hera == Gold+ cores
class L1_ICache_HERA(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_HERA, self).__init__()
        self.cpu_name = "Cortex_X1"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.datasubcache = []
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L0_MOP", 2048, 1, 16])
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)

class L1_DCache_Matterhorn(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_Matterhorn, self).__init__()
        self.cpu_name = "Matterhorn"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)


class L1_DCache_HERCULES(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_HERCULES, self).__init__()
        self.cpu_name = "Cortex_A78"
        self.NumSets = 0x80
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L1_DCache_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_Matterhorn_ELP, self).__init__()
        self.cpu_name = "Matterhorn_ELP"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)
        
class L1_DCache_HERA(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_HERA, self).__init__()
        self.cpu_name = "Cortex_X1"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L1_ICache_KRYO5XX_GOLD(CacheDumpType_v3):
    def __init__(self):
        super(L1_ICache_KRYO5XX_GOLD, self).__init__()
        self.cpu_name = "Kryo5Gold"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.datasubcache = []
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L0_MOP", 1024, 1, 16])
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_I_TAG_DATA", 0, tagfile_name)

class L1_DCache_KRYO5XX_GOLD(CacheDumpType_v3):
    def __init__(self):
        super(L1_DCache_KRYO5XX_GOLD, self).__init__()
        self.cpu_name = "Kryo5Gold"
        self.NumSets = 0x100
        self.NumWays = 4
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]
        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L1_D_TAG_DATA", 0, tagfile_name)

class L2_Cache_KRYO5XX_GOLD(CacheDumpType_v3):
    def __init__(self, numsets):
        super(L2_Cache_KRYO5XX_GOLD, self).__init__()
        self.cpu_name = "Kryo5Gold"
        self.NumSets = numsets
        self.NumWays = 8
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]

        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, tagfile_name)

class L1_ITLB_KRYO5XX_GOLD(TlbDumpType_v4):
    def __init__(self):
        super(L1_ITLB_KRYO5XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Kryo5Gold"
        self.NumWays = 1
        self.NumSets = 48
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB_DATA", 0,\
                            datafile_name)

class L1_DTLB_KRYO5XX_GOLD(TlbDumpType_v4):
    def __init__(self):
        super(L1_DTLB_KRYO5XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Kryo5Gold"
        self.NumWays = 1
        self.NumSets = 48
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB_DATA", 0,\
                            datafile_name)

class L2_TLB_KRYO5XX_GOLD(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_KRYO5XX_GOLD, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Kryo5Gold"
        #num sets and num ways can be found in json/ directory within this
        #cpu's json file
        self.NumSets = 256
        self.NumWays = 5
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, datafile_name)


class L2_TLB_Klein(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_Klein, self).__init__()
        self.cpu_name = "Klein"
        #refer to section A5.2.2 in TRM
        self.NumWays = 4
        self.NumSets = 0x100
        #refer to new src for dumping tag data to see number of tag entries
        self.NumTagRegs = 3
        #refer to new src for dumping tag data to see size. Use bytes
        self.RegSize = 4
        self.tlbsubcache = []
        self.datasubcache = []
        #represet subcache in this form ['name', set, ways, blocksize]
        self.tlbsubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_TAG_WALK",16, 4, 12])
        self.tlbsubcache.append([ "CACHEDUMP_CACHE_ID_L2_TLB_TAG_IPA",16 , 4, 12])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_WALK", 16, 4, 8])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_IPA",16, 4, 8])

    def parse_dump(self):
        tagfile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, tagfile_name)


class L2_TLB_KRYO5XX_SILVER(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_KRYO5XX_SILVER, self).__init__()
        self.cpu_name = "Kryo5Silver"
        #refer to section A5.2.2 in TRM
        self.NumWays = 4
        self.NumSets = 0x100
        #refer to new src for dumping tag data to see number of tag entries
        self.NumTagRegs = 3
        #refer to new src for dumping tag data to see size. Use bytes
        self.RegSize = 4
        self.tlbsubcache = []
        self.datasubcache = []
        #represet subcache in this form ['name', set, ways, blocksize]
        self.tlbsubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_TAG_WALK",16, 4, 12])
        self.tlbsubcache.append([ "CACHEDUMP_CACHE_ID_L2_TLB_TAG_IPA",16 , 4, 12])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_WALK", 16, 4, 8])
        self.datasubcache.append(["CACHEDUMP_CACHE_ID_L2_TLB_DATA_IPA",16, 4, 8])

    def parse_dump(self):
        tagfile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, tagfile_name)


class L2_Cache_Matterhorn_ELP(CacheDumpType_v3):
    def __init__(self, numsets):
        super(L2_Cache_Matterhorn_ELP, self).__init__()
        self.cpu_name = "Matterhorn_ELP"
        self.NumSets = numsets
        self.NumWays = 8
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]

        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, tagfile_name)
        
class L2_Cache_HERA(CacheDumpType_v3):
    def __init__(self, numsets):
        super(L2_Cache_HERA, self).__init__()
        self.cpu_name = "Cortex_X1"
        self.NumSets = numsets
        self.NumWays = 8
        self.NumTagRegs = 1
        self.RegSize = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        tagfile_name = 'MSM_DUMP_DATA' + self.outfile.name[-4:]

        self.kryo_cache_parse("CACHEDUMP_CACHE_ID_L2_TAG_DATA", 0, tagfile_name)


class L1_ITLB_Matterhorn_ELP(TlbDumpType_v4):
    def __init__(self):
        super(L1_ITLB_Matterhorn_ELP, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn_ELP"
        self.NumWays = 1
        self.NumSets = 48
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB_DATA", 0,\
                            datafile_name)
                            
class L1_ITLB_HERA(TlbDumpType_v4):
    def __init__(self):
        super(L1_ITLB_HERA, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_X1"
        self.NumWays = 1
        self.NumSets = 48
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_I_TLB_DATA", 0,\
                            datafile_name)

class L1_DTLB_Matterhorn_ELP(TlbDumpType_v4):
    def __init__(self):
        super(L1_DTLB_Matterhorn_ELP, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn_ELP"
        self.NumWays = 1
        self.NumSets = 42
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB_DATA", 0,\
                            datafile_name)


class L1_DTLB_HERA(TlbDumpType_v4):
    def __init__(self):
        super(L1_DTLB_HERA, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_X1"
        self.NumWays = 1
        self.NumSets = 42
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L1_D_TLB_DATA", 0,\
                            datafile_name)


class L2_TLB_Matterhorn_ELP(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_Matterhorn_ELP, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Matterhorn_ELP"
        #num sets and num ways can be found in json/ directory within this
        #cpu's json file
        self.NumSets = 256
        self.NumWays = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, datafile_name)



class L2_TLB_HERA(TlbDumpType_v4):
    def __init__(self):
        super(L2_TLB_HERA, self).__init__()
        #name must match expected name from kryo tlb parser
        self.cpu_name = "Cortex_X1"
        #num sets and num ways can be found in json/ directory within this
        #cpu's json file
        self.NumSets = 256
        self.NumWays = 8
        self.tlbsubcache = []
        self.datasubcache = []

    def parse_dump(self):
        datafile_name = "MSM_DUMP_DATA" + self.outfile.name[-4:]
        self.kryo_tlb_parse("CACHEDUMP_CACHE_ID_L2_TLB_TAG_DATA", 0, datafile_name)

# "kona"
cache_table[("kona", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("kona", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("kona", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("kona", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("kona", 0x84, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("kona", 0x85, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("kona", 0x86, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("kona", 0x87, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("kona", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("kona", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("kona", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("kona", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("kona", 0x64, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("kona", 0x65, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("kona", 0x66, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("kona", 0x67, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("kona", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("kona", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("kona", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("kona", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("kona", 0xc4, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x200)
cache_table[("kona", 0xc5, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x200)
cache_table[("kona", 0xc6, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x200)
cache_table[("kona", 0xc7, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x400)

#kona
tlb_table[("kona", 0x24, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x25, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x26, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x27, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x44, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x45, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x46, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x47, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("kona", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("kona", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("kona", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("kona", 0x124, 0x14)] = L2_TLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x125, 0x14)] = L2_TLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x126, 0x14)] = L2_TLB_KRYO5XX_GOLD()
tlb_table[("kona", 0x127, 0x14)] = L2_TLB_KRYO5XX_GOLD()

# "lagoon"
cache_table[("lagoon", 0x80, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x81, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x82, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x83, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x84, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x85, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x86, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("lagoon", 0x87, 0x14)] = L1_DCache_KRYO5XX_GOLD()
cache_table[("lagoon", 0x60, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x61, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x62, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x63, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x64, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x65, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("lagoon", 0x66, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("lagoon", 0x67, 0x14)] = L1_ICache_KRYO5XX_GOLD()
cache_table[("lagoon", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lagoon", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lagoon", 0xc6, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x200)
cache_table[("lagoon", 0xc7, 0x10)] = L2_Cache_KRYO5XX_GOLD(numsets=0x200)

#lagoon
tlb_table[("lagoon", 0x26, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("lagoon", 0x27, 0x14)] = L1_ITLB_KRYO5XX_GOLD()
tlb_table[("lagoon", 0x46, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("lagoon", 0x47, 0x14)] = L1_DTLB_KRYO5XX_GOLD()
tlb_table[("lagoon", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("lagoon", 0x126, 0x14)] = L2_TLB_KRYO5XX_GOLD()
tlb_table[("lagoon", 0x127, 0x14)] = L2_TLB_KRYO5XX_GOLD()

# "holi"
cache_table[("holi", 0x80, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x81, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x82, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x83, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x84, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x85, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("holi", 0x86, 0x14)] = L1_DCache_KRYO4XX_GOLD()
cache_table[("holi", 0x87, 0x14)] = L1_DCache_KRYO4XX_GOLD()
cache_table[("holi", 0x60, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x61, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x62, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x63, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x64, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x65, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("holi", 0x66, 0x14)] = L1_ICache_KRYO4XX_GOLD()
cache_table[("holi", 0x67, 0x14)] = L1_ICache_KRYO4XX_GOLD()
cache_table[("holi", 0xc6, 0x10)] = L2_Cache_KRYO4XX_GOLD(numsets=0x200)
cache_table[("holi", 0xc7, 0x10)] = L2_Cache_KRYO4XX_GOLD(numsets=0x200)

#holi
tlb_table[("holi", 0x26, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
tlb_table[("holi", 0x27, 0x14)] = L1_ITLB_KRYO4XX_GOLD()
tlb_table[("holi", 0x46, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
tlb_table[("holi", 0x47, 0x14)] = L1_DTLB_KRYO4XX_GOLD()
tlb_table[("holi", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("holi", 0x126, 0x14)] = L2_TLB_KRYO4XX_GOLD()
tlb_table[("holi", 0x127, 0x14)] = L2_TLB_KRYO4XX_GOLD()

# "blair"
cache_table[("blair", 0x80, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x81, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x82, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x83, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x84, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x85, 0x14)] = L1_DCache_KRYO4XX_SILVER()
cache_table[("blair", 0x86, 0x14)] = L1_DCache_HERCULES()
cache_table[("blair", 0x87, 0x14)] = L1_DCache_HERCULES()
cache_table[("blair", 0x60, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x61, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x62, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x63, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x64, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x65, 0x14)] = L1_ICache_KRYO4XX_SILVER()
cache_table[("blair", 0x66, 0x14)] = L1_ICache_HERCULES()
cache_table[("blair", 0x67, 0x14)] = L1_ICache_HERCULES()
cache_table[("blair", 0xc6, 0x14)] = L2_Cache_HERCULES(numsets=0x200)
cache_table[("blair", 0xc7, 0x14)] = L2_Cache_HERCULES(numsets=0x200)

#blair
tlb_table[("blair", 0x26, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("blair", 0x27, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("blair", 0x46, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("blair", 0x47, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("blair", 0x120, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x121, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x122, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x123, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x124, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x125, 0x14)] = L2_TLB_KRYO4XX_SILVER()
tlb_table[("blair", 0x126, 0x14)] = L2_TLB_HERCULES()
tlb_table[("blair", 0x127, 0x14)] = L2_TLB_HERCULES()

# "lahaina"
cache_table[("lahaina", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x84, 0x14)] = L1_DCache_HERCULES()
cache_table[("lahaina", 0x85, 0x14)] = L1_DCache_HERCULES()
cache_table[("lahaina", 0x86, 0x14)] = L1_DCache_HERCULES()
cache_table[("lahaina", 0x87, 0x14)] = L1_DCache_HERA()
cache_table[("lahaina", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("lahaina", 0x64, 0x14)] = L1_ICache_HERCULES()
cache_table[("lahaina", 0x65, 0x14)] = L1_ICache_HERCULES()
cache_table[("lahaina", 0x66, 0x14)] = L1_ICache_HERCULES()
cache_table[("lahaina", 0x67, 0x14)] = L1_ICache_HERA()
cache_table[("lahaina", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lahaina", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lahaina", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lahaina", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("lahaina", 0xc4, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("lahaina", 0xc5, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("lahaina", 0xc6, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("lahaina", 0xc7, 0x10)] = L2_Cache_HERA(numsets=0x800)

#lahaina
tlb_table[("lahaina", 0x24, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("lahaina", 0x25, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("lahaina", 0x26, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("lahaina", 0x27, 0x14)] = L1_ITLB_HERA()
tlb_table[("lahaina", 0x44, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("lahaina", 0x45, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("lahaina", 0x46, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("lahaina", 0x47, 0x14)] = L1_DTLB_HERA()
tlb_table[("lahaina", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("lahaina", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("lahaina", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("lahaina", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("lahaina", 0x124, 0x14)] = L2_TLB_HERCULES()
tlb_table[("lahaina", 0x125, 0x14)] = L2_TLB_HERCULES()
tlb_table[("lahaina", 0x126, 0x14)] = L2_TLB_HERCULES()
tlb_table[("lahaina", 0x127, 0x14)] = L2_TLB_HERA()

#taro
cache_table[("taro", 0x80, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("taro", 0x81, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("taro", 0x82, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("taro", 0x83, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("taro", 0x84, 0x14)] = L1_DCache_Matterhorn()
cache_table[("taro", 0x85, 0x14)] = L1_DCache_Matterhorn()
cache_table[("taro", 0x86, 0x14)] = L1_DCache_Matterhorn()
cache_table[("taro", 0x87, 0x14)] = L1_DCache_Matterhorn_ELP()
cache_table[("taro", 0x60, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("taro", 0x61, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("taro", 0x62, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("taro", 0x63, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("taro", 0x64, 0x14)] = L1_ICache_Matterhorn()
cache_table[("taro", 0x65, 0x14)] = L1_ICache_Matterhorn()
cache_table[("taro", 0x66, 0x14)] = L1_ICache_Matterhorn()
cache_table[("taro", 0x67, 0x14)] = L1_ICache_Matterhorn_ELP()
cache_table[("taro", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("taro", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("taro", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("taro", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("taro", 0xc4, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("taro", 0xc5, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("taro", 0xc6, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("taro", 0xc7, 0x10)] = L2_Cache_Matterhorn_ELP(numsets=0x800)

tlb_table[("taro", 0x24, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("taro", 0x25, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("taro", 0x26, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("taro", 0x27, 0x14)] = L1_ITLB_Matterhorn_ELP()
tlb_table[("taro", 0x44, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("taro", 0x45, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("taro", 0x46, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("taro", 0x47, 0x14)] = L1_DTLB_Matterhorn_ELP()
tlb_table[("taro", 0x120, 0x14)] = L2_TLB_Klein()
tlb_table[("taro", 0x121, 0x14)] = L2_TLB_Klein()
tlb_table[("taro", 0x122, 0x14)] = L2_TLB_Klein()
tlb_table[("taro", 0x123, 0x14)] = L2_TLB_Klein()
tlb_table[("taro", 0x124, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("taro", 0x125, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("taro", 0x126, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("taro", 0x127, 0x14)] = L2_TLB_Matterhorn_ELP()

tlb_table[("taro", 0x170, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("taro", 0x171, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("taro", 0x172, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("taro", 0x173, 0x14)] = L1_DC_DIRTY_Klein()

tlb_table[("taro", 0x180, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("taro", 0x181, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("taro", 0x182, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("taro", 0x183, 0x14)] = L1_DC_MTE_Klein()

tlb_table[("taro", 0x190, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("taro", 0x191, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("taro", 0x192, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("taro", 0x193, 0x14)] = L2_DC_MTE_Klein()

tlb_table[("taro", 0x1A4, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("taro", 0x1A5, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("taro", 0x1A6, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("taro", 0x1A7, 0x14)] = L0_MOP_CA_Matterhorn_ELP()

tlb_table[("taro", 0x1B4, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("taro", 0x1B5, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("taro", 0x1B6, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("taro", 0x1B7, 0x14)] = L1_BTB_Matterhorn_ELP()

tlb_table[("taro", 0x1C4, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("taro", 0x1C5, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("taro", 0x1C6, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("taro", 0x1C7, 0x14)] = L1_GHB_Matterhorn_ELP()

tlb_table[("taro", 0x1D4, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("taro", 0x1D5, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("taro", 0x1D6, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("taro", 0x1D7, 0x14)] = L1_BIM_Matterhorn_ELP()

tlb_table[("taro", 0x1E4, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("taro", 0x1E5, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("taro", 0x1E6, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("taro", 0x1E7, 0x14)] = L2_VICTIM_Matterhorn_ELP()

#diwali
cache_table[("diwali", 0x80, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("diwali", 0x81, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("diwali", 0x82, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("diwali", 0x83, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("diwali", 0x84, 0x14)] = L1_DCache_Matterhorn()
cache_table[("diwali", 0x85, 0x14)] = L1_DCache_Matterhorn()
cache_table[("diwali", 0x86, 0x14)] = L1_DCache_Matterhorn()
cache_table[("diwali", 0x87, 0x14)] = L1_DCache_Matterhorn_ELP()
cache_table[("diwali", 0x60, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("diwali", 0x61, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("diwali", 0x62, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("diwali", 0x63, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("diwali", 0x64, 0x14)] = L1_ICache_Matterhorn()
cache_table[("diwali", 0x65, 0x14)] = L1_ICache_Matterhorn()
cache_table[("diwali", 0x66, 0x14)] = L1_ICache_Matterhorn()
cache_table[("diwali", 0x67, 0x14)] = L1_ICache_Matterhorn_ELP()
cache_table[("diwali", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("diwali", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("diwali", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("diwali", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("diwali", 0xc4, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("diwali", 0xc5, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("diwali", 0xc6, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("diwali", 0xc7, 0x10)] = L2_Cache_Matterhorn_ELP(numsets=0x800)

tlb_table[("diwali", 0x24, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("diwali", 0x25, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("diwali", 0x26, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("diwali", 0x27, 0x14)] = L1_ITLB_Matterhorn_ELP()
tlb_table[("diwali", 0x44, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("diwali", 0x45, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("diwali", 0x46, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("diwali", 0x47, 0x14)] = L1_DTLB_Matterhorn_ELP()
tlb_table[("diwali", 0x120, 0x14)] = L2_TLB_Klein()
tlb_table[("diwali", 0x121, 0x14)] = L2_TLB_Klein()
tlb_table[("diwali", 0x122, 0x14)] = L2_TLB_Klein()
tlb_table[("diwali", 0x123, 0x14)] = L2_TLB_Klein()
tlb_table[("diwali", 0x124, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("diwali", 0x125, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("diwali", 0x126, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("diwali", 0x127, 0x14)] = L2_TLB_Matterhorn_ELP()

tlb_table[("diwali", 0x170, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("diwali", 0x171, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("diwali", 0x172, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("diwali", 0x173, 0x14)] = L1_DC_DIRTY_Klein()

tlb_table[("diwali", 0x180, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("diwali", 0x181, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("diwali", 0x182, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("diwali", 0x183, 0x14)] = L1_DC_MTE_Klein()

tlb_table[("diwali", 0x190, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("diwali", 0x191, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("diwali", 0x192, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("diwali", 0x193, 0x14)] = L2_DC_MTE_Klein()

tlb_table[("diwali", 0x1A4, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("diwali", 0x1A5, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("diwali", 0x1A6, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("diwali", 0x1A7, 0x14)] = L0_MOP_CA_Matterhorn_ELP()

tlb_table[("diwali", 0x1B4, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("diwali", 0x1B5, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("diwali", 0x1B6, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("diwali", 0x1B7, 0x14)] = L1_BTB_Matterhorn_ELP()

tlb_table[("diwali", 0x1C4, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("diwali", 0x1C5, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("diwali", 0x1C6, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("diwali", 0x1C7, 0x14)] = L1_GHB_Matterhorn_ELP()

tlb_table[("diwali", 0x1D4, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("diwali", 0x1D5, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("diwali", 0x1D6, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("diwali", 0x1D7, 0x14)] = L1_BIM_Matterhorn_ELP()

tlb_table[("diwali", 0x1E4, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("diwali", 0x1E5, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("diwali", 0x1E6, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("diwali", 0x1E7, 0x14)] = L2_VICTIM_Matterhorn_ELP()

#cape
cache_table[("cape", 0x80, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("cape", 0x81, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("cape", 0x82, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("cape", 0x83, 0x14)] = L1_DCache_Klein_SILVER()
cache_table[("cape", 0x84, 0x14)] = L1_DCache_Matterhorn()
cache_table[("cape", 0x85, 0x14)] = L1_DCache_Matterhorn()
cache_table[("cape", 0x86, 0x14)] = L1_DCache_Matterhorn()
cache_table[("cape", 0x87, 0x14)] = L1_DCache_Matterhorn_ELP()
cache_table[("cape", 0x60, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("cape", 0x61, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("cape", 0x62, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("cape", 0x63, 0x14)] = L1_ICache_Klein_SILVER()
cache_table[("cape", 0x64, 0x14)] = L1_ICache_Matterhorn()
cache_table[("cape", 0x65, 0x14)] = L1_ICache_Matterhorn()
cache_table[("cape", 0x66, 0x14)] = L1_ICache_Matterhorn()
cache_table[("cape", 0x67, 0x14)] = L1_ICache_Matterhorn_ELP()
cache_table[("cape", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("cape", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("cape", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("cape", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("cape", 0xc4, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("cape", 0xc5, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("cape", 0xc6, 0x10)] = L2_Cache_Matterhorn(numsets=0x400)
cache_table[("cape", 0xc7, 0x10)] = L2_Cache_Matterhorn_ELP(numsets=0x800)

tlb_table[("cape", 0x24, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("cape", 0x25, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("cape", 0x26, 0x14)] = L1_ITLB_Matterhorn()
tlb_table[("cape", 0x27, 0x14)] = L1_ITLB_Matterhorn_ELP()
tlb_table[("cape", 0x44, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("cape", 0x45, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("cape", 0x46, 0x14)] = L1_DTLB_Matterhorn()
tlb_table[("cape", 0x47, 0x14)] = L1_DTLB_Matterhorn_ELP()
tlb_table[("cape", 0x120, 0x14)] = L2_TLB_Klein()
tlb_table[("cape", 0x121, 0x14)] = L2_TLB_Klein()
tlb_table[("cape", 0x122, 0x14)] = L2_TLB_Klein()
tlb_table[("cape", 0x123, 0x14)] = L2_TLB_Klein()
tlb_table[("cape", 0x124, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("cape", 0x125, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("cape", 0x126, 0x14)] = L2_TLB_Matterhorn()
tlb_table[("cape", 0x127, 0x14)] = L2_TLB_Matterhorn_ELP()

tlb_table[("cape", 0x170, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("cape", 0x171, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("cape", 0x172, 0x14)] = L1_DC_DIRTY_Klein()
tlb_table[("cape", 0x173, 0x14)] = L1_DC_DIRTY_Klein()

tlb_table[("cape", 0x180, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("cape", 0x181, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("cape", 0x182, 0x14)] = L1_DC_MTE_Klein()
tlb_table[("cape", 0x183, 0x14)] = L1_DC_MTE_Klein()

tlb_table[("cape", 0x190, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("cape", 0x191, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("cape", 0x192, 0x14)] = L2_DC_MTE_Klein()
tlb_table[("cape", 0x193, 0x14)] = L2_DC_MTE_Klein()

tlb_table[("cape", 0x1A4, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("cape", 0x1A5, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("cape", 0x1A6, 0x14)] = L0_MOP_CA_Matterhorn()
tlb_table[("cape", 0x1A7, 0x14)] = L0_MOP_CA_Matterhorn_ELP()

tlb_table[("cape", 0x1B4, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("cape", 0x1B5, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("cape", 0x1B6, 0x14)] = L1_BTB_Matterhorn()
tlb_table[("cape", 0x1B7, 0x14)] = L1_BTB_Matterhorn_ELP()

tlb_table[("cape", 0x1C4, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("cape", 0x1C5, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("cape", 0x1C6, 0x14)] = L1_GHB_Matterhorn()
tlb_table[("cape", 0x1C7, 0x14)] = L1_GHB_Matterhorn_ELP()

tlb_table[("cape", 0x1D4, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("cape", 0x1D5, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("cape", 0x1D6, 0x14)] = L1_BIM_Matterhorn()
tlb_table[("cape", 0x1D7, 0x14)] = L1_BIM_Matterhorn_ELP()

tlb_table[("cape", 0x1E4, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("cape", 0x1E5, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("cape", 0x1E6, 0x14)] = L2_VICTIM_Matterhorn()
tlb_table[("cape", 0x1E7, 0x14)] = L2_VICTIM_Matterhorn_ELP()

#yupik
cache_table[("yupik", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("yupik", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("yupik", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("yupik", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("yupik", 0x84, 0x14)] = L1_DCache_HERCULES()
cache_table[("yupik", 0x85, 0x14)] = L1_DCache_HERCULES()
cache_table[("yupik", 0x86, 0x14)] = L1_DCache_HERCULES()
cache_table[("yupik", 0x87, 0x14)] = L1_DCache_HERCULES()
cache_table[("yupik", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("yupik", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("yupik", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("yupik", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("yupik", 0x64, 0x14)] = L1_ICache_HERCULES()
cache_table[("yupik", 0x65, 0x14)] = L1_ICache_HERCULES()
cache_table[("yupik", 0x66, 0x14)] = L1_ICache_HERCULES()
cache_table[("yupik", 0x67, 0x14)] = L1_ICache_HERCULES()
cache_table[("yupik", 0x140, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("yupik", 0x141, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("yupik", 0x142, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("yupik", 0x143, 0x10)] = LLC_SYSTEM_CACHE_KRYO3XX()
cache_table[("yupik", 0xc4, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("yupik", 0xc5, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("yupik", 0xc6, 0x10)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("yupik", 0xc7, 0x10)] = L2_Cache_HERCULES(numsets=0x400)

#yupik
tlb_table[("yupik", 0x24, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("yupik", 0x25, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("yupik", 0x26, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("yupik", 0x27, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("yupik", 0x44, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("yupik", 0x45, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("yupik", 0x46, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("yupik", 0x47, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("yupik", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("yupik", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("yupik", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("yupik", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("yupik", 0x124, 0x14)] = L2_TLB_HERCULES()
tlb_table[("yupik", 0x125, 0x14)] = L2_TLB_HERCULES()
tlb_table[("yupik", 0x126, 0x14)] = L2_TLB_HERCULES()
tlb_table[("yupik", 0x127, 0x14)] = L2_TLB_HERCULES()

#khaje
cache_table[("khaje", 0x80, 0x14)] = L1_DCache_KRYO2XX_SILVER()
cache_table[("khaje", 0x81, 0x14)] = L1_DCache_KRYO2XX_SILVER()
cache_table[("khaje", 0x82, 0x14)] = L1_DCache_KRYO2XX_SILVER()
cache_table[("khaje", 0x83, 0x14)] = L1_DCache_KRYO2XX_SILVER()
cache_table[("khaje", 0x84, 0x14)] = L1_DCache_KRYO2XX_GOLD()
cache_table[("khaje", 0x85, 0x14)] = L1_DCache_KRYO2XX_GOLD()
cache_table[("khaje", 0x86, 0x14)] = L1_DCache_KRYO2XX_GOLD()
cache_table[("khaje", 0x87, 0x14)] = L1_DCache_KRYO2XX_GOLD()
cache_table[("khaje", 0x60, 0x14)] = L1_ICache_KYRO2XX_SILVER()
cache_table[("khaje", 0x61, 0x14)] = L1_ICache_KYRO2XX_SILVER()
cache_table[("khaje", 0x62, 0x14)] = L1_ICache_KYRO2XX_SILVER()
cache_table[("khaje", 0x63, 0x14)] = L1_ICache_KYRO2XX_SILVER()
cache_table[("khaje", 0x64, 0x14)] = L1_ICache_KRYO2XX_GOLD()
cache_table[("khaje", 0x65, 0x14)] = L1_ICache_KRYO2XX_GOLD()
cache_table[("khaje", 0x66, 0x14)] = L1_ICache_KRYO2XX_GOLD()
cache_table[("khaje", 0x67, 0x14)] = L1_ICache_KRYO2XX_GOLD()

# "khaje"
tlb_table[("khaje", 0x120, 0x14)] = L2_TLB_KRYOBXX_SILVER()
tlb_table[("khaje", 0x121, 0x14)] = L2_TLB_KRYOBXX_SILVER()
tlb_table[("khaje", 0x122, 0x14)] = L2_TLB_KRYOBXX_SILVER()
tlb_table[("khaje", 0x123, 0x14)] = L2_TLB_KRYOBXX_SILVER()
tlb_table[("khaje", 0x124, 0x14)] = L1_TLB_KRYO2XX_GOLD()
tlb_table[("khaje", 0x125, 0x14)] = L1_TLB_KRYO2XX_GOLD()
tlb_table[("khaje", 0x126, 0x14)] = L1_TLB_KRYO2XX_GOLD()
tlb_table[("khaje", 0x127, 0x14)] = L1_TLB_KRYO2XX_GOLD()

#Neo
cache_table[("neo", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("neo", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("neo", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("neo", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("neo", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("neo", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("neo", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("neo", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()

#neo
tlb_table[("neo", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("neo", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("neo", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("neo", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()

#parrot
cache_table[("parrot", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("parrot", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("parrot", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("parrot", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("parrot", 0x84, 0x14)] = L1_DCache_HERCULES()
cache_table[("parrot", 0x85, 0x14)] = L1_DCache_HERCULES()
cache_table[("parrot", 0x86, 0x14)] = L1_DCache_HERCULES()
cache_table[("parrot", 0x87, 0x14)] = L1_DCache_HERCULES()
cache_table[("parrot", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("parrot", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("parrot", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("parrot", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("parrot", 0x64, 0x14)] = L1_ICache_HERCULES()
cache_table[("parrot", 0x65, 0x14)] = L1_ICache_HERCULES()
cache_table[("parrot", 0x66, 0x14)] = L1_ICache_HERCULES()
cache_table[("parrot", 0x67, 0x14)] = L1_ICache_HERCULES()
cache_table[("parrot", 0xc4, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("parrot", 0xc5, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("parrot", 0xc6, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("parrot", 0xc7, 0x14)] = L2_Cache_HERCULES(numsets=0x400)

#parrot
tlb_table[("parrot", 0x24, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("parrot", 0x25, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("parrot", 0x26, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("parrot", 0x27, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("parrot", 0x44, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("parrot", 0x45, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("parrot", 0x46, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("parrot", 0x47, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("parrot", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("parrot", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("parrot", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("parrot", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("parrot", 0x124, 0x14)] = L2_TLB_HERCULES()
tlb_table[("parrot", 0x125, 0x14)] = L2_TLB_HERCULES()
tlb_table[("parrot", 0x126, 0x14)] = L2_TLB_HERCULES()
tlb_table[("parrot", 0x127, 0x14)] = L2_TLB_HERCULES()

#anorak
cache_table[("anorak", 0x80, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x81, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x82, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x83, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x84, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x85, 0x14)] = L1_DCache_HERCULES()
cache_table[("anorak", 0x60, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0x61, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0x62, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0x63, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0x64, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0x65, 0x14)] = L1_ICache_HERCULES()
cache_table[("anorak", 0xc0, 0x14)] = L2_Cache_HERCULES(numsets=0x200)
cache_table[("anorak", 0xc1, 0x14)] = L2_Cache_HERCULES(numsets=0x200)
cache_table[("anorak", 0xc2, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("anorak", 0xc3, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("anorak", 0xc4, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("anorak", 0xc5, 0x14)] = L2_Cache_HERCULES(numsets=0x400)

#anorak
tlb_table[("anorak", 0x20, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x21, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x22, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x23, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x24, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x25, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("anorak", 0x40, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x41, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x42, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x43, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x44, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x45, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("anorak", 0x120, 0x14)] = L2_TLB_HERCULES()
tlb_table[("anorak", 0x121, 0x14)] = L2_TLB_HERCULES()
tlb_table[("anorak", 0x122, 0x14)] = L2_TLB_HERCULES()
tlb_table[("anorak", 0x123, 0x14)] = L2_TLB_HERCULES()
tlb_table[("anorak", 0x124, 0x14)] = L2_TLB_HERCULES()
tlb_table[("anorak", 0x125, 0x14)] = L2_TLB_HERCULES()

#ravelin
cache_table[("ravelin", 0x80, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x81, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x82, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x83, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x84, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x85, 0x14)] = L1_DCache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x86, 0x14)] = L1_DCache_HERCULES()
cache_table[("ravelin", 0x87, 0x14)] = L1_DCache_HERCULES()
cache_table[("ravelin", 0x60, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x61, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x62, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x63, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x64, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x65, 0x14)] = L1_ICache_KRYO5XX_SILVER()
cache_table[("ravelin", 0x66, 0x14)] = L1_ICache_HERCULES()
cache_table[("ravelin", 0x67, 0x14)] = L1_ICache_HERCULES()
cache_table[("ravelin", 0xc6, 0x14)] = L2_Cache_HERCULES(numsets=0x400)
cache_table[("ravelin", 0xc7, 0x14)] = L2_Cache_HERCULES(numsets=0x400)

#ravelin
tlb_table[("ravelin", 0x26, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("ravelin", 0x27, 0x14)] = L1_ITLB_HERCULES()
tlb_table[("ravelin", 0x46, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("ravelin", 0x47, 0x14)] = L1_DTLB_HERCULES()
tlb_table[("ravelin", 0x120, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x121, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x122, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x123, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x124, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x125, 0x14)] = L2_TLB_KRYO5XX_SILVER()
tlb_table[("ravelin", 0x126, 0x14)] = L2_TLB_HERCULES()
tlb_table[("ravelin", 0x127, 0x14)] = L2_TLB_HERCULES()
