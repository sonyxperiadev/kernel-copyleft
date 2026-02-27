# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.

import re
import struct
import copy
from typing import Any
from print_out import printd
from print_out import print_out_str

class Param():
    def __init__(self, name, offset, size, prefix="") -> None:
        '''
        parameters to store outputs of
                    'ptype /o struct [Struct_Name]
        each line was a Prarm object
        /* offset    |  size */  type = struct elf64_hdr {
        /*    0      |    16 */    unsigned char e_ident[16];
        /*   16      |     2 */    Elf64_Half e_type;


        :_p_name text: field name  eg: e_type or e_ident
        :_p_offset int: field offset  eg: 16
        :_p_size int: field size  eg: 2
        :_p_prefix text: print indent for sub-struct or sub-field
        :_p_type_name: type name defined in struct eg: Elf64_Half
        :_p_rel_type_name: basic type name, it's used for determine the formats to pack or unpack the data 
                        --> eg: Elf64_Half e_type;
                        -->_p_type_name = Elf64_Half
                        -->_p_rel_type_name = unsigned short
        :_p_data: the raw data content
        '''
        #
        self._p_name = name
        self._p_offset = int(offset)
        self._p_size = size
        self._p_prefix = prefix
        self._p_type_name = "" #elf_gregset_t
        self._p_rel_type_name = "" #long [34]
        self._p_data = 0

class StructObj(Param):
    '''
    represent struct/union structure

    :field_mapping: all the field defined in the structure

    dict  attr_name --> FieldObj
    '''
    def __init__(self, type_name, offset, size, prefix) -> None:
        super().__init__("", offset, size, prefix)
        self._p_type_name = type_name
        self.field_mapping = {}

    def add_field(self, attr_name, fieldobj):
        self.field_mapping[attr_name] = fieldobj

    def fields(self):
        ''' retrun field list'''
        return self.field_mapping

    def get_field(self, attr_name):
        return self.field_mapping[attr_name]

    def __repr__(self) -> str:
        '''
        string output for object
        used for str() or repr()
        '''
        ret ="/* {} | {} */ {}   {} {}=".format(
            self._p_offset, self._p_size, self._p_prefix, self._p_type_name, self._p_name)
        ret += "{\n"

        for obj in self.field_mapping.values():
            obj._p_prefix = self._p_prefix + "   "
            ret += repr(obj)
        ret += self._p_prefix + "}\n"
        return ret

    def __getattr__(self, name):
        if "__" in name: ## private attribute
            name = "__" + name.split("__")[-1]

        obj = self.field_mapping[name]
        if isinstance(obj, StructObj):
            return obj
        else:
            return obj._p_data

    def __setattr__(self, __name: str, __value: Any):
        # struct task_struct->__state
        # this field was a private field in python
        # here need to pre-handle it.
        if "__" in __name:
            __name = "__" + __name.split("__")[-1]
        try:
            if __name in self.__dict__["field_mapping"].keys():
                obj = self.__dict__["field_mapping"][__name]
                if isinstance(obj, FieldObj):
                    self.__dict__["field_mapping"][__name]._p_data = __value
                else:
                    raise Exception("Set to {} {} was not allowed".format(self._p_type_name, self._p_name))
            else:
                super().__setattr__(__name, __value)
        except:
            return super().__setattr__(__name, __value)

    def __bytes__(self):
        '''
        generate bytes for object
        used for bytes(object)
        '''
        pbytes = bytearray(b'\x00' * self._p_size)
        for obj in self.field_mapping.values():
            pos_s = obj._p_offset-self._p_offset
            pos_e = pos_s + obj._p_size
            o_bytes =bytes(obj)
            pbytes[pos_s : pos_e] = o_bytes

        if len(pbytes) != self._p_size:
            print_out_str("++++++++ ERROR ++++++++++++++++")
            print_out_str(self)
            raise Exception("{} {} size is incorrect {} expected {}".format(
                self._p_type_name, self._p_name, len(pbytes), self._p_size))
        return bytes(pbytes)

    def __deepcopy__(self, src):
        dst = StructObj(self._p_type_name, self._p_offset, self._p_size, self._p_prefix)
        for key, value in self.__dict__.items():
            setattr(dst, key, copy.deepcopy(value))
        return dst

class FieldObj(Param):
    '''
    represent a field in structure
    '''
    def __init__(self, ramdump, name, offset, size, type_name, rel_type_name, prefix="") -> None:
        super().__init__(name, offset, size, prefix)

        self._p_type_name = type_name
        self._p_rel_type_name = rel_type_name
        self.is_array_type = False
        self.preset_array_field()
        # formats to pack or unpack for sturct
        if ramdump:
            self.formats = self.unpack_formats(ramdump)

    def unpack_formats(self, ramdump):
        fmt = None
        size = self._p_size
        if self.is_array_type:
            size = int(self._p_size / len(self._p_data))

        try:
            fmt = getattr(ramdump, "_RamDump__unpack_format")(size, self._p_rel_type_name)
            fmt = fmt.strip() ## to test fmt is None
        except Exception as e:
            msg = "type {} rel type {} size {}".format(self._p_type_name, self._p_rel_type_name, size)
            raise Exception(msg)
        return fmt

    def get_array_len(self):
        # unsigned char e_ident[16];
        array_len = 0
        match = re.search("(.*)\[(\d+)\]", self._p_name)
        if match:
            self._p_name = match.group(1)
            array_len = int(match.group(2))
        ## ptype elf_gregset_t type = unsigned long [34]
        if self._p_rel_type_name != self._p_type_name:
            match = re.search("(.*)\[(\d+)\]", self._p_rel_type_name)
            if match:
                self._p_rel_type_name = match.group(1)
                array_len = int(match.group(2)) if array_len == 0 else int(match.group(2)) * array_len
        return array_len

    def preset_array_field(self):
        ### specifix for __uint128_t
        array_len = self.get_array_len()
        if array_len >1:
            item_size = int(self._p_size /array_len)
            if item_size > 8:
                printd(self, "{} has a huge type {} size:{} need a list to store data".format(
                                self._p_data, self._p_type_name, self._p_size ))
                array_len = int(item_size / 8) * array_len
        elif self._p_size > 8:
            printd(self, "{} has a huge type {} size:{} need a list to store data".format(
                                self._p_data, self._p_type_name, self._p_size ))
            array_len = int(self._p_size / 8)

        if array_len > 1:
            self.is_array_type = True
            self._p_data = [0 for i in range(0,  array_len)]

    def fill_data(self, dbytes):
        if self.is_array_type:
            object_size = int(self._p_size / len(self._p_data))
            for i in range(0, len(self._p_data)):
                self._p_data[i] = struct.unpack(self.formats, dbytes[(i * object_size) : ((i+1) * object_size)])[0]
        else:
            self._p_data = struct.unpack(self.formats, dbytes)[0]

    def __bytes__(self):
        dbytes = b''

        if self.is_array_type:
            for data in self._p_data:
                dbytes += struct.pack(self.formats, data)
        else:

            dbytes += struct.pack(self.formats, self._p_data)

        if len(dbytes) != self._p_size:
            print_out_str("++++++++ ERROR ++++++++++++++++")
            print_out_str(self)
            raise Exception("Fieldobj {} {}__bytes size is incorrect {} expected {}"
                            .format(self._p_type_name, self._p_name, len(dbytes), self._p_size))

        return dbytes

    def __repr__(self) -> str:
        ret = "/* {} | {} {} */ {}   {} {}= ".format(
            self._p_offset, self._p_size, self.formats,
            self._p_prefix, self._p_type_name, self._p_name)

        if self._p_rel_type_name == "char" and type(self._p_data) == list:
            #char * array
            ret += "{} ".format(bytes(self._p_data).decode('ascii', 'ignore').split("\0")[0])
            ret += "    "+"{}".format(self._p_data)
        elif self._p_rel_type_name == "char" and type(self._p_data) == int:
            if "unsigned" in self._p_rel_type_name:
                ret += "{:c}".format(self._p_data)
            else:
                ret += "{}".format(self._p_data)
        elif type(self._p_data) == int:
            if self._p_name in ["pr_pid", "pr_uid", "pr_gid", "pr_ppid", "pr_pgrp", "pr_sid"]:
                ret += "{}".format(self._p_data)
            else:
                ret += "0x{:x}".format(self._p_data)
        elif type(self._p_data) == bytes:
            ret += "{}".format(self._p_data)
        else:
            ret += "{}".format(self._p_data)
        ret += "\n"

        return ret

    def __deepcopy__(self, src):
        dst = FieldObj(None, self._p_name, self._p_offset, self._p_size,
                       self._p_type_name, self._p_rel_type_name, self._p_prefix)
        for key, value in self.__dict__.items():
            setattr(dst, key, copy.deepcopy(value))
        return dst

class StructParser:
    def __init__(self, ramdump) -> None:
        self.ramdump = ramdump
        self.datatype_dict = {}
        self.addr_struct_dict = {}

    def read_struct(self, struct_addr, type_name, attr_list=None):
        size = self.ramdump.sizeof(type_name)
        data = getattr(self.ramdump, "_RamDump__get_bin_data")(struct_addr, size)
        if not data:
            raise Exception("Error!! read_struct get None data from address 0x{} with size {}".format(
                            struct_addr, size))
        if len(data) != size:
            raise Exception("Error!! read_struct get data from address 0x{} with size {}, but got size {}".format(
                struct_addr, size, len(data)))

        var_obj = self.parser_struct_def(type_name)
        self.fill_pdata(var_obj, data, attr_list)

        _p_data = bytes(var_obj)
        if len(_p_data) != len(data):
            raise Exception("Error!! read_struct size is not same {} {}".format(len(_p_data), len(data)))
        self.addr_struct_dict[struct_addr] = var_obj
        #return copy.deepcopy(var_obj)
        return var_obj

    def parser_struct_def(self, type_name, attr_list=None):
        size = self.ramdump.sizeof(type_name)
        var_obj, vsize, tem_p_name = self.__parser_struct_def(type_name)

        if vsize != size:
            raise Exception("sizeof({}) size={} parser type info got invalid size {}".format(
                type_name, size, vsize))

        if not var_obj:
            raise Exception("Parse {} type failed".format(type_name))
        return copy.deepcopy(var_obj)

    def fill_pdata(self, var_type, data, attr_list=None):
        for attr_name, fieldobj in var_type.field_mapping.items():
            if attr_list and attr_name not in attr_list:
                continue
            if isinstance(fieldobj, StructObj):
                self.fill_pdata(fieldobj, data)
            elif isinstance(fieldobj, FieldObj):
                fieldobj.fill_data(data[fieldobj._p_offset: fieldobj._p_offset + fieldobj._p_size])

    def __parser_struct_def(self, the_type, offset=0):
        """
        Function to return type info for the type.

        :param the_type: type of the structure field.
        :type the_type: str

        :return: d_type, size
        """
        if the_type in self.datatype_dict.keys():
            return self.datatype_dict[the_type]
        else:
            text = []
            try:
                text = self.ramdump.gdbmi.getStructureData(the_type)
                size = self.ramdump.sizeof(the_type)
            except Exception as e:
                raise Exception("Parse %s failed!! %s" % (the_type, str(e)))

            if text:
                d_type = text[0].split("type = ")[1]
                d_type = d_type.replace("{", "").strip()

                d_type = getattr(self.ramdump, "_RamDump__ignore_storage_class")(d_type)
                d_type = getattr(self.ramdump, "_RamDump__ignore_expanded_pointer")(text, d_type)
                if d_type == "struct":
                    '''
                    (gdb) ptype atomic_t
                            type = struct {
                                int counter;
                            }
                    '''
                    d_type = the_type
                if not self.is_general_type(d_type):
                    master_obj, attr_name, _ = self.__create_object(text, offset, 0, size)
                    self.datatype_dict[the_type] = master_obj, size, attr_name
                    return master_obj, size, attr_name
                self.datatype_dict[the_type] = d_type, size, None
                return d_type, size, None
            return None, 0, None

    def is_general_type(self, datatype):
        if "*" in datatype:
            # a pointer type
            return True
        datalist = self.ramdump.gdbmi.getStructureData(datatype)
        if datalist[-1].strip() == "} *" and self.ramdump.sizeof(datatype) == 8:
            # struct pointer type eg: ptype lockdep_map_p
            return True
        return len(datalist) == 1

    def __create_object(self, text, base_offset, curr_index, obj_size, prefix="",):
        '''
        Function to create a python object from the gdb text output with meta data
        like size and offset of all the members, needed to populate the values from
        the binary dump files.

        :param text: text gdb output for a particular symbol/type.
        :type the_type: str

        :param base_offset: base offset value.
        :type field: int

        :param curr_index: current line index in 'text'.
        :type field: int

        :return: py object created based on 'text', array check flag, current index
        /* offset    |  size */  type = struct elf_prstatus_common {
        /*    0      |    12 */    struct elf_siginfo {
        '''

        if curr_index == 0:
            d_type = text[0].split("{")[0]
        else:
            d_type = text[curr_index-1].split("{")[0]

        d_type = d_type.split("[")[0]
        d_type = d_type.split("*/")[-1].split("type =")[-1].strip()

        newclass = type(d_type, (StructObj,), {})
        curr_obj = newclass(d_type, base_offset, obj_size, prefix)

        curr_offset = base_offset
        total_size = len(text)
        size = 0
        attr_name = None

        while total_size > curr_index:
            line = text[curr_index]
            curr_index = curr_index + 1
            if line is None:
                break
            if "(" in line and ")" in line:
                continue
            if re.search("\/\* offset\s+\|\s+ size \*\/", line.lstrip().rstrip()):
                continue

            is_struct, curr_offset, size = self.is_struct_field(line, curr_offset)
            if is_struct:
                ### structobj, struct name,
                obj, attr_name, curr_index = self.__create_object(text, curr_offset, curr_index, size, prefix + "    ")
                '''
                /* 2176      |     0 */    struct syscall_user_dispatch {
                            <no data fields>
                               /* total size (bytes):    0 */
                           } syscall_dispatch;
                '''
                ## put condition here
                if size == 0:
                    continue
                if attr_name is not None:
                    curr_obj.add_field(attr_name, obj)
                else:
                    # adding anonimous union members to parent
                    for attr, value in obj.field_mapping.items():
                        curr_obj.add_field(attr, value)
            else:
                is_field, curr_offset, size, datatype, attr_name = self.is_gen_field(line, curr_offset)
                #   /* 1088      |     0 */    unsigned long cpu_bitmap[];
                if size == 0: #skip padding
                    continue
                if is_field:
                    if ")(" in datatype:
                        attr_name = datatype.split(")(")[0].split("(")[1]
                    if attr_name.lstrip()[0] == '*':
                        datatype = datatype + " *"
                        attr_name = attr_name.lstrip('*')

                    curr_field = None
                    if not self.is_general_type(datatype):
                        #  /*  112      |   272 */    elf_gregset_t pr_reg;
                        var_type = self.parser_struct_def(datatype)
                        var_type._p_name = attr_name
                        var_type._p_prefix = prefix + "    "
                        var_type._p_offset = curr_offset
                        self.adjust_offset(var_type, curr_offset)
                        curr_obj.add_field(attr_name, var_type)
                    else:
                        newclass = type(attr_name, (FieldObj,), {})
                        if "*" in datatype:
                            rel_type = datatype
                        else:
                            rel_type = self.ramdump.gdbmi.getStructureData(datatype)[0].split("type = ")[1]
                        curr_field = newclass(self.ramdump, attr_name, curr_offset, size, datatype, rel_type)

                        #setattr(curr_obj, attr_name, curr_field)
                        curr_obj.add_field(curr_field._p_name, curr_field)
                    continue
                re_obj = re.search('\s*} (\S+);', line)
                if re_obj is not None:
                    curr_obj._p_name = re_obj.group(1)
                    return curr_obj, re_obj.group(1), curr_index
                re_obj = re.search('\s*};', line)
                if re_obj:
                    return curr_obj, None, curr_index
                re_obj = re.search('\s*}\s*(\[\d+\])', line)
                if re_obj:
                    return curr_obj, re_obj.group(1), curr_index
        # None means unnamed union or struct
        return curr_obj, None, curr_index

    def adjust_offset(self, var_type, offset):
        for _name, fieldobj in var_type.field_mapping.items():
            fieldobj._p_offset += offset
            if isinstance(fieldobj, StructObj):
                self.adjust_offset(fieldobj, offset)

    def is_struct_field(self, line, curr_offset):
        # sample match : "/*    0      |    40 */    struct thread_info {"
        re1 = re.search('\s+(\d+)\s+[|]\s+(\d+) \*\/\s+(struct|union) .*{', line)
        if re1:
            curr_offset = int(re1.group(1))
            size = int(re1.group(2))
            return True, curr_offset, size
        # sample match : "/*                 8 */            struct {"
        re2 = re.search('\/\*\s+(\d+) \*\/\s+(struct|union) .*{', line)
        if re2:
            size = int(re2.group(1))
            return True, curr_offset, size
        return False, curr_offset, None

    def is_gen_field(self, line, curr_offset):
        # sample match : "/*   20      |     4 */                u32 need_resched;"
        re1 = re.search('/\*\s+(\d+)\s+[|]\s+(\d+)\s\*/\s+([^:]+) (\S+);', line)
        if re1 is not None:
            curr_offset = int(re1.group(1))
            size = int(re1.group(2))
            datatype = re1.group(3)
            attr_name = (re1.group(4))
            return True, curr_offset, size, datatype, attr_name
        # sample match : "/*                 4 */    uint32_t v;"
        re2 = re.search('/\*\s+(\d+)\s\*/\s+([^:]+) (\S+);', line)
        if re2 is not None:
            size = int(re2.group(1))
            datatype = re2.group(2)
            attr_name = (re2.group(3))
            return True, curr_offset, size, datatype, attr_name
        return False, curr_offset, None, None, None