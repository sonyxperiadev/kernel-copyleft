#!/usr/bin/env python
# Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import optparse
import sys
import os
import json
import math
import ctypes
import re

from collections import OrderedDict

DEBUG_LEVEL = 0
DEFAULT_HEX = 0
DEBUG_check_string_bitfield_is_1 = 0
DEBUG_get_header_str = 0
BITS_IN_CHAR = 8
DEBUG_POST_PROCESS = 2

CONFIG_PREPEND_ZEROES_TO_MSB = True

_BLOCKSIZE_STR_ = "blockSize"
_NUMSETS_STR_ = "numSets"
_ASSOCIATIVITY_STR_ = "associativity"
_CACHETYPELIST_STR_ = "cacheTypes"
_STRUCTURE_STR_ = "structure"
_PARENT_STR_ = "parent"
_FIELDS_STR_ = "fields"
_BITFIELD_STR_ = "bitfield"
_NAME_STR_ = "name"
_HEX_STR_ = "hex"
_POST_PROCESS_STR_ = "post_process"
_SUBCACHE_STR_ = "subCache"
_OFFSET_STR_ = "offset"
_NEXT_STR_ = "next"
_VALUE_MAP_STR_ = "valueMap"

# Cachedump header related constants
# Magic strings were obtained by computing the sha256sum of "CACHEDUMP HEADER"
# Top 4 bytes of the sha256sum were considered
__CACHEDUMP_HEADER_MAGIC_STR__ = [0x9620edbb, 0xa8865c52, 0x027dead8, 0x41c43418]

# This is highest version of cachedump header supported by this script
MAX_CACHEDUMP_HEADER_VERSION_SUPPORTED = 1


# Cachedump Header structure definition (same as library)
class Header(ctypes.LittleEndianStructure):
    """
    Helper class to use C style structures
    """
    _fields_ = [
        ("magic_val_0", ctypes.c_uint32),
        ("magic_val_1", ctypes.c_uint32),
        ("magic_val_2", ctypes.c_uint32),
        ("magic_val_3", ctypes.c_uint32),
        ("header_size", ctypes.c_uint32),
        ("header_version", ctypes.c_uint32),
        ("valid", ctypes.c_uint32),
        ("cache_type", ctypes.c_uint32),
        ("dump_size", ctypes.c_uint32),
        ("errcode", ctypes.c_uint32),
        ("midr", ctypes.c_uint32),
        ("mpidr", ctypes.c_uint32),
        ("revidr", ctypes.c_uint32),
        ("soc_id", ctypes.c_uint32),
        ("lib_commit_id", ctypes.c_uint64),
        ("time_to_save", ctypes.c_uint64)
        # End of Version 1 of the header
    ]

    def populate(self, bytes):
        """
        This function takes a byte string as input and
        populates the cachedump header object
        :param bytes: Byte string
        :return: No return value
        """
        mem_size = min(len(bytes), ctypes.sizeof(self))
        ctypes.memmove(ctypes.addressof(self), bytes, mem_size)


# Global Header object to instantiate cachedump header data
cachedump_header = Header()
header_string = ""

# Output file list (for merging TAG_DATA outputs)
output_file_list = []


class Field():
    """
    This class and its methods are used to process individual fields
    in each cache dump.
    """
    def __init__(self, Dict):
        """
        This is the constructor for the Field class.

        It takes care of the initialization of fields for each Field object.
        Each Field object describes exactly one field entry for the given cache type.

        :param Dict: A dictionary containing a single field (of a single cache-type) from the JSON
        """
        self.name = Dict[_NAME_STR_]
        self.bitfield = Dict[_BITFIELD_STR_]
        self.value_map = dict()
        if _HEX_STR_ in Dict.keys() and int(Dict[_HEX_STR_]) == 1:
            self.displayBase = 16
        else:
            self.displayBase = 10
        if _POST_PROCESS_STR_.replace(' ', '_').lower() in [k.replace(' ', '_').lower() for k in Dict.keys()]:
            dict_ordered = OrderedDict(Dict)
            i = [k.replace(' ', '_').lower() for k in dict_ordered.keys()].index(
                _POST_PROCESS_STR_.replace(' ', '_').lower())
            self.post_process_list = dict_ordered[dict_ordered.keys()[i]]
        else:
            self.post_process_list = None
        if _VALUE_MAP_STR_ in Dict.keys():
            temp = Dict[_VALUE_MAP_STR_]
            for key, value in temp.items():
                self.value_map[int(key)] = value
        else:
            self.value_map = None

    def get_field_width(self):
        """
        This function returns the width of this Field object, as specified in the JSON.
        If a "fields" entry does not exist in the JSON, the width is 8 hex characters (4 bytes) by default.

        :return: Width of the Field object (This is the width that will be displayed in the output file)
        """
        return max(self.get_name_width(), self.get_bitfield_width())

    def get_name_width(self):
        """
        This function is used to return the width of this Field object's name.

        This is required in order to have a more readable output file, where the field values
        match the width of the field name.

        :return: Width of field name
        """
        return max(len(self.name.strip()), self.get_max_value_width())

    def get_post_processed_width(self, width):
        """
        For some fields such as the PA field in TAGRAMs, there is some post-processing
        which is required for the field to be more meaningful. In such cases, this function
        computes the width of the post processed string.

        :param width: Width of the field before processing
        :return: Width of the field after processing
        """
        for operation in self.post_process_list:
            for operator in operation.keys():
                if "<<" in operator:
                    try:
                        width = width + eval(str(operation[operator]))
                    except TypeError:
                        sys.stderr.write(str(operation[operator]) + '\n')
                elif ">>" in operator:
                    try:
                        width = width - eval(str(operation[operator]))
                    except TypeError:
                        sys.stderr.write(str(operation[operator]) + '\n')
                elif "sign_extend" in operator.strip().lower().replace(' ', '_'):
                    try:
                        extended_width = eval(str(operation[operator]['width']))  # type: int
                        width = extended_width if (extended_width > width) else width
                    except TypeError:
                        sys.stderr.write(str(operation[operator]['width']) + '\n')
        return width

    def get_max_value_width(self):
        """
        This function is used to compute the maximum width of all possible string values in a
        valueMap. This is needed for a neater formatting in the output file.

        :return: Length of the longest string in a valueMap
        """
        max_len = 0
        if self.value_map is not None:
            for key, val in self.value_map.items():
                if len(val) > max_len:
                    max_len = len(val)
        return max_len

    def get_bitfield_width(self):
        """
        This function computes the width of each field that is printed in the
        output file. It makes sure that the widest possible value that could be
        printed under a field is set as the default width for that field.

        It is the most important function needed for ensuring a neatly formatted, readable
        output file.

        :return: Width of this Field object.
        """
        display_width = 0
        if DEBUG_LEVEL >= 2:
            print "func: get_bitfield_width"
            print "    : name = " + self.name
            print "    : list = " + str(self.get_bitfield_list())
            print "    : width = " + str(len(self.get_bitfield_list()))
            print "    : displayBase = " + str(self.displayBase)
        width = len(self.get_bitfield_list())  # no. of bits in the field
        width_1 = 0
        width_2 = 0
        if self.post_process_list is not None:
            width_1 = self.get_post_processed_width(width)
        width_2 = self.get_max_value_width()
        width = max(width_1, width_2, width)

        if self.displayBase == 16:
            display_width = int((width + 3) / 4)
        else:
            display_width = int(math.ceil(math.log10((1 << width))))
        if DEBUG_LEVEL >= 2:
            print "    : display_width = " + str(display_width)
        return display_width


    def post_process_value(self, res, set_num=None, way_num=None):
        """
        This function is responsible for performing any post processing (if any) on the
        parsed data retrieved from the input binary file.
        (Ex: Post processing of TAGRAM entries to obtain a more meaningful PA)

        Such post-processing operations are defined under fields in
        the JSON file for each cache and for each CPU.

        :param res: The binary dump as read by the parser
        :param set_num: Set number
        :param way_num: Way number
        :return: Post-processed value obtained by processing res
        """
        for operation in self.post_process_list:
            for operator in operation.keys():
                if '<<' in operator:
                    try:
                        res = res << eval(str(operation[operator]))
                    except TypeError:
                        sys.stderr.write(str(operation[operator]) + '\n')
                elif '>>' in operator:
                    try:
                        res = res >> eval(str(operation[operator]))
                    except TypeError:
                        sys.stderr.write(str(operation[operator]) + '\n')
                elif '|' in operator or '&' in operator:
                    try:
                        operand = 0

                        # find whether set or way
                        if 'set' in [k.strip().lower() for k in operation[operator].keys()]:
                            src = set_num
                        elif 'way' in [k.strip().lower() for k in operation[operator].keys()]:
                            src = way_num

                        # extract the bitfield
                        try:
                            bf = operation[operator]['Set']
                        except:
                            sys.stderr.write('could not interpret bitfield for operator ' + str(
                                operator) + ' in post-process-list ' + str(
                                self.post_process_list) + ' for field ' + self.name)
                            exit(-1)
                        for i in range(min([int(k) for k in bf.split(':')]), max([int(k) for k in bf.split(':')]) + 1):
                            if ((1 << i) & src): operand = (operand + (1 << i))

                        if '<<' in operation[operator].keys():
                            shift_val = operation[operator]['<<']
                            operand = operand << shift_val
                        res = eval(str(res) + operator + str(operand))
                    except:
                        sys.stderr.write('Error occurred in post_process list for field:' + self.name + '\n')
                        exit(-1)
                elif 'sign_extend' in operator.strip().lower().replace(' ', '_'):
                   extended_width = eval(str(operation[operator]['width']))  # type: int
                   sign_bit = eval(str(operation[operator]['sign_bit']))  # type: int
                   extension_start = eval(str(operation[operator]['extend_start']))  # type: int
                   sign_extend_needed = (res >> sign_bit) & 1
                   if sign_extend_needed:
                       sign_extension_width = (extended_width - extension_start)
                       mask = (((1 << sign_extension_width) - 1) << extension_start) if sign_extension_width >= 1 else 0
                       res = mask | res
        return res

    def get_aligned_name(self, str_type=None):
        """
        This function is used to obtain an aligned header string.
        An aligned header string is the first line of each output file.
        Each field name in this string is aligned as per the values that the field can take.

        :param str_type: To choose if this is a header/entry string
        :return: Width string that will be passed to the str.format() function.
        """
        if str_type is 'header':
            return self.get_width_string(str_type='header').format(self.name.replace(' ', '_'))
        else:
            return self.get_width_string().format(self.name.replace(' ', '_'))

    def get_width_string(self, str_type=None):
        """
        This function is used to return the width of non-hexadecimal strings.
        The return value is passed to the str.format() function in order to generate fixed
        width strings.

        :param str_type: header/normal entry
        :return: Width string
        """
        if CONFIG_PREPEND_ZEROES_TO_MSB and str_type is not 'header' and self.post_process_list is not None:
            return '{:0>' + str(self.get_field_width()) + '}'
        else:
            return '{:>' + str(self.get_field_width()) + '}'

    def get_width_hex_string(self):
        """
        This function is used to return the width of hexadecimal strings.
        The return value is passed to the str.format() function in order to generate fixed
        width hex strings.

        :return: Width string
        """
        if CONFIG_PREPEND_ZEROES_TO_MSB:
            return '{:0>' + str(self.get_field_width()) + 'x}'
        else:
            return '{:>' + str(self.get_field_width()) + 'x}'

    def get_bitfield_list(self):
        """
        This function is used to retrieve bitfields for this cache Field.
        These are defined in the JSON under the field entry, for each cache/CPU.

        :return: A list containing bitfields for this Field.
        """
        bitfield_list = []
        field_list = self.bitfield.split(',')
        for f in field_list:
            if not f.split():
                continue
            if f.find(':') == -1:
                bitfield_list.append(int(eval(f)))
            else:
                nums = f.split(':')
                num1 = int(eval(nums[0]))
                num2 = int(eval(nums[1]))
                for i in range(min(num1, num2), max(num1, num2) + 1):
                    bitfield_list.append(i)
        return bitfield_list

    def substitute_bitfield_value(self, val):
        """
        This function accepts a key (the raw value of a Field) and maps it
        to the matching string value in the valueMap dictonary of the JSON entry.

        These string values are as per the core's TRM.

        This function will work if the key in the valueMap dictionary is either a
        decimal representation or a binary representation (Ex: It will work with
        keys '110' as well as '6').

        Point of caution: The string values are stored in unicode. So be sure to
        convert them into str using the str() method to avoid any confusion!

        :param val: Field value to be referenced in valueMap
        :return: String value referenced from valueMap
        """
        for value, uni_str in self.value_map.items():
            temp = str(value)
            try:
                temp2 = int(temp, 2)
            except ValueError:
                temp2 = value
            if val == temp2:
                return str(uni_str)
        # If there is no mapping for the passed value, just return the value back
        return str(val)

    def get_aligned_bitfield(self, data, set_num=None, way_num=None):
        """
        This is the top level function that takes care of aligning bitfields for each
        cache field as per the width of the values that the field can take. It invokes
        various other width based functions in order to compute the correct output width.

        :param data: Cache line contents (in binary format)
        :param set_num: Set number
        :param way_num: Way number
        :return:
        """
        res = 0
        bitfield_list = self.get_bitfield_list()  # type: List[Field]
        bitfield_list.sort()
        for i in range(len(bitfield_list)):
            if check_string_bitfield_is_1(data, bitfield_list[i]):
                res = res + (1 << i)
        if self.post_process_list:
            res = self.post_process_value(res, set_num, way_num)
        if self.value_map:
            res = self.substitute_bitfield_value(res)
        if self.displayBase == 16:
            width_string = self.get_width_hex_string()
        else:
            width_string = self.get_width_string()
        ret_str = width_string.format(res)
        if DEBUG_LEVEL == -1:
            print ret_str
        return ret_str


def check_string_bitfield_is_1(s, bf):
    """
    This function checks if the 'i'th position in the str bitfield is 1.
    It returns a value of 1 if the bitfield is 1, else 0.

    :param s: Cache line contents
    :param bf: Bitfield being considered in the cache line
    :return: 1, if bf has value 1 in it, else 0
    """
    ret_val = 0
    lsb = 0
    msb = 0
    for i in range(0, len(s)):
        lsb = BITS_IN_CHAR * i
        msb = lsb + BITS_IN_CHAR - 1
        if msb >= bf >= lsb:
            if ord(s[i]) & (1 << (bf % BITS_IN_CHAR)):
                ret_val = 1
            break
    if DEBUG_check_string_bitfield_is_1:
        sys.stderr.write("func: check_string_bitfield_is_1: string=" + ' '.join(
            '{:02x}'.format(ord(ch)) for ch in s) + ' bitfield=' + str(bf) + ' res=' + str(ret_val))
        sys.stderr.write("    : lsb = " + str(lsb))
        sys.stderr.write("    : msb = " + str(msb))
        sys.stderr.write("    : 1 << (bf % BITS_IN_CHAR) = " + str(1 << (bf % BITS_IN_CHAR)))
        sys.stderr.write("    : ord(s[i]) = " + str(ord(s[i])))
    return ret_val


def get_header_str(field_list):
    """
    This function is responsible for generating the header string, which will go into the
    output file.

    If the _FIELDS_STR_ key is present for the given cache's JSON entry, the header string
    will contain those fields as is. For cases where this entry does not exist, the header
    string is simply 4-byte offset labels.

    :param field_list: List of cache fields, as returned by the get_cache_fields(...) function
    :return: The header string for this cache's output file
    """
    ret_str = ''
    for field in field_list:
        if DEBUG_get_header_str:
            sys.stderr.write(field.get_aligned_name())
        ret_str = ret_str + field.get_aligned_name(str_type='header') + ' '
    return ret_str


def verify_json_file_sanity(filename):
    """
    This is a helper function.
    The function checks if the a given file is a valid json file.

    :param filename: Name of the JSON file to be validated
    :return: True if filename is a valid JSON file, else False
    """
    ret_val = True
    with open(filename) as fd:
        data = fd.read().replace('\n', '')
        try:
            json.loads(data)
        except:
            sys.stderr.write("Error: Invalid json file: " + filename + "\n")
            exit(-1)
            ret_val = False
    return ret_val


def search_json_obj_for_attribute(jsonObj, cacheType, attribute):
    """
    This is a helper function.

    Dictionaries consist of key:value pairs. This function returns the value
    for a given key in the passed json object(if it exists).

    :param jsonObj: Dictionary (might be nested) that is obtained from the json file
    :param cacheType: Cache type (One that is supported by the target CPU)
    :param attribute: Attribute to be searched for in json dictionary
    :return: Value for the provided attribute (key) in the json object (if it exists), else None
    """
    if DEBUG_LEVEL >= 3:
        print ("\n\nentered function " + "search_json_obj_for_attribute")
        print "jsonObj = " + "\n" + str(jsonObj)
        print "cacheType = " + cacheType
        print "attribute = " + str(attribute)

    ret_val = None
    dict_type = type({})
    if cacheType in jsonObj.keys():
        if type(jsonObj[cacheType]) == dict_type and _STRUCTURE_STR_ in jsonObj[cacheType].keys():
            try:
                if type(jsonObj[cacheType][_STRUCTURE_STR_]) == dict_type:
                    if attribute in jsonObj[cacheType][_STRUCTURE_STR_].keys():
                        ret_val = jsonObj[cacheType][_STRUCTURE_STR_][attribute]
                    elif _PARENT_STR_ in jsonObj[cacheType][_STRUCTURE_STR_].keys():
                        ret_val = jsonObj[jsonObj[cacheType][_STRUCTURE_STR_][_PARENT_STR_]][_STRUCTURE_STR_][attribute]
            except:
                if DEBUG_LEVEL >= 2:
                    print sys.exc_info()[0]
                ret_val = None
    return ret_val


def search_json_file_for_attribute(filename, target_cpu, cache_type, attribute, json_type):
    """
    This is a helper function.

    It looks within a JSON file for a given attribute (key).

    :param filename: JSON file within which search is to be performed
    :param target_cpu: Target CPU
    :param cache_type: Cache type (One that is supported by the target CPU)
    :param attribute: Attribute (key) to be looked for in the json file
    :param json_type: (Unused) Used to specify if it is a CPU/SOC JSON file
    :return: Value for the provided attribute (key) in the json object (if it exists), else None
    """
    ret_val = None
    if DEBUG_LEVEL >= 3:
        print ("\n\nentered function " + "search_json_file_for_attribute")
        print "filename = " + filename
        print "targetCpu = " + target_cpu
        print "cacheType = " + cache_type
        print "attribute = " + str(attribute)
        print "json_type = " + json_type

    if json_type == "cpu":
        # we know that the cpu specific json file is sane
        with open(filename) as fd:
            cpu_data_obj = fd.read().replace('\n', '')
            cpu_data = json.loads(cpu_data_obj)
        ret_val = search_json_obj_for_attribute(cpu_data, cache_type, attribute)
    elif json_type == "soc":
        verify_json_file_sanity(filename)
        try:
            with open(filename) as fd:
                soc_data_obj = fd.read().replace('\n', '')
                soc_data = json.loads(soc_data_obj)
            for socCpu in soc_data.keys():
                if target_cpu.lower() == socCpu.lower():
                    ret_val = search_json_obj_for_attribute(soc_data[socCpu], cache_type, attribute)
        except:
            print sys.exc_info()[0]
            ret_val = None
    if DEBUG_LEVEL >= 2:
        print "retVal = " + str(ret_val)
    return ret_val


def search_for_json_file(filename):
    """
    This is a helper function.

    It looks for a {filename}.json in the json directory of the cachedump parser repository.

    :param filename: The name of the file to be searched
    :return: Absolute path to the file if the file is found, else None
    """
    for f in os.listdir(os.path.join(os.path.dirname(os.path.realpath(__file__)),os.path.join('extensions','json'))):
        if f.lower() == (filename.lower() + ".json"):
            return os.path.abspath(os.path.join(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.path.join('extensions','json')), f))
    return None


def extract_element_from_file(filename, seek_offset, block_size, Set, set_max, Way, way_max):
    """
    This function is invoked from cache_dump_parse(...).

    This function is used to read the input binary file and return the cache
    line for the specified seek_offset + (set, way).

    :param filename: Input binary which contains the cache dumps
    :param seek_offset: Offset after which the required dump exists in the file
    :param block_size: Size of cache line, in bytes
    :param Set: Maximum number of sets in the cache
    :param set_max: Set number of cache line to be retrieved
    :param Way: Maximum number of ways in the cache
    :param way_max: Way number of cache line to be retrieved
    :return: Byte string containing required cache line
    """
    with open(filename, "rb") as fd:
        offset = seek_offset + Way * (set_max * block_size) + (Set * block_size)
        fd.seek(offset)
        elem = fd.read(block_size)
    return elem


def parse_cache_line(line, cache_fields, set_num=None, way_num=None):
    """
    This function is used to parse a single cache line in the binary.

    :param line: The cache line contents, passed as a binary string
    :param cache_fields: Fields (if any) present for this cache type
    :param set_num: Set number
    :param way_num: Way number
    :return: A string containing the expanded cacheline. This string is written into the output file.
    """
    ret_str = ''
    for field in cache_fields:
        ret_str = ret_str + field.get_aligned_bitfield(line, set_num, way_num) + ' '
    return ret_str.rstrip()


def get_cache_fields(cpuDataFile, cacheType):
    """
    This function is invoked from cache_dump_parse(...).

    It retrieves information about the fields that are defined for each cache line.

    If the _FIELDS_STR_ entry exists in the json, it returns those key value pairs.
    If the _FIELDS_STR_ does not exist in the json, it returns the structure in which the
    data from the input binary will be dumped.

    :param cpuDataFile: json containing cache data for the provided target CPU
    :param cacheType: Cache type (One that is supported by the target CPU)
    :return: A list of Field type objects
    """
    fields = []
    verify_json_file_sanity(cpuDataFile)
    with open(cpuDataFile) as fd:
        cpu_data_obj = fd.read().replace('\n', '')
        cpu_data = json.loads(cpu_data_obj)
    try:
        if type(cpu_data[cacheType][_FIELDS_STR_]) == type([]):
            for obj in cpu_data[cacheType][_FIELDS_STR_]:
                fields.append(Field(obj))
    except KeyError:
        sys.stderr.write("Cannot find any 'fields' key in cache description, dumping binary data\n")
        for i in range(int(cpu_data[cacheType][_STRUCTURE_STR_][_BLOCKSIZE_STR_]) / 4):  # Display groups of 4 Bytes
            fields.append(
                Field(
                    {"name": "{:>8x}".format(i*4),
                     "bitfield": "{}:{}".format(BITS_IN_CHAR * 4 * i, (BITS_IN_CHAR * 4 * (i + 1)) - 1), "hex": "1"}))
    return fields


def get_cache_structure(cpu_data_file, target_cpu, cache_type, opt_num_sets,
                        opt_associativity, opt_soc):
    """
    This function is invoked from cache_dump_parse(...).

    It serves the purpose of returning information about the structure of the specified cache,
    such as the number of sets and ways, the block size and so on. This information is used to
    dump the exact dump data for the specified cache.

    In some cases, we may have certain smaller caches that will be need to be parsed as a part
    of a larger cache (Ex: L0 MOP cache is parsed along with the L1-I Data cache in Deimos).
    For these instances, this function also returns the offset from the start of the larger
    cache's dump.

    :param cpu_data_file: json containing cache data for the provided target CPU
    :param target_cpu: CPU type
    :param cache_type: Cache type (One that is supported by the target CPU)
    :param opt_num_sets: [Optional] Number of sets in the cache type passed
    :param opt_associativity: [Optional] Number of ways in the cache type passed
    :param opt_soc: [Unused] SOC type (Maps to SOC file)
    :return: (block_size, num_sets, associativity, offset)
    """
    num_sets = 0
    associativity = 0
    block_size = 0
    offset = 0

    # num sets
    if opt_num_sets:
        num_sets = int(opt_num_sets)
    elif opt_soc and search_for_json_file(opt_soc) and search_json_file_for_attribute(search_for_json_file(opt_soc),
                                                                                      target_cpu,
                                                                                      cache_type, _NUMSETS_STR_, "soc"):
        num_sets = search_json_file_for_attribute(search_for_json_file(opt_soc), target_cpu, cache_type, _NUMSETS_STR_,
                                                  "soc")
    else:
        num_sets = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _NUMSETS_STR_, "cpu")
    if isinstance(num_sets, basestring):
        try:
            num_sets = eval(num_sets)
        except:
            sys.stderr.write(
                "Cannot evaluate property:" + _NUMSETS_STR_ + ' cache:' + cache_type + ' cpu:' + target_cpu + '\n')
            exit(-1)
    # associativity
    if opt_associativity:
        associativity = int(opt_associativity)
    elif opt_soc and search_for_json_file(opt_soc) and search_json_file_for_attribute(search_for_json_file(opt_soc),
                                                                                      target_cpu,
                                                                                      cache_type, _ASSOCIATIVITY_STR_,
                                                                                      "soc"):
        associativity = search_json_file_for_attribute(search_for_json_file(opt_soc), target_cpu, cache_type,
                                                       _ASSOCIATIVITY_STR_, "soc")
    else:
        associativity = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _ASSOCIATIVITY_STR_,
                                                       "cpu")
    if isinstance(associativity, basestring):
        try:
            associativity = eval(associativity)
        except:
            sys.stderr.write(
                'Cannot evaluate property:' + _ASSOCIATIVITY_STR_ + ' cache:' + cache_type + ' cpu:' + target_cpu + '\n')
            exit(-1)

    # number of bytes in each dump element
    block_size = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _BLOCKSIZE_STR_, "cpu")
    if isinstance(block_size, basestring):
        try:
            block_size = eval(block_size)
        except:
            sys.stderr.write(
                "Cannot evaluate property:" + _BLOCKSIZE_STR_ + ' cache:' + cache_type + ' cpu:' + target_cpu + '\n')
            exit(-1)

    is_sub_cache = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _SUBCACHE_STR_, "cpu")
    if is_sub_cache == 'True':
        offset = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _OFFSET_STR_, "cpu")
        if isinstance(offset, basestring):
            try:
                offset = eval(offset)
            except:
                sys.stderr.write("Cannot evaluate property:" + _OFFSET_STR_ +
                                 ' cache:' + cache_type + ' cpu:' + target_cpu + '\n')
                exit(-1)

    return block_size, num_sets, associativity, offset


def cache_dump_parse(input_filename, output_filename, target_cpu, cpu_data_file, seek_offset, cache_type, opt_num_sets,
                     opt_associativity, opt_soc):
    """
    This is the top level parsing function that is invoked from main().

    It obtains pre-validated command line arguments from main(), and uses them
    to parse a given input binary for a given cache type and a given CPU, at a
    specified offset.

    In some cases, it may be desirable to pass the number of sets and ways from the
    command line. For such instances, the optional parameters opt_num_sets and opt_associativity
    can be used.

    The output is written into an output file named output_filename + cache_type[19:0]

    :param input_filename: Input binary dump file
    :param output_filename: Output file prefix
    :param target_cpu: CPU type
    :param cpu_data_file: json containing cache data for the provided target CPU
    :param seek_offset: Offset within the input binary from which the parsing is to be performed
    :param cache_type: One of the cache types that is supported by the target CPU
    :param opt_num_sets: [Optional] Number of sets in the cache type passed
    :param opt_associativity: [Optional] Number of ways in the cache type passed
    :param opt_soc: [Unused] SOC type (Maps to SOC file)
    :return: This function does not return a value
    """
    global output_file_list

    with open(cpu_data_file) as fd:
        cpu_data_obj = fd.read().replace('\n', '')
        original_stdout = sys.stdout
        original_output_filename = output_filename
    done = False

    total_parsed_bytes = 0

    while not done:
        output_filename = original_output_filename

        # Get the output file handle
        if output_filename is not None:
            output_filename = output_filename + "_" + cache_type[19:]
            output_file_list.append(output_filename)
            sys.stdout = open(output_filename, 'w+')

        # Get the cache structure: sets, ways, Bytes/line
        (block_size, num_sets, associativity, offset) = get_cache_structure(cpu_data_file, target_cpu,
                                                                            cache_type, opt_num_sets, opt_associativity,
                                                                            opt_soc)

        if offset != 0:
            seek_offset = seek_offset + offset

        try:
            total_bytes = block_size * num_sets * associativity
            total_parsed_bytes = total_parsed_bytes + total_bytes
        except TypeError:
            sys.stderr.write('Type block_size    = ' + str(type(block_size)) + '\n')
            sys.stderr.write('Type num_sets      = ' + str(type(num_sets)) + '\n')
            sys.stderr.write('Type associativity = ' + str(type(associativity)) + '\n')
            exit(-1)

        # print cache structure information
        sys.stderr.write("\n" + "CPU: " + target_cpu)
        sys.stderr.write("\n" + "CacheType: " + cache_type)
        sys.stderr.write("\n" + _BLOCKSIZE_STR_ + ": " + str(block_size))
        sys.stderr.write("\n" + _NUMSETS_STR_ + ": " + str(num_sets))
        sys.stderr.write("\n" + _ASSOCIATIVITY_STR_ + ": " + str(associativity))
        sys.stderr.write("\n" + "Total cache size" + ": " + str(total_bytes))
        sys.stderr.write("\n" + "Input File: " + input_filename)
        sys.stderr.write("\n" + "Seek Offset: " + str(seek_offset))
        sys.stderr.write("\n" + "Output File: " + output_filename if output_filename is not None else "stdout")
        sys.stderr.write("\n")

        # Get the individual fields of the cache line
        cache_fields = get_cache_fields(cpu_data_file, cache_type)

        # Check if we exceed the file bounds
        if (seek_offset + total_bytes) > os.path.getsize(input_filename):
            sys.stderr.write("Error: exceeding file boundary for cache-type " + cache_type + "\n")
            sys.exit(-1)

        print(header_string)
        header_str = get_header_str(cache_fields)
        print('Way  Set ' + header_str)

        for way in range(associativity):
            for Set in range(num_sets):
                elem = extract_element_from_file(input_filename, seek_offset, block_size, Set, num_sets, way, associativity)
                field_str = parse_cache_line(elem, cache_fields, Set, way)
                print '{:>3}'.format(str(way)) + ' ' + '{:>4}'.format(
                    str(Set)) + ' ' + field_str

        # restore stdout
        if output_filename is not None:
            sys.stdout = original_stdout

        res = search_json_file_for_attribute(cpu_data_file, target_cpu, cache_type, _NEXT_STR_, "cpu")

        if res is None:
            done = True
        else:
            cache_type = res

    return total_parsed_bytes


def is_cache_supported(cpu_data_file, cache_type):
    """
    This is a helper function, which is used to check if the passed cache type
    is supported by the CPU described by the cpu_data_file. The function simply
    a 0/1 value, indicating False/True. This value is to be used in the calling
    function (this function does not abort execution).

    :param cpu_data_file: Name of JSON file containing CPU specific data.
    :param cache_type: Name of the cache to be parsed
    :return: True/False, indicating whether the cache is supported by this CPU.
    """
    with open(cpu_data_file) as fd:
        cpu_data_obj = fd.read().replace('\n', '')
        try:
            cpu_data = json.loads(cpu_data_obj)
        except:
            sys.stderr.write("Error(1): Invalid json file: " + cpu_data_file + "\n")
            sys.exit(-1)
        cache_types = cpu_data.keys()
    cache_type_found = 0
    for c in cache_types:
        if c.lower() == cache_type.lower():
            cache_type_found = 1
            break
    return cache_type_found, cache_types


def get_cache_type(cache_id):
    """
    This is a helper function to fetch the Cache Type, given a
    Cache ID.

    The Cache Type is used to validate if a certain cache is supported
    by the CPU, and fetch size and field information for the same.

    :param cache_id: Base-10 integer cache ID value
    :return: Cache-Type (string)
    """
    formatted_cache_id = "{:08x}".format(cache_id)
    cache_id_map_file = search_for_json_file("Cache_ID_Map")
    with open(cache_id_map_file) as f:
        cache_id_map = json.load(f)
    try:
        retval = cache_id_map[formatted_cache_id.upper()]
        return retval
    except KeyError:
        sys.stderr.write("\n[Header] Cache Type {} not supported\n".format(formatted_cache_id))
        sys.stderr.write("[Header] Value will be used from cmdline, if provided along with -O option.\n")
        return None


def get_cpu_type(midr):
    """
    This is a helper function to fetch the CPU Type, given an
    ARM MIDR value.

    The CPU Type is used to reference the JSON files which contain
    cache size information.

    :param midr: Base-10 integer MIDR value
    :return: CPU-Type (string)
    """
    formatted_midr = hex(midr & 0xFF0FFFF0).upper()[2:-1]
    midr_map_file = search_for_json_file("MIDR_Map")
    with open(midr_map_file) as f:
        midr_map = json.load(f)
    try:
        retval = midr_map[formatted_midr]
        return retval
    except KeyError:
        sys.stderr.write("\n[Header] MIDR {} not supported\n".format(formatted_midr))
        sys.stderr.write("[Header] Value will be used from cmdline, if provided along with -O option.\n")
        return None


def check_magic_string():
    """
    This is a helper function to check for magic strings.

    Magic strings are fixed width constants which mark the beginning
    of every cachedump header. The same string is defined by both the
    library and the parser.

    :param str_list: String list containing 4 byte chunks of the (potential) magic value
    :return: True if it is a match, else False
    """
    for i in range(0, 4):
        if __CACHEDUMP_HEADER_MAGIC_STR__[i] != eval("cachedump_header.magic_val_{}".format(i)):
            return False
    return True


def validate_cachedump_header(input_bin_file, offset):
    """
    This is a helper function to validate if a cachedump header is present
    as a part of the dumped data.

    :param input_bin_file: Binary file containing the dump (from cmdline)
    :return: True, if a header is present, else False
    """
    with open(r'{}'.format(input_bin_file), "rb") as f:
        ret = False
        f.seek(offset)
        byte_string = f.read(ctypes.sizeof(cachedump_header))
        cachedump_header.populate(byte_string)
        header_present = check_magic_string()
        if header_present:
            ret = True
            if cachedump_header.valid == 0:
                sys.stderr.write("\n [HEADER] Invalid dump.\n")
                sys.exit(-1)
            if cachedump_header.header_version > MAX_CACHEDUMP_HEADER_VERSION_SUPPORTED:
                sys.stderr.write("\n[HEADER] Version {} not supported. Use -O option with parameters.\n".format(cachedump_header.header_version))
                ret = False
        return ret


def create_and_delete_tag_data_output(tag_file, data_file):
    """
    This is a helper function to create the merged TAG_DATA output file and get rid of the individual TAG and DATA
    files. This function is invoked by merge_tag_data_dumps().

    If both the TAG and DATA parts of the output are available, this function merges both those files and gets rid
    of the individual files.

    :param tag_file: Path to output TAG file
    :param data_file: Path to output DATA file
    """
    merge_file_contents = ''
    header_obtained = False

    # Create merged file contents by parsing tag and data files
    with open(r'{}'.format(data_file)) as data, open(r'{}'.format(tag_file)) as tag:
        data_lines = data.readlines()
        tag_lines = tag.readlines()
        for d, t in zip(data_lines, tag_lines):
            if 'Way' in t and 'Set' in t:
                header_obtained = True

            if header_obtained:
                row = t
                d_list = re.split(r'(\s+)', d)
                if d_list[0] == '':
                    d_list = d_list[1:]
                d_list = d_list[4:]
                d_list = [d_list[i] + d_list[i + 1] for i in range(0, len(d_list) - 1, 2)]
                d = ''.join(d_list)
                row += d
                row = row.replace('\n', ' ')
                row += '\n'
                merge_file_contents += row
            else:
                merge_file_contents += t

    # Get merged file name and path
    tag_file_name = os.path.basename(tag_file)
    dir_path = os.path.dirname(tag_file)
    tag_list = tag_file_name.split('_')
    idx = tag_list.index('TAG')
    tag_list = tag_list[:idx + 1] + ['DATA'] + tag_list[idx + 1:]
    merged_file_name = '_'.join(tag_list)
    merged_file_name = os.path.join(dir_path, merged_file_name)

    # Write to merged file
    with open(r'{}'.format(merged_file_name), 'w') as fd:
        fd.write(merge_file_contents)

    # Remove individual TAG and DATA files
    os.remove(tag_file)
    os.remove(data_file)

    sys.stderr.write("Merge successful!\n")
    sys.stderr.write("Output file: {}\n\n".format(os.path.abspath(merged_file_name)))


def merge_tag_data_dumps():
    """
    This is a post-processing function that is used to merge TAG and DATA output files for the same cache type.
    Specifically, this function is called when a TAG_DATA cache type is detected.

    This function compiles two lists: one for the output paths to the TAG files and the other for the output paths
    for the DATA files. It passes elements from these lists to the create_and_delete_tag_data_output() function
    for processing.

    This function is invoked from main().
    """
    tag_list = []
    data_list = []

    sys.stderr.write("\nTAG_DATA cache type detected!\n")
    sys.stderr.write("Attempting to merge output into single file...\n")

    for filename in output_file_list:
        if "TAG" in filename:
            tag_list.append(filename)
        elif "DATA" in filename:
            data_list.append(filename)

    # Abort if no TAG files are created
    if not tag_list:
        sys.stderr.write("TAG file(s) not found. Aborting merge!\n")
        return
    # Abort if no DATA files are created
    elif not data_list:
        sys.stderr.write("DATA file(s) not found. Aborting merge!\n")
        return
    # Abort if an unequal number of DATA and TAG files are created
    elif len(tag_list) != len(data_list):
        sys.stderr.write("TAG and DATA output mismatch. Aborting merge!\n\n")
        return

    for t, d in zip(tag_list, data_list):
        create_and_delete_tag_data_output(t, d)


def main():
    """
    This is the main function. Execution begins from here.

    The purpose of this function is to obtain and validate command line arguments.
    If valid, the command line arguments are passed to the cache_dump_parse(...) function.

    :return: This function does not return a value
    """
    global DEBUG_LEVEL
    input_filename = ''
    output_filename = ''
    target_cpu = ''
    seek_offset = 0
    cache_type = ''

    # Define cmd-line args
    parser = optparse.OptionParser()
    parser.add_option("-i", "--input", dest="input_filename",
                      help="Input binary dump file")
    parser.add_option("-O", "--override", dest="override", action='store_true', default=False,
                      help="Override header values with command line values")
    parser.add_option("-o", "--output", dest="output_filename",
                      help="Output file")
    parser.add_option("-c", "--cpu", dest="target_cpu",
                      help="CPU name")
    parser.add_option("-s", "--seek", dest="seek_offset",
                      help="Offset within input file (in bytes)")
    parser.add_option("-t", "--type", dest="cache_type",
                      help="Cache type")
    parser.add_option("--sets", dest="num_sets",
                      help="Number of sets(cache index lines)")
    parser.add_option("--ways", dest="associativity",
                      help="Number of ways(cache associativity)")
    parser.add_option("--soc", dest="soc",
                      help="SOC number(for cache configuration detection)")
    parser.add_option("-D", "--debug", dest="debug_level",
                      help="Level of debug information sought")

    (options, args) = parser.parse_args()

    DEBUG_LEVEL = options.debug_level
    # Check input file sanity
    if not options.input_filename or not os.path.isfile(options.input_filename):
        sys.stderr.write("Error: incorrect input file\n")
        sys.exit(-1)
    else:
        input_filename = options.input_filename
    if DEBUG_LEVEL >= 1:
        sys.stderr.write("input_filename = " + input_filename + "\n")

    # Check output file sanity
    if options.output_filename is None:
        sys.stderr.write("No output file provided; writing to stdout\n")
    else:
        output_dirname = os.path.dirname(os.path.abspath(options.output_filename))
        if not os.access(output_dirname, os.W_OK):
            sys.stderr.write("Error: No write permissions for output file\n")
            sys.exit(-1)
    if DEBUG_LEVEL >= 1:
        sys.stderr.write("output_filename = " + str(options.output_filename) + "\n")

    # Check validity of seek offset
    try:
        seek_offset_int = eval(options.seek_offset)
    except:
        sys.stderr.write("seek offset is not an integer, please see usage for details")
        sys.exit(-1)
    if os.path.getsize(input_filename) < seek_offset_int:
        sys.stderr.write("Error: Seek offset exceeds file size\n")
        if DEBUG_LEVEL >= 2:
            sys.stderr.write("filesize of " + str(input_filename) + " = " + str(os.path.getsize(input_filename)) + "\n")
            sys.stderr.write("offset = " + str(options.seek_offset) + "\n")
            sys.stderr.write("type filesize = " + str(type(os.path.getsize(input_filename))))
        sys.exit(-1)
    else:
        seek_offset = seek_offset_int
    if DEBUG_LEVEL >= 2:
        sys.stderr.write("offset = " + str(seek_offset) + "\n")

    # This function call checks if a cachedump header is present
    header_present = validate_cachedump_header(options.input_filename, seek_offset)

    if header_present and DEBUG_LEVEL >= 1:
        sys.stderr.write("Header found\n")
    if not header_present and DEBUG_LEVEL >= 1:
        sys.stderr.write("Header not found\n")

    # Check CPU sanity
    if header_present:
        if not options.override or options.target_cpu == None:
            options.target_cpu = get_cpu_type(cachedump_header.midr)
    if options.target_cpu is None:
        sys.stderr.write("Error: Please provide a valid CPU\n")
        sys.exit(-1)

    cpu_found = 0
    cpu_data_file = search_for_json_file(options.target_cpu)
    if cpu_data_file: cpu_found = 1
    if not cpu_found:
        sys.stderr.write("Error: CPU not supported\n")
        sys.exit(-1)
    if DEBUG_LEVEL >= 1:
        sys.stderr.write("cpu_data_file = " + cpu_data_file + "\n")

    # Check whether the cache type is supported by the particular CPU
    if header_present:
        if not options.override or options.cache_type == None:
            options.cache_type = get_cache_type(cachedump_header.cache_type)
    if options.cache_type is None:
        sys.stderr.write("Error: Please provide a valid cache type\n")
        sys.exit(-1)

    cache_type_found, cache_types = is_cache_supported(cpu_data_file, options.cache_type)
    if not cache_type_found:
        sys.stderr.write("Error: Cache type " + options.cache_type + " not supported by CPU\n")
        sys.stderr.write("Cache types supported are\n")
        sys.stderr.write("\n".join(cache_types) + "\n")
        sys.exit(-1)

    if header_present:
        global header_string
        seek_offset = seek_offset + cachedump_header.header_size
        header_string = "Header information from dump:" \
                 + "\n\tHeader size: " + str(hex(cachedump_header.header_size).rstrip("L") \
                 + "\n\tVersion: " + str(cachedump_header.header_version)) \
                 + "\n\tValid: " + str(cachedump_header.valid) \
                 + "\n\tCache ID: " +  str(hex(cachedump_header.cache_type)).rstrip("L") + ' (' + str(options.cache_type) + ')'\
                 + "\n\tDump size: " +  str(hex(cachedump_header.dump_size)).rstrip("L") \
                 + "\n\tError code: " +  str(cachedump_header.errcode) \
                 + "\n\tMIDR: " +  str(hex(cachedump_header.midr)).rstrip("L") + ' (' + str(options.target_cpu) + ')' \
                 + "\n\tMPIDR: " +  str(hex(cachedump_header.mpidr)).rstrip("L") \
                 + "\n\tREVIDR: " +  str(hex(cachedump_header.revidr)).rstrip("L") \
                 + "\n\tSOC ID: " +  str(hex(cachedump_header.soc_id)).rstrip("L") \
                 + "\n\tCommit ID: " +  str(hex(cachedump_header.lib_commit_id)).rstrip("L") \
                 + "\n\tTime taken to dump: " +  str(cachedump_header.time_to_save) + " ns" \
                 + "\n"

    if "TAG_DATA" in options.cache_type:
        tag = options.cache_type.split('_TAG_DATA')[0] + '_TAG'
        tag_exists, dummy_list = is_cache_supported(cpu_data_file, tag)
        if tag_exists:
            tag_size = cache_dump_parse(input_filename, options.output_filename, options.target_cpu, cpu_data_file, seek_offset,
                             tag.upper(), options.num_sets, options.associativity, options.soc)
            seek_offset = seek_offset + tag_size
        data = options.cache_type.split('_TAG_DATA')[0] + '_DATA'
        data_exists, dummy_list = is_cache_supported(cpu_data_file, data)
        if data_exists:
            cache_dump_parse(input_filename, options.output_filename, options.target_cpu, cpu_data_file, seek_offset,
                             data.upper(), options.num_sets, options.associativity, options.soc)
        merge_tag_data_dumps()
    else:
        cache_dump_parse(input_filename, options.output_filename, options.target_cpu, cpu_data_file, seek_offset,
                         options.cache_type.upper(), options.num_sets, options.associativity, options.soc)


if __name__ == "__main__":
    main()
