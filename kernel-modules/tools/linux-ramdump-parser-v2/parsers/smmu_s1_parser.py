# Copyright (c) 2021, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

from parser_util import register_parser, RamParser
import string, os, re, print_out, dmesglib

@register_parser('--smmu-s1-fault', 'Parse SMMU Stage 1 context fault')
class SmmuParser(RamParser):

    # Flag if the system did not have time to print hard/soft iova-to-phys on dmesg
    __no_both_iova = True

    # Global parsing metadata
    __soft_iova_2_pa     = ""
    __hard_iova_2_pa     = ""
    __ioval_mismatch     = False
    __permission_fault   = False
    __atos_message       = False
    __global_text        = ""

    # Extract data and call the parser
    def parse(self):

        data = self.read_smmu_fault_data()
        parsed_text = self.__parse_log(data)

        # Write output to a file

        file_name = "smmu_s1_fault.txt"
        file = open(file_name,"w")
        file.write(parsed_text)
        file.close()

    # Parse each line, add debugging advice
    def __parse_log(self, text):

        parsed_text        = "\n---\n"
        self.__global_text = parsed_text

        # Call the SMMU parser
        parsed_text = self.parse_lines(parsed_text, text)

        # In case we replaced variables from previous lines
        if parsed_text != self.__global_text:
            parsed_text = self.__global_text

        # Make it clear that the parser ran even if we did not find any SMMU S1 fault
        if parsed_text.find("arm-smmu") == -1:
            return "No SMMU S1 fault detected\n"

        parsed_text = parsed_text + "---"

        # Include debugging advice
        parsed_text = parsed_text + "---\nDEBUGGING ADVICE:\n"
        parsed_text = parsed_text + self.offer_advice()

        return parsed_text

    def extract_smmu_log_lines(self, lines):

        aux = []
        for line in lines:
            if line.find("arm-smmu ") != -1:
                aux.append(line)
        return aux

    def read_smmu_fault_data(self):

        file_name    = "log_scratch.txt"
        scratch_file = open(file_name,"w+")

        dmesglib.DmesgLib(self.ramdump, scratch_file).extract_dmesg()
        scratch_file.seek(0,0)
        dmesg_lines = scratch_file.readlines()
        scratch_file.close()
        os.remove(file_name)

        smmu_log_lines = self.extract_smmu_log_lines(dmesg_lines)

        return smmu_log_lines

    # Parse Fault Address Register (FAR)
    # Bits[63:N] Reserved
    # FADDR, bits[N-1:0]
    def parse_FAR(self, line):

        far_line = "Faulting I/O Virtual Address (SMMU_CBn_FAR):"
        far_line = far_line + line[line.index('=') + 1:]
        return far_line

    # Invert string
    def reverse_binary(self, string):

        return string[::-1]

    # Translate the hexadecimal string to binary
    def hex_to_binary(self, hexadecimal):

        fill = 64

        if (len(hexadecimal[hexadecimal.index('x') + 1:]) < 16):
            fill = 32

        zeroes = "0" * fill

        # Sanitize input
        original    = hexadecimal
        hexadecimal = hexadecimal[hexadecimal.index('x') + 1:]

        if hexadecimal.find(' ') != -1:
            hexadecimal = hexadecimal[:hexadecimal.index(' ')]

        if self.all_zero(hexadecimal) is True:
            binary = zeroes
        else:
            try:
                binary = bin(int(hexadecimal, 16))[2:].zfill(fill)

                # If we want the rest of the code to access the array using the same indexes
                # provided in the documentation, we need to reverse this. For example, with
                # 0x4 we get 100, and if the documentation refers to bit 0 it actually means
                # (100)[2]
                binary = self.reverse_binary(binary)
            except Exception as e:
                print("Error:" + original + "(" + hexadecimal + ") couldn't convert, assuming 0x0")
                binary = zeroes

        return binary

    # Check value of certain position in the binary
    def evaluate(self, binary, position):

        string = "NO"

        # Little Endian
        if binary[position] == '1':
            string = "YES"

        return string

    # Make output pretty
    def pretty_align(self, line, length, size):

        aux = line

        for i in range(length,size):
            aux = aux + " "

        return aux

    # Format line with a question and the evaluated answer
    def question(self, question, bin, position, line=""):

        line = "\n | " + question

        # Make output pretty
        line  = self.pretty_align(line, len(question), 60)
        value = self.evaluate(bin, position)
        line  = line + value

        return line

    # Last two arguments are optional, pass only when needed
    def multi_question(self, question, bin, start, end, format):

        line    = "\n | " + question
        val     = bin[start:end + 1]

        # Make output pretty
        line = self.pretty_align(line,len(question), 60)
        val  = val[0 : (end - start + 1)]
        val  = self.reverse_binary(val)

        # Check all the options in the dictionary
        i = 0
        for key in format:
            if val == key:
                line = line + format[key]
                return line
            i = i + 1

        return "multi_question() couldn't find format for " + val

    # Parse Fault Status Register (FSR), which contains information on the address fault type
    def parse_FSR(self, line):

        fsr_line = "Fault Status Register (SMMU_CBn_FSR):"
        value    = line[line.index('=') + 1:]
        bin      = self.hex_to_binary(value)
        fsr_line = fsr_line + value
        pos      = 0

        faults = [
            "Reserved",
            "Translation Fault",
            "Access flag fault",
            "Permission fault",
            "External fault",
            "TLB match conflict fault",
            "TLB Lock Fault",
            "Incorrect address size",
            "Unsupported upstream transaction",
            "Error finding fault on parse_FSR()" # Keep this option last
        ]

        format_bits_dict = {
            '00' :'AArch32 Short-descriptor scheme',
            '01' :'AArch32 Long-descriptor translation scheme',
            '10' :'AArch64 translation scheme',
            '11' : 'AArch64 translation scheme'
        }

        fsr_line = fsr_line + self.question("Multiple faults when FSR was nonzero?", bin, 31)
        fsr_line = fsr_line + self.question("Was the context stalled?", bin, 30)
        fsr_line = fsr_line + self.multi_question("Translation scheme:", bin, 9, 10, format_bits_dict)

        #The following faults are mutually exclusive so let us reduce the output

        fsr_line  = fsr_line + "\n | Fault type: "
        fault_idx = bin[0:8].find('1')

        if fault_idx == -1:
            # Index to the error message of the array
            print("Error finding fault on parse_FSR()")
            fault_idx = len(faults) -1

        aux = faults[fault_idx]

        if fault_idx == 3:
            self.__permission_fault = True

        fsr_line = self.pretty_align(fsr_line, len("\n | Fault type: ") + 1, 65)
        fsr_line = fsr_line + aux + "\n"

        return fsr_line

    # Parse FSYNR0, which holds fault syndrome information about the memory access that
    # caused a synchronous abort exception.
    def parse_FSYNR0(self, line):

        FSYNR0_line = "Fault Syndrome Register 0 (SMMU_CBn_FSYNR0):"
        value       = line[line.index('=') + 1:]
        bin         = self.hex_to_binary(value)
        aux         = "\n | Type of fault:"
        size        = 64

        FSYNR0_line     = FSYNR0_line + value

        format_bits_dict = {
            '00' : 'Level 0, SMMUv2 only',
            '01' : 'Level 1',
            '10' : 'Level 2',
            '11' : 'Level 3'
        }

        FSYNR0_line = FSYNR0_line + self.question("Was a fault recorded synchronously?", bin, 11)
        FSYNR0_line = FSYNR0_line + self.question("Was there a Page Table Walk Fault?", bin, 10)
        FSYNR0_line = FSYNR0_line + aux

        # Make output pretty
        FSYNR0_line = self.pretty_align(FSYNR0_line, len(aux), size)

        if self.evaluate(bin, 4) == "YES":
            FSYNR0_line = FSYNR0_line + "Write Fault"
        else:
            FSYNR0_line = FSYNR0_line + "Read Fault"

        FSYNR0_line = FSYNR0_line + self.multi_question("Translation Table Level:", bin, 0, 1, format_bits_dict)

        return FSYNR0_line + "\n"

    # Parse context bank
    def parse_context_bank(self, line):

        context_bank_line = "Context bank:"
        context_bank_line = context_bank_line + line[line.index('=') + 1:]

        return context_bank_line

    # Parse SCTLR, which provides top-level control of the translation system for the
    # related translation context bank.
    def parse_SCTLR(self, line):

        SCTLR_line  = "System Control Register (SMMU_CBn_SCTLR):"
        value       = line[line.index('=') + 1:]
        bin         = self.hex_to_binary(value)
        value       = value.replace(" ar","")
        SCTLR_line  = SCTLR_line + value

        # The Write/Read allocate and Shared configurations are overwritten by the S1 page
        # tables so their values provide no useful information.

        SCTLR_line = SCTLR_line + self.question("Force Broadcast of TLB maintenance?", bin, 21)
        SCTLR_line = SCTLR_line + self.question("Protected Translation Walk enabled?", bin, 13)
        SCTLR_line = SCTLR_line + self.question("Stall instead of terminate transaction?", bin, 7)
        SCTLR_line = SCTLR_line + self.question("Is Context Fault Interrupt enabled?", bin, 6)
        SCTLR_line = SCTLR_line + self.question("Is Context Fault Report enabled?", bin, 5)
        SCTLR_line = SCTLR_line + self.question("Big Endian translation table entries?", bin, 4)
        SCTLR_line = SCTLR_line + self.question("Is TEX Remap enabled?", bin, 1)
        SCTLR_line = SCTLR_line + "\n"

        return SCTLR_line

    # Parse CBAR
    def parse_CBAR(self, line):

        CBAR_line = "Context Bank Attribute Registers (SMMU_CBAR):"
        CBAR_line = CBAR_line + line[line.index('=') + 1:]

        return CBAR_line

    # Parse MAIR0
    def parse_MAIR0(self, line):

        MAIR0_line = "Memory Attribute Indirection Registers 0 (SMMU_CBn_MAIR0):"
        MAIR0_line = MAIR0_line + line[line.index('=') + 1:]

        return MAIR0_line

    # Parse SID
    def parse_SID(self, line):

        SID_line = "Context Bank Fault Restricted Syndrome Register (SMMU_CBFRSYNRA):"
        SID_val  = line[line.index('=') + 1:]
        SID_line = SID_line + SID_val
        SID_val  = SID_val.replace(" ","")
        SID_val  = " ".join(SID_val.split()).strip()
        SID_val  = SID_val[:5]

        self.__global_text = self.__global_text.replace("SID=?", "SID=" + SID_val)

        return SID_line

    # Parse Client info
    def parse_client_info(self, line):

        Client_info_line = ""

        left             =  line[line.index(':') + 1:]
        BID_val          =  left[left.index("=") + 1: left.index(",")]
        PID_val          =  left[left.index(",") + 1: left.rfind("=") - 3]
        MID_val          =  left[left.rfind("=") + 1: len(left)]
        PID_val          = PID_val.replace(" ","")

        self.__global_text = self.__global_text.replace("BID=?","BID=" + BID_val)
        self.__global_text = self.__global_text.replace("PID=?",PID_val)
        self.__global_text = self.__global_text.replace("MID=?","MID=" + MID_val)
        self.__global_text = self.__global_text.replace(", ,", ',')
        self.__global_text = self.__global_text.replace(",,", ',')

        return Client_info_line

    # Parse soft iova-to-phys
    def  parse_soft_iova_to_phy(self, line):

        val = line[line.index('=') + 1:]
        soft_iova_to_phy_line = "Software table walk result: "
        soft_iova_to_phy_line = soft_iova_to_phy_line + val
        self.__soft_iova_2_pa  = val

        return soft_iova_to_phy_line

    # Parse hard iova-to-phys
    def parse_hard_iova_to_phy(self, line):

        hard_iova_to_phy_line = "SMMU table walk result: "
        val = line[line.index('=') + 1:]

        self.__hard_iova_2_pa = val
        self.__no_both_iova = False

        # Do the software and HW translations of the IOVA match?
        if self.__hard_iova_2_pa != self.__soft_iova_2_pa :
            self.__ioval_mismatch = True

        hard_iova_to_phy_line = hard_iova_to_phy_line + val

        return hard_iova_to_phy_line

    def parse_line(self, line):

        new_line = ""

        # Is this a completely unrelated debug message?
        if line.find("arm-smmu") == -1:
            self.__global_text = self.__global_text + new_line
            return new_line

        # Remove unnecessary left-side stuff such as the dmesg print time
        line = line[line.index(':') + 2:]

        # A switch statement would require a single variable against a range of constant values,
        # so the correct logic here -even if utterly ugly- is an if/else.

        if line.find("Unhandled") != -1:
            new_line  = line + " \nClient information: SID=?,BID=?,PID=?,MID=?"
        elif line.find("FAR") != -1:
                new_line  = self.parse_FAR(line)
        elif line.find("FSR") != -1:
            new_line  = self.parse_FSR(line)
        elif line.find("FSYNR0") != -1:
            new_line  = self.parse_FSYNR0(line)
        elif line.find("context bank") != -1:
            new_line  = self.parse_context_bank(line)
        elif line.find("SCTLR") != -1:
            new_line  = self.parse_SCTLR(line)
        elif line.find("CBAR") != -1:
            new_line  = self.parse_CBAR(line)
        elif line.find("MAIR0") != -1:
            new_line  = self.parse_MAIR0(line)
        elif line.find("SID") != -1:
            new_line  = self.parse_SID(line)
            if new_line == "":
                return new_line
        elif line.find("Client info") != -1:
            new_line  = self.parse_client_info(line)
            return ""
        elif line.find("soft iova-to-phys") != -1:
            new_line  = self.parse_soft_iova_to_phy(line)
        elif line.find("hard iova-to-phys") != -1:
            new_line  = self.parse_hard_iova_to_phy(line)
        elif line.find("ATOS") != -1:
            self.__atos_message = True
            return ""
        else:
            other_regs = ["PAR","TTBR0","TTBR1","FSYNR1"]
            for reg in other_regs:
                if line.find(reg) != -1:
                    return ""
            new_line = "Unknown message disregarded, line says: " + line
            print(new_line)

        self.__global_text = self.__global_text + new_line + "\n"

        return new_line

    def parse_lines(self, parsed_text, lines):

        extra_text = parsed_text

        for line in lines:
            extra_text = extra_text + self.parse_line(line)

        return extra_text

    # Check if we have all zeroes in the value string
    def all_zero(self, str):

        ret = True

        if isinstance(str, int) is True:
            if str == 0:
                ret = False
        else:
            for i in range(0, len(str)):
                if (str[i] != '0' and not (i == 1 and str[i] == 'x')):
                    ret = False
                    break

        return ret

    # Offer debugging advice based on the global variables obtained parsing
    def offer_advice(self):

        advice = ""
        io_mismatch = self.__ioval_mismatch
        soft        = self.__soft_iova_2_pa
        permission  = self.__permission_fault
        atos        = self.__atos_message

        advice1 = ("The hardware and software translations of the I/O virtual address do not match,"
            " this is an indication of a race condition. It is likely that you are trying"
            " to access a memory resource while unmapping it elsewhere.")

        advice2 = ("The hardware and software translations of the I/O virtual address match but "
            "are zero, this is an indication of an access to an unmapped value.")

        advice3 = ("The hardware and software translations of the I/O virtual address match"
            " and are meaningful. A possible explanation is that between the"
            " time the context fault was triggered -when hardware tried to "
            "access a deallocated resource- and the handler started, you mapped"
            " other buffer that happened to have the same IO virtual address.")

        advice4 = ("There has been a permission fault (and we have meaningful and equal"
            " I/O translations). To determine if this is a legitimate permission"
            " fault, try to replicate the error mapping the buffers as privileged. Perhaps"
            " you are mapping with the wrong permission flags?")

        advice5 = ("If you repeat the experiment with privileged buffer after flushing the"
            "TLB (iommu_flush_iotlb_all) and it succeeds, it is likely that"
            " you are facing a Stale TLB entry. If so, contact the kernel team for support.")

        advice6 = ("No accurate debugging advice can be given without knowing if the "
            "hardware and software translations match.")

        if self.__no_both_iova is True:
            advice = advice6

        elif io_mismatch:
            advice = advice1

        else:
            if soft == "0x0" or soft == "0x0000000000000000" or self.all_zero(soft) is True:
                advice = advice2
            else:
                if permission is False:
                    advice = advice3
                else:
                    if atos is False:
                        advice = advice4
                    else:
                        advice = advice5

        advice = advice + "\n\n"

        return advice
