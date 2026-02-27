# Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import os
import re

config_sim_t32 = """; Printer settings
PRINTER=WINDOWS

; Environment variables
OS=
ID=T32
TMP=C:\\Temp
;SYS=C:\\T32

PBI=SIM

SCREEN=
VFULL
FONT=MEDIUM
"""

launch_t32_gmu = """set PWD=%~dp0
start C:\\t32\\bin\\windows64\\t32marm.exe -c %PWD%/configsim.t32, %PWD%\\snapshot\\gmu_t32\\gmu_startup_script.cmm %PWD%\\snapshot\\gmu_t32
"""

gmu_startup_script_header = """title "GMU snapshot"

ENTRY &DumpDir &AxfSymbol

WinCLEAR
AREA
AREA.CLEAR

sys.cpu CORTEXM3
sys.up
PRINT "DumpDir is &DumpDir"
cd &DumpDir
PRINT "using symbol from &AxfSymbol"

"""

gmu_startup_script_footer = """

; enable single-stepping
PRINT "Enabling single-stepping..."
SYStem.Option IMASKASM ON
SYStem.Option IMASKHLL ON
SNOOP.PC ON

; reset the processor registers
Register.Init

; load the saved fault info register values
&r0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r0))
Register.Set R0 (&r0)
&r1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r1))
Register.Set R1 (&r1)
&r2=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r2))
Register.Set R2 (&r2)
&r3=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r3))
Register.Set R3 (&r3)
&r12=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r12))
Register.Set R12 (&r12)
&lr=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.stackedLR))
Register.Set LR (&lr)
&pc=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.stackedPC))
Register.Set PC (&pc)
&xpsr=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.stackedxPSR))
Register.Set XPSR (&xpsr)
&sp=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.faultStack))
Register.Set SP (&sp)

; load the other core register values
&r4=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[4]))
Register.Set R4 (&r4)
&r5=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[5]))
Register.Set R5 (&r5)
&r6=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[6]))
Register.Set R6 (&r6)
&r7=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[7]))
Register.Set R7 (&r7)
&r8=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[8]))
Register.Set R8 (&r8)
&r9=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[9]))
Register.Set R9 (&r9)
&r10=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[10]))
Register.Set R10 (&r10)
&r11=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[11]))
Register.Set R11 (&r11)

; load the saved fault info register values
&r0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[0]))
Register.Set R0 (&r0)
&r1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[1]))
Register.Set R1 (&r1)
&r2=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[2]))
Register.Set R2 (&r2)
&r3=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[3]))
Register.Set R3 (&r3)
&r12=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[12]))
Register.Set R12 (&r12)
&r13=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[13]))
Register.Set R13 (&r13)
&r14=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[14]))
Register.Set R14 (&r14)
&r15=Data.Long(Var.ADDRESS(gs_cm3Snapshot.coreRegs.r[15]))
Register.Set R15 (&r15)

; load the special register values
&msp=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.MSP))
Register.Set MSP (&msp)
&psp=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.PSP))
Register.Set PSP (&psp)
&basepri=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.BASEPRI))
Register.Set BASEPRI (&basepri)
&primask=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.PRIMASK))
Register.Set PRIMASK (&primask)
&faultmask=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.FAULTMASK))
Register.Set FAULTMASK (&faultmask)
&control=Data.Long(Var.ADDRESS(gs_cm3Snapshot.specialRegs.CONTROL))
Register.Set CONTROL (&control)

; load the nvic register values
&iser0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ISER[0]))
Data.Set SD:0xE000E100 %LE %Long &iser0
&iser1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ISER[1]))
Data.Set SD:0xE000E104 %LE %Long &iser1
&icer0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ICER[0]))
Data.Set SD:0xE000E180 %LE %Long &icer0
&icer1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ICER[1]))
Data.Set SD:0xE000E184 %LE %Long &icer1
&ispr0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ISPR[0]))
Data.Set SD:0xE000E200 %LE %Long &ispr0
&ispr1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ISPR[1]))
Data.Set SD:0xE000E204 %LE %Long &ispr1
&icpr0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ICPR[0]))
Data.Set SD:0xE000E280 %LE %Long &icpr0
&icpr1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.ICPR[1]))
Data.Set SD:0xE000E284 %LE %Long &icpr1
&iabr0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IABR[0]))
Data.Set SD:0xE000E300 %LE %Long &iabr0
&iabr1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IABR[1]))
Data.Set SD:0xE000E304 %LE %Long &iabr1

; load the interrupt priority register values
&ipr0=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[0]))
Data.Set SD:0xE000E400 %LE %Long &ipr0
&ipr1=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[1]))
Data.Set SD:0xE000E404 %LE %Long &ipr1
&ipr2=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[2]))
Data.Set SD:0xE000E408 %LE %Long &ipr2
&ipr3=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[3]))
Data.Set SD:0xE000E40C %LE %Long &ipr3
&ipr4=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[4]))
Data.Set SD:0xE000E410 %LE %Long &ipr4
&ipr5=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[5]))
Data.Set SD:0xE000E414 %LE %Long &ipr5
&ipr6=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[6]))
Data.Set SD:0xE000E418 %LE %Long &ipr6
&ipr7=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[7]))
Data.Set SD:0xE000E41c %LE %Long &ipr7
&ipr8=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[8]))
Data.Set SD:0xE000E420 %LE %Long &ipr8
&ipr9=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[9]))
Data.Set SD:0xE000E424 %LE %Long &ipr9
&ipr10=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[10]))
Data.Set SD:0xE000E428 %LE %Long &ipr10
&ipr11=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[11]))
Data.Set SD:0xE000E42C %LE %Long &ipr11
&ipr12=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[12]))
Data.Set SD:0xE000E430 %LE %Long &ipr12
&ipr13=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[13]))
Data.Set SD:0xE000E434 %LE %Long &ipr13
&ipr14=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[14]))
Data.Set SD:0xE000E438 %LE %Long &ipr14
&ipr15=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.IPR[15]))
Data.Set SD:0xE000E43C %LE %Long &ipr15

; load the software trigger interrupt register value
&stir=Data.Long(Var.ADDRESS(gs_cm3Snapshot.nvicRegs.swTriggerIntrReg))
Data.Set SD:0xE000EF00 %LE %Long &stir

; load the fault status register values
&cfsr=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultStatus.cfsr))
Data.Set SD:0xE000ED28 %LE %Long &cfsr
&hfsr=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultStatus.hfsr))
Data.Set SD:0xE000ED2C %LE %Long &hfsr
&mmfar=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultStatus.mmfar))
Data.Set SD:0xE000ED34 %LE %Long &mmfar
&bfar=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultStatus.bfar))
Data.Set SD:0xE000ED38 %LE %Long &bfar

&exec_return=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.excReturn))

PRINT &exec_return
IF (&exec_return&0x4)==0
(
    PRINT "MSP in use"
)
ELSE
(
    PRINT "PSP in use"
)

&SP=Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.faultStack))
&SP=&SP+0x20

Register.Set R0 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r0))
Register.Set R1 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r1))
Register.Set R2 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r2))
Register.Set R3 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r3))
Register.Set R12 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.r12))


Register.Set R13 &SP

Register.Set R14 Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.stackedLR))
Register.Set PC Data.Long(Var.ADDRESS(gs_cm3Snapshot.faultInfo.stackedPC))

v.v gs_cm3Snapshot
b::Register
v.f

PRINT "Success!"
"""

gmu_startup_script_symbol = 'data.load.elf "code_compile_mem.axf" /nocode'
gmu_t32_path = "gpu_parser/snapshot/gmu_t32/"


def prepare_bin_load_command(file):
    address = re.split(r"-|\.", file)[3]
    return ('data.load.binary "' + str(file) + '" ' + address + '\n')


def generate_cmm_script(dump):
    gmu_startup_script = gmu_startup_script_header

    for root, dirs, files in os.walk(gmu_t32_path):
        bin_load_commands = [prepare_bin_load_command(file) for file in files
                             if file.endswith('.bin')]
        gmu_startup_script += "".join(bin_load_commands)

    gmu_startup_script += "\n"
    gmu_startup_script += gmu_startup_script_symbol
    gmu_startup_script += gmu_startup_script_footer

    file = dump.open_file(gmu_t32_path + "gmu_startup_script.cmm", "w")
    file.write(gmu_startup_script)
    file.close()


def generate_gmu_t32_files(dump):
    generate_cmm_script(dump)

    cfg_file = dump.open_file("gpu_parser/configsim.t32", "w")
    cfg_file.write(config_sim_t32)
    cfg_file.close()

    launcht32_bat = dump.open_file("gpu_parser/launch_t32_gmu.bat", "w")
    launcht32_bat.write(launch_t32_gmu)
    launcht32_bat.close()
