# Makefile
#
# RISC-V 32-bit Processor
#
# ECE 18-447
# Carnegie Mellon University
#
# This is the 18-447 Makefile.
#
# The Makefile implements the build system for the 18-447 labs. This file
# handles setting up the targets for assembling RISC-V test programs, compiling
# the processor into a simulator, running and testing the simulator, and
# synthesizing the processor.
#
# Authors:
#   - 2011: Joshua Wise
#   - 2013: Yoongu Kim
#   - 2016: Zhipeng Zhao
#	- 2016 - 2017: Brandon Perez

################################################################################
#                           DO NOT MODIFY THIS FILE!                           #
#           You should only add or change files in the src directory!          #
################################################################################

################################################################################
# User Controlled Parameters
################################################################################

# The output directories for simulation and synthesis files and results
OUTPUT_BASE_DIR = output
SIM_OUTPUT = $(OUTPUT_BASE_DIR)/simulation
SYNTH_OUTPUT = $(OUTPUT_BASE_DIR)/synthesis

# Default the output directory to the synthesis for synthesis targets
SYNTH_TARGETS = synth view-timing view-power view-area synth-clean
ifneq ($(filter $(SYNTH_TARGETS),$(MAKECMDGOALS)),)
    OUTPUT = $(SYNTH_OUTPUT)
else
    OUTPUT = $(SIM_OUTPUT)
endif

# The DC script used for synthesizing the processor
SYNTH_SCRIPT = dc/dc_synth.tcl

# The lab number for the current lab. The build system is targeted at this.
DEFAULT_LAB_18447 = 1b
LAB_18447 = $(DEFAULT_LAB_18447)
VALID_LABS = 1b 2 3 4a 4b
VALID_LABS_STRING = {1b, 2, 3, 4a, 4b}

################################################################################
# General Targets and Variables
################################################################################

# Set the shell to bash for when the Makefile runs shell commands. Enable the
# pipefail option, so if any command in a pipe fails, the whole thing fails.
# This is necessary for when a failing command is piped to `tee`.
SHELL = /bin/bash -o pipefail

# Set the number of threads to use for parallel compilation (2 * cores)
CORES = $(shell getconf _NPROCESSORS_ONLN)
THREADS = $(shell echo $$((2 * $(CORES))))

# Terminal color and modifier attributes. Make sure to handle when no terminal
# is connected.
# Return to the normal terminal colors
n := $(shell tty -s && tput sgr0)
# Red color
r := $(shell tty -s && tput setaf 1)
# Green color
g := $(shell tty -s && tput setaf 2)
# Bold text
b := $(shell tty -s && tput bold)
# Underlined text
u := $(shell tty -s && tput smul)

# These targets don't correspond to actual generated files
.PHONY: default all clean veryclean check-test-defined check-lab-number-valid

# By default, display the help message for the Makefile
default all: help

# Clean up most of the intermediate files generated by compilation
clean: assemble-clean build-clean sim-clean synth-clean
	@rm -rf $(OUTPUT)
	@rm -rf $(OUTPUT_BASE_DIR)

# Clean up all the intermediate files generated by compilation
veryclean: clean assemble-veryclean build-veryclean sim-veryclean \
		synth-veryclean

# Create the specified output directory, if it doesn't exist
$(OUTPUT):
	@mkdir -p $@

# Check that the TEST variable was specified by the user
check-test-defined:
ifeq ($(strip $(TEST)),)
	@printf "$rError: Variable $bTEST$n$r was not specified.\n$n"
	@exit 1
endif

# Check that the LAB_18447 lab number specified is valid
check-lab-number-valid:
ifeq ($(filter $(LAB_18447),$(VALID_LABS)),)
	@printf "$rError: Invalid lab '%s' specified. Lab number must " $(LAB_18447)
	@printf "be one of $(VALID_LABS_STRING).$n\n"
	@exit 1
endif

################################################################################
# Assemble Test Programs
################################################################################

# These targets don't correspond to actual files
.PHONY: assemble assemble-clean assemble-veryclean assemble-check-compiler \
		assemble-check-test assemble-check-extension assemble-check-objcopy \
		assemble-check-objdump

# The name of the entry point for assembly tests, which matches the typical main
RISCV_ENTRY_POINT = main

# The runtime environment directory, which has the startup file for C programs
# and the linker script used to layout the test programs.
447_RUNTIME_DIR = 447runtime
RISCV_STARTUP_FILE = $(447_RUNTIME_DIR)/crt0.S
RISCV_LINKER_SCRIPT = $(447_RUNTIME_DIR)/test_program.ld

# The compiler for test programs, and its flags. Even though integer
# multiplication and floating point instructions aren't supported by the
# processor, we can use libgcc's software implementation of these instructions.
RISCV_CC = riscv64-unknown-elf-gcc
RISCV_CFLAGS = -static -nostdlib -nostartfiles -march=rv32im -mabi=ilp32 -Wall \
		-Wextra -std=c11 -pedantic -g -Werror=implicit-function-declaration
RISCV_AS_LDFLAGS = -Wl,-e$(RISCV_ENTRY_POINT)
RISCV_LDFLAGS = -Wl,-T$(RISCV_LINKER_SCRIPT) -lgcc

# If a C is being compiled, do so at the highest optimization level.
ifeq ($(dir $(TEST)),benchmarks/)
    RISCV_CFLAGS += -O -fno-inline
else ifeq ($(dir $(TEST)),benchmarksO3/)
    RISCV_CFLAGS += -O3 -fno-inline
else ifeq ($(dir $(TEST)),private/)
    RISCV_CFLAGS += -O -fno-inline
else ifeq ($(dir $(TEST)),privateO3/)
    RISCV_CFLAGS += -O3 -fno-inline
endif

# The objcopy utility for ELF files, along with its flags.
RISCV_OBJCOPY = riscv64-unknown-elf-objcopy
RISCV_OBJCOPY_FLAGS = -O binary

# The objdump utility for ELF files, along with its flags
RISCV_OBJDUMP = riscv64-unknown-elf-objdump
RISCV_OBJDUMP_FLAGS = -d -M numeric,no-aliases $(addprefix -j ,.text .ktext \
		.data .bss .kdata .kbss)

# The file extensions for all files generated, including intermediate ones
ELF_EXTENSION = elf
BINARY_EXTENSION = bin
DISAS_EXTENSION = disassembly.s

# The binary files generated when the program is assembled. There's one for each
# assembled segment: user and kernel text and data sections.
TEST_NAME = $(basename $(TEST))
BIN_SECTIONS = $(addsuffix .$(BINARY_EXTENSION),text data ktext kdata)
TEST_BIN = $(addprefix $(TEST_NAME).,$(BIN_SECTIONS))

# The name of the binary files in the output directory, used by the testbench
OUTPUT_NAME = $(OUTPUT)/mem
TEST_OUTPUT_BIN = $(addprefix $(OUTPUT_NAME).,$(BIN_SECTIONS))

# The ELF and disassembly files generated when the test is assembled
TEST_EXECUTABLE = $(addsuffix .$(ELF_EXTENSION), $(TEST_NAME))
TEST_DISASSEMBLY = $(addsuffix .$(DISAS_EXTENSION), $(TEST_NAME))

# Log file for capturing the output of assembling the test
ASSEMBLE_LOG = $(OUTPUT)/assemble.log

# Always re-run the recipe for copying binary files to the output directory,
# because the specified test can change based on the user's input.
.PHONY: $(TEST_OUTPUT_BIN)

# Prevent make from automatically deleting the intermediate ELF file generated.
# Instead, this is done manually, which prevents the commands from being echoed.
.PRECIOUS: %.$(ELF_EXTENSION)

# User-facing target to assemble the specified test
assemble: $(TEST) $(TEST_OUTPUT_BIN) $(TEST_DISASSEMBLY) | check-test-defined \
		assemble-check-extension

# Copy an assembled ASCII binary file for a section to the output directory
$(TEST_OUTPUT_BIN): $(OUTPUT_NAME).%.$(BINARY_EXTENSION): \
		$(TEST_NAME).%.$(BINARY_EXTENSION) | $(OUTPUT)
	@cp $^ $@

# Extract the given section from the program ELF file, generating a binary
$(TEST_NAME).%.$(BINARY_EXTENSION): $(TEST_EXECUTABLE) | $(OUTPUT) \
		assemble-check-objcopy check-test-defined
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .$* $^ $@ |& \
			tee -a $(ASSEMBLE_LOG)

# The user data section must be handled specially when extracting it from the
# ELF file, because the .bss section is also extracted and concatenated with it.
$(TEST_NAME).data.$(BINARY_EXTENSION): $(TEST_EXECUTABLE) | \
		assemble-check-objcopy
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .data -j .bss \
			--set-section-flags .bss=alloc,load,contents $^ $@

# The kernel data section must also be handled specially for the same reason
$(TEST_NAME).kdata.$(BINARY_EXTENSION): $(TEST_EXECUTABLE) | \
		assemble-check-objcopy
	@$(RISCV_OBJCOPY) $(RISCV_OBJCOPY_FLAGS) -j .kdata -j .kbss \
			--set-section-flags .kbss=alloc,load,contents $^ $@

# Generate a disassembly of the compiled program for debugging proposes
%.$(DISAS_EXTENSION): %.$(ELF_EXTENSION) | $(OUTPUT) assemble-check-objdump
	@$(RISCV_OBJDUMP) $(RISCV_OBJDUMP_FLAGS) $^ > $@ |& tee -a $(ASSEMBLE_LOG)
	@printf "\nAssembly of the test has completed. The assembly log can be "
	@printf "found at $u$(ASSEMBLE_LOG)$n.\n"
	@printf "A disassembly of the test can be found at "
	@printf "$u$*.$(DISAS_EXTENSION)$n.\n"

# Compile the assembly test program with a *.S extension to create an ELF file
%.$(ELF_EXTENSION): %.S $(RISCV_LINKER_SCRIPT) | $(OUTPUT) \
		assemble-check-compiler assemble-check-test
	@printf "Assembling test $u$<$n into binary files...\n"
	@$(RISCV_CC) $(RISCV_CFLAGS) $^ $(RISCV_LDFLAGS) $(RISCV_AS_LDFLAGS) -o $@ \
			|& tee $(ASSEMBLE_LOG)

# Compile the C test program with the startup file to create an ELF file
%.$(ELF_EXTENSION): $(RISCV_STARTUP_FILE) %.c $(RISCV_LINKER_SCRIPT) | \
		$(OUTPUT) assemble-check-compiler assemble-check-test
	@printf "Assembling test $u$(word 2,$^)$n into binary files...\n"
	@$(RISCV_CC) $(RISCV_CFLAGS) $(wordlist 1,2,$^) $(RISCV_LDFLAGS) -o $@ |& \
			tee $(ASSEMBLE_LOG)

# Checks that the given test exists. This is used when the test doesn't have
# a known extension, and suppresses the 'no rule to make...' error message
$(TEST): | assemble-check-extension assemble-check-test

# Clean up the binary files in the output directory
assemble-clean:
	@printf "Cleaning up assembled binary files in $u$(OUTPUT)$n...\n"
	@rm -f $(TEST_OUTPUT_BIN) $(ASSEMBLE_LOG)

# Clean up all the binary files in the output and project directories
assemble-veryclean: assemble-clean
	@printf "Cleaning up assembled binary files in the project directory...\n"
	@rm -f $$(find -L -name '*.$(BINARY_EXTENSION)' \
			-o -name '*.$(ELF_EXTENSION)' -o -name '*.$(DISAS_EXTENSION)')

# Check that the RISC-V compiler exists
assemble-check-compiler:
ifeq ($(shell which $(RISCV_CC) 2> /dev/null),)
	@printf "$rError: $u$(RISCV_CC)$n$r: RISC-V compiler was not found in "
	@printf "your PATH.$n\n"
	@exit 1
endif

# Check that the specified test file exists
assemble-check-test:
ifeq ($(wildcard $(TEST)),)
	@printf "$rError: $u$(TEST)$n$r: RISC-V test file does not exist.$n\n"
	@exit 1
endif

# Check that the specified test file exists
assemble-check-extension:
ifeq ($(filter %.c %.S,$(TEST)),)
	@printf "$rError: $u$(TEST)$n$r: RISC-V test file does not have a .c or .S "
	@printf "extension.$n\n"
	@exit 1
endif

# Check that the RISC-V objcopy binary utility exists
assemble-check-objcopy:
ifeq ($(shell which $(RISCV_OBJCOPY) 2> /dev/null),)
	@printf "$rError: $u$(RISCV_OBJCOPY)$n$r: RISC-V objcopy binary utility "
	@printf "was not found in your PATH.$n\n"
	@exit 1
endif

# Check that the RISC-V objdump binary utility exists
assemble-check-objdump:
ifeq ($(shell which $(RISCV_OBJDUMP) 2> /dev/null),)
	@printf "$rError: $u$(RISCV_OBJDUMP)$n$r: RISC-V objdump binary utility "
	@printf "was not found in your PATH.$n\n"
	@exit 1
endif

################################################################################
# Compile the Simulator
################################################################################

# These targets don't correspond to actual files.
.PHONY: build build-clean build-veryclean build-check-compiler

# The starter code files provided by the 18-447 staff, used for simulation.
447_SRC_DIR := $(shell readlink -m 447src)
447_INCLUDE_DIR := $(shell readlink -m 447include)
447_SRC = $(shell find -L $(447_SRC_DIR) $(447_INCLUDE_DIR) -type f \
		-name '*.v' -o -name '*.sv' -o -name '*.vh' | sort)

# The code files and directories created by the student. Keep the paths as
# relative for the help message. Sort the files so that the ordering is
# consistent. This is important for compilation, as the order of compilation
# affects whether or not errors are thrown by missing `include directives.
help: SRC_DIR = src
SRC_DIR := $(shell readlink -m src)
SRC = $(shell find -L $(SRC_DIR) -type f -name '*.v' -o -name '*.sv' \
		-o -name '*.vh' | sort)
SRC_SUBDIRS = $(shell find -L $(SRC_DIR) -type d | sort)

# Compiler for simulation, along with its flags
SIM_CC = vcs
SIM_CFLAGS = -sverilog -debug_all +memcbk -q -j $(THREADS) +warn=all \
		+lint=PCWM,IWU,TFIPC,ONGS,VNGS,IRIMW,UI,CAWM-L +error+20 \
		-xzcheck nofalseneg +define+SIMULATION_18447 \
		'+define+LAB_18447="$(LAB_18447)"'
SIM_INC_FLAGS = $(addprefix +incdir+,$(447_SRC_DIR) $(447_INCLUDE_DIR) \
		$(SRC_SUBDIRS))

# The names of log for compilation and the executable generated for simulation.
SIM_COMPILE_LOG = compilation.log
SIM_EXECUTABLE = riscv_core

# The other files generated by VCS compilation
VCS_BUILD_FILES = csrc $(SIM_EXECUTABLE).daidir
BUILD_EXTRA_FILES = $(addprefix $(OUTPUT)/,$(VCS_BUILD_FILES) \
		$(SIM_COMPILE_LOG))

# The user-facing target to compile the processor simulator into an executable.
build: $(OUTPUT)/$(SIM_EXECUTABLE)

# Compile the processor into a simulator executable. This target only depends on
# the output directory existing, so don't force it to re-run because of it.
$(OUTPUT)/$(SIM_EXECUTABLE): $(447_SRC) $(SRC) | $(OUTPUT) \
		build-check-compiler check-lab-number-valid
	@printf "Compiling design into a simulator in $u$(OUTPUT)$n...\n"
	@cd $(OUTPUT) && $(SIM_CC) $(SIM_CFLAGS) $(SIM_INC_FLAGS) \
			$(filter %.v %.sv,$^) -o $(SIM_EXECUTABLE) |& tee $(SIM_COMPILE_LOG)
	@printf "\nCompilation of the simulator has completed. The compilation log "
	@printf "can be found at $u$(OUTPUT)/$(SIM_COMPILE_LOG)$n\n"
	@printf "The simulator executable can be found at $u$@$n.\n"

# Clean up an intermediate files generated by compiling the simulator.
build-clean:
	@printf "Cleaning up the simulator executable...\n"
	@rm -rf $(OUTPUT)/$(SIM_EXECUTABLE) $(BUILD_EXTRA_FILES)

# Very clean is the same as clean for the build targets.
build-veryclean: build-clean

# Check that the Verilog simulator compiler exists
build-check-compiler:
ifeq ($(shell which $(SIM_CC) 2> /dev/null),)
	@printf "$rError: $u$(SIM_CC)$n$r: Verilog simulator compiler was not "
	@printf "found in your $bPATH$n$r.\n$n"
	@exit 1
endif

################################################################################
# Simulate Verilog
################################################################################

# These targets don't correspond to actual files
.PHONY: sim sim-gui

# The register dump and log files generator by running the processor simulator.
SIM_REGDUMP = $(OUTPUT)/simulation.reg
SIM_LOG = simulation.log

# The other files generated by VCS simulation and/or running the DVE GUI
VCS_SIM_FILES = DVEfiles $(SIM_EXECUTABLE).vdb ucli.key inter.vpd
SIM_EXTRA_FILES = $(addprefix $(OUTPUT)/,$(VCS_SIM_FILES) $(SIM_LOG))

# Always run the simulator to generate the register dump, because the specified
# test can change based on the user's input.
.PHONY: $(SIM_REGDUMP)

# User-facing target to run the simulator with the given test
sim: $(SIM_REGDUMP) | assemble check-test-defined

# Open the waveform viewer for the processor simulation with the given test.
# Wait until the simulator GUI starts up before finishing.
sim-gui: $(TEST) $(TEST_OUTPUT_BIN) $(OUTPUT)/$(SIM_EXECUTABLE) | assemble \
		check-test-defined
	@printf "Starting up the simulator gui in $u$(OUTPUT)$n...\n"
	@cd $(OUTPUT) && ./$(SIM_EXECUTABLE) -gui &
	@sleep 2

# Run the processor simulation with the given test, generating a register dump
$(SIM_REGDUMP): $(TEST) $(TEST_OUTPUT_BIN) $(OUTPUT)/$(SIM_EXECUTABLE) | \
		$(OUTPUT) assemble
	@printf "Simulating test $u$(TEST)$n in $u$(OUTPUT)$n...\n"
	@cd $(OUTPUT) && ./$(SIM_EXECUTABLE) |& tee $(SIM_LOG)
	@printf "\nSimulation has completed. The simulation log can be found at "
	@printf "$u$(OUTPUT)/$(SIM_LOG)$n\n"
	@printf "The simulator register dump can be found at $u$(SIM_REGDUMP)$n\n"

# Suppresses 'no rule to make...' error when the REF_REGDUMP doesn't exist
$(REF_REGDUMP):

# Run the reference simulator with the specified test
REFSIM_EXECUTABLE = /afs/ece/class/ece447/bin/riscv-ref-sim

refsim: $(REFSIM_EXECUTABLE) $(TEST) | assemble check-test-defined
	@printf "Running reference sim on test $u$(TEST)$n...\n"
	@$(REFSIM_EXECUTABLE) $(TEST)

# Run the reference simulator with the specified test and generate the reference regdump
REFSIM_REGDUMP = refdump.reg

refdump: $(TEST_BIN) $(REFSIM_EXECUTABLE) $(TEST) | assemble
	@printf "Generating refdump.reg from reference sim on test $u$(TEST)$n...\n"
	@printf "go\nrdump $(REFSIM_REGDUMP)\n" | $(REFSIM_EXECUTABLE) $(TEST)


# Clean up all the files generated by VCS compilation and the DVE GUI
sim-clean:
	@printf "Cleaning up simulation files...\n"
	@rm -rf $(SIM_REGDUMP) $(SIM_EXTRA_FILES) $(REFSIM_REGDUMP)

# Very clean is the same as clean for simulation
sim-veryclean: sim-clean

################################################################################
# Verify Verilog Simulation
################################################################################

# The script used to verify, and the options for it
VERIFY_SCRIPT = sdiff
VERIFY_OPTIONS = --ignore-all-space --ignore-blank-lines

# The reference register dump used to verify the simulator's
REF_REGDUMP = $(basename $(TEST)).reg

# Based on the lab number being targeted, select that the tests that the
# students are required to pass for checkoff. Some labs do not require control
# flow instructions, so those tests are skipped.

# Basic ALU test set
PUBLIC_TESTS = $(addprefix 447inputs/,additest.S addtest.S arithtest.S \
		 lwtest.S memtest0.S memtest1.S shifttest.S syscalltest.S)

# Dependence stress test
ifneq ($(filter $(LAB_18447),2 3 4a 4b),)
    PUBLIC_TESTS += $(addprefix 447inputs/,dependLow.S depend.S )
endif

# Control flow test
ifneq ($(filter $(LAB_18447),1b 3 4b),)
    PUBLIC_TESTS += $(addprefix 447inputs/,beqtest.S brtest0.S brtest1.S brtest2.S)
endif

# multiply tests basic
ifneq ($(filter $(LAB_18447),1b 2 3 4a 4b),)
    PUBLIC_TESTS += $(addprefix 447inputs/, multest.S)
endif

# multiply tests depend
ifneq ($(filter $(LAB_18447),2 3 4a 4b),)
    PUBLIC_TESTS += $(addprefix 447inputs/, dependMulLow.S dependMul.S)
endif

# C tests
ifneq ($(filter $(LAB_18447),3 4b),)
    PUBLIC_TESTS += $(addprefix benchmarks/,fibi.c fibm.c fibr.c)
endif

# MMM tests
ifneq ($(filter $(LAB_18447),3 4b),)    
    PUBLIC_TESTS += $(addprefix benchmarks/, mmmRV32IM.c)
    PUBLIC_TESTS += $(addprefix benchmarksO3/, mmmRV32IM.c)
endif

# lab4b specials
ifneq ($(filter $(LAB_18447),4b),)    
    PUBLIC_TESTS += $(addprefix benchmarks/, mixed.c)
endif

# 1b full isa detailed test
ifneq ($(filter $(LAB_18447),1b),)
PUBLIC_TESTS += $(addprefix 447inputs2/, \
                  add.S and.S or.S slt.S sltu.S sra.S srl.S sub.S xor.S \
                  addi.S andi.S lui.S ori.S slli.S slti.S sltiu.S srai.S srli.S xori.S auipc.S \
                  lb.S lbu.S lh.S lhu.S lw.S \
                  sb.S sh.S sw.S \
                  beq.S bge.S bgeu.S blt.S bltu.S bne.S jalr.S jal.S jr.S j.S)
endif

# The autograde tests default to the public tests, if none were specified.
ifeq ($(strip $(TESTS)),)
    TESTS = $(PUBLIC_TESTS)
endif

# These targets don't correspond to actual generated files
.PHONY: verify autograde verify-check-ref-regdump

# Verify that the processor simulator's register dump for the given test matches
# the reference register dump, in the corresponding *.reg file
verify: $(SIM_REGDUMP) $(REF_REGDUMP) | assemble verify-check-ref-regdump \
		check-test-defined
	@printf "\n"
	@if $(VERIFY_SCRIPT) $(VERIFY_OPTIONS) $^ &> /dev/null; then \
		printf "$gCorrect! The simulator register dump matches the "; \
		printf "reference.$n\n"; \
	else \
		printf "\n%-67s\t%s\n\n" "$u$(SIM_REGDUMP)$n" "$u$(REF_REGDUMP)$n"; \
		$(VERIFY_SCRIPT) $(VERIFY_OPTIONS) $^; \
		printf "$rIncorrect! The simulator register dump does not match the "; \
		printf "reference.$n\n"; \
		exit 1; \
	fi

# Run verification on the specified series of tests. If left unspecified, then
# this defaults to the public tests students are required to pass for this lab.
autograde:
	@printf "%-30s %s\n" "Test" "Result"
	@printf "%.0s-" {1..37}
	@printf "\n"
	@for test in $(TESTS); do \
		printf "$b%-26s %s$n" "$${test}" "Running..."; \
		make verify TEST=$${test} OUTPUT=$(OUTPUT) &> /dev/null; \
		if [ $$? -eq 0 ]; then \
			printf "\r$g%-30s %s$n\n" "$${test}" "Passed"; \
		else \
			printf "\r$r%-30s %s$n\n" "$${test}" "Failed"; \
		fi \
	done

# Suppresses 'no rule to make...' error when the REF_REGDUMP doesn't exist
$(REF_REGDUMP):

# Check that the reference register dump for the specified test exists
verify-check-ref-regdump:
ifeq ($(wildcard $(REF_REGDUMP)),)
	@printf "$rError: $u$(REF_REGDUMP)$n$r: Reference register dump for test "
	@printf "$u$(TEST)$n$r does not exist.\n$n"
	@exit 1
endif

################################################################################
# Synthesize Verilog
################################################################################

# Compiler for synthesis
SYNTH_CC = dc_shell-xg-t

# The DC script used for synthesis. Convert its path to an absolute one.
DC_SCRIPT := $(shell readlink -m $(SYNTH_SCRIPT))

# The files generated by running synthesis, the area, timing, and power reports,
# along with the netlist for the processor.
TIMING_REPORT = timing_riscv_core.rpt
POWER_REPORT = power_riscv_core.rpt
AREA_REPORT = area_riscv_core.rpt
REPORTS = $(TIMING_REPORT) $(POWER_REPORT) $(AREA_REPORT)
NETLIST = netlist_riscv_core.sv
SYNTH_REPORTS = $(addprefix $(OUTPUT)/,$(REPORTS) $(NETLIST))

# Log file for capturing the output of synthesis, stored in the output directory
SYNTH_LOG = synthesis.log

# The other files generated by DC synthesis
DC_FILES = work default.svf command.log
SYNTH_EXTRA_FILES = $(addprefix $(OUTPUT)/,$(DC_FILES) $(SYNTH_LOG))

# If the user specified a clock period, pass it to the DC script
ifneq ($(strip $(CLOCK_PERIOD)),)
    SET_CLOCK_PERIOD = ; set clock_period $(CLOCK_PERIOD)
endif

# These targets don't correspond to actual generated files
.PHONY: synth view-timing view-power view-area synth-clean synth-veryclean \
		synth-check-compiler synth-check-script

# User-facing target to synthesize the processor into a physical design
synth: $(SYNTH_REPORTS)

# View the timing report from synthesis. If it doesn't exist, run synthesis.
view-timing:
	@if [ ! -e $(OUTPUT)/$(TIMING_REPORT) ]; then \
		make OUTPUT=$(OUTPUT) SYNTH_SCRIPT=$(SYNTH_SCRIPT) synth; \
	fi
	@printf "$uTiming Report: $(OUTPUT)/$(TIMING_REPORT):$n\n\n"
	@cat $(OUTPUT)/$(TIMING_REPORT)

# View the power report from synthesis. If it doesn't exist, run synthesis.
view-power:
	@if [ ! -e $(OUTPUT)/$(POWER_REPORT) ]; then \
		make OUTPUT=$(OUTPUT) SYNTH_SCRIPT=$(SYNTH_SCRIPT) synth; \
	fi
	@printf "$uPower Report: $(OUTPUT)/$(POWER_REPORT):$n\n\n"
	@cat $(OUTPUT)/$(POWER_REPORT)

# View the area report from synthesis. If it doesn't exist, run synthesis.
view-area:
	@if [ ! -e $(OUTPUT)/$(AREA_REPORT) ]; then \
		make OUTPUT=$(OUTPUT) SYNTH_SCRIPT=$(SYNTH_SCRIPT) synth; \
	fi
	@printf "$uArea Report: $(OUTPUT)/$(AREA_REPORT):$n\n\n"
	@cat $(OUTPUT)/$(AREA_REPORT)

# Synthesize the processor into a physical design, generating reports on its
# area, timing, and power
$(SYNTH_REPORTS): $(447_SRC) $(SRC) $(DC_SCRIPT) | $(OUTPUT) \
			synth-check-compiler synth-check-script check-lab-number-valid
	@printf "Synthesizing design in $u$(OUTPUT)$n..."
	@cd $(OUTPUT) && $(SYNTH_CC) -f $(DC_SCRIPT) -x "set project_dir $(PWD);  \
		set lab_18447 $(LAB_18447)$(SET_CLOCK_PERIOD)" |& tee $(SYNTH_LOG)
	@printf "\nSynthesis has completed. The synthesis log can be found at "
	@printf "$u$(OUTPUT)/$(SYNTH_LOG)$n\n"
	@printf "The timing report can be found at $u$(OUTPUT)/$(TIMING_REPORT)$n\n"
	@printf "The power report can be found at $u$(OUTPUT)/$(POWER_REPORT)$n\n"
	@printf "The area report can be found at $u$(OUTPUT)/$(AREA_REPORT)$n\n"
	@if grep -i latch $(OUTPUT)/$(SYNTH_LOG) &> /dev/null; then \
		printf "\n\n\n\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX   Found disallowed latch inference. \n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX   grep -i latch $(OUTPUT)/$(SYNTH_LOG)  \n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
		printf "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"; \
	else \
		printf "No latches found in $u$(OUTPUT)/$(SYNTH_LOG)$n\n"; \
	fi 


# Clean up all the files generated by DC synthesis
synth-clean:
	@printf "Cleaning up synthesis files...\n"
	@rm -rf $(SYNTH_REPORTS) $(SYNTH_EXTRA_FILES)

# Very clean is the same as clean for synthesis
synth-veryclean: synth-clean

# Suppresses 'no rule to make...' error when the DC_SCRIPT doesn't exist
$(DC_SCRIPT):

# Check that the Verilog synthesis compiler exists
synth-check-compiler:
ifeq ($(shell which $(SYNTH_CC) 2> /dev/null),)
	@printf "$rError: $u$(SYNTH_CC)$n$r: Verilog synthesis compiler was not "
	@printf "found in your $bPATH$n$r.\n$n"
	@exit 1
endif

# Check that script used for synthesis exists
synth-check-script:
ifeq ($(wildcard $(SYNTH_SCRIPT)),)
	@printf "$rError: $u$(SYNTH_SCRIPT)$n$r: DC synthesis script does not "
	@printf "exist.\n$n"
	@exit 1
endif

################################################################################
# Help Target
################################################################################

# These targets don't correspond to actual generated files
.PHONY: help

# Display a help message about how to use the Makefile to the user
help:
	@printf "$b18-447 Makefile Usage:$n\n"
	@printf "\tmake [$uvariable$n ...] $utarget$n\n"
	@printf "\n"
	@printf ""
	@printf "$bTargets:$n\n"
	@printf "\t$bsim$n\n"
	@printf "\t    Runs the simulator with the specified $bTEST$n program.\n"
	@printf "\t    Builds the simulator and assembles the program $bTEST$n as\n"
	@printf "\t    necessary.\n"
	@printf "\n"
	@printf "\t$bsim-gui$n\n"
	@printf "\t    Performs the same actions as $bsim$n, but runs the\n"
	@printf "\t    specified $bTEST$n with the waveform viewer.\n"
	@printf "\n"
	@printf "\t$bverify$n\n"
	@printf "\t    Runs and verifies the specified $bTEST$n program. Takes\n"
	@printf "\t    the same steps as the $bsim$n target and then compares the\n"
	@printf "\t    simulation's register dump against the reference.\n"
	@printf "\n"
	@printf "\t$bautograde$n\n"
	@printf "\t    Runs and verifies all the programs specified by $bTESTS$n.\n"
	@printf "\t    Prints out a summary of the passing and failing programs,\n"
	@printf "\t    and suppresses the output of each test. If $bTESTS$n is\n"
	@printf "\t    not specified, then it defaults to the public tests for\n"
	@printf "\t    this lab.\n"
	@printf "\n"
	@printf "\t$bsynth$n\n"
	@printf "\t    Compiles the Verilog files in the $u$(SRC_DIR)$n into a\n"
	@printf "\t    physical design using the specified $bSYNTH_SCRIPT$n. All\n"
	@printf "\t    outputs are placed in $bOUTPUT$n.\n"
	@printf "\n"
	@printf "\t$bview-timing, view-power, view-area$n\n"
	@printf "\t    Displays the timing, power, or area report from the last\n"
	@printf "\t    synthesis run. Reruns 'synth' if the report doesn't exist.\n"
	@printf "\n"
	@printf "\t$bbuild$n\n"
	@printf "\t    Compiles the Verilog files in the $u$(SRC_DIR)$n directory\n"
	@printf "\t    into an executable. Generates an executable at\n"
	@printf "\t    $bOUTPUT$n$u/$(SIM_EXECUTABLE)$n.\n"
	@printf "\n"
	@printf "\t$bassemble$n\n"
	@printf "\t    Assembles the specified $bTEST$n program into binary files\n"
	@printf "\t    for each code section. The binary files are placed in the\n"
	@printf "\t    test's directory under $u<test_name>.<section>.bin$n.\n"
	@printf "\t    A disassembly of the compiled test is created at\n"
	@printf "\t    $u<test_name>.$(DISAS_EXTENSION)$n.\n"
	@printf "\n"
	@printf "\t$brefsim$n\n"
	@printf "\t    Runs the reference C simulator with the specified $bTEST$n program.\n"
	@printf "\t    Builds $bTEST$n as necessary.\n"
	@printf "\n"
	@printf "\t$brefdump$n\n"
	@printf "\t    Runs the reference C simulator with the specified $bTEST$n program.\n"
	@printf "\t    Builds $bTEST$n as necessary. Afterwards, a reference register dump\n"
	@printf "\t    is stored in regdump.reg at the top-level directory.\n"
	@printf "\n"
	@printf "\t$bclean$n\n"
	@printf "\t    Cleans up all of the files generated by compilation in the\n"
	@printf "\t    $bOUTPUT$n directory.\n"
	@printf "\n"
	@printf "\t$bveryclean$n\n"
	@printf "\t    Takes the same steps as the $bclean$n target and also\n"
	@printf "\t    cleans up all files in the project directory generated\n"
	@printf "\t    from assembling programs.\n"
	@printf "\n"
	@printf "$bVariables:$n\n"
	@printf "\t$bTEST$n\n"
	@printf "\t    The program to assemble or run with processor simulation.\n"
	@printf "\t    This is a single RISC-V assembly file or C file.\n"
	@printf "\n"
	@printf "\t$bTESTS$n\n"
	@printf "\t    A list of programs to verify processor simulation with.\n"
	@printf "\t    This is only used for the $bautograde$n target. The\n"
	@printf "\t    variable supports glob patterns, a list when quoted, or a\n"
	@printf "\t    single program. Defaults to the tests required to pass\n"
	@printf "\t    checkoff for the current lab.\n"
	@printf "\n"
	@printf "\t$bCLOCK_PERIOD$n\n"
	@printf "\t    The clock period (in nanoseconds) to use when synthesizing\n"
	@printf "\t    the design. This is used as the timing constraint for the\n"
	@printf "\t    DC synthesis script. Defaults to the maximum clock period\n"
	@printf "\t    required for the current lab.\n"
	@printf "\n"
	@printf "\t$bOUTPUT$n\n"
	@printf "\t    Specifies the output directory where generated files are\n"
	@printf "\t    are placed. For simulation targets, defaults to\n"
	@printf "\t    $u$(SIM_OUTPUT)$n. For the synthesis target, defaults to\n"
	@printf "\t    $u$(SYNTH_OUTPUT)$n.\n"
	@printf "\n"
	@printf "\t$bLAB_18447$n\n"
	@printf "\t    The lab at which to target the build system. This affects\n"
	@printf "\t    the tests required to be passed for autograding, some of\n"
	@printf "\t    the processor microarchitecture, and the timing required\n"
	@printf "\t    for synthesis. Defaults to lab $u$(DEFAULT_LAB_18447)$n.\n"
	@printf "\t    Must be one of $(VALID_LABS_STRING).\n"
	@printf "\n"
	@printf "\t$bSYNTH_SCRIPT$n\n"
	@printf "\t    The TCL script to use for synthesizing the processor into\n"
	@printf "\t    a design. This is only used for the 'synth' target.\n"
	@printf "\t    Defaults to $u$(SYNTH_SCRIPT)$n.\n"
	@printf "\n"
	@printf "$bExamples:$n\n"
	@printf "\tmake sim TEST=inputs/mytest.S\n"
	@printf "\tmake sim-gui TEST=inputs/mytest.S\n"
	@printf "\tmake verify TEST=inputs/mytest.S\n"
	@printf "\tmake autograde\n"
	@printf "\tmake autograde TESTS=inputs/mytest.S\n"
	@printf "\tmake autograde TESTS=\"inputs/mytest1.S inputs/mytest2.S\"\n"
	@printf "\tmake autograde TESTS=447inputs/*.S\n"
	@printf "\tmake synth\n"
	@printf "\tmake synth CLOCK_PERIOD=3.0\n"
	@printf "\tmake view-timing\n"