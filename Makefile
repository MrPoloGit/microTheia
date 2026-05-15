MAKEFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

RUN_TAG = $(shell ls librelane/runs/ | tail -n 1)
TOP = chip_top

PDK_ROOT ?= $(MAKEFILE_DIR)/gf180mcu
PDK ?= gf180mcuD
PDK_TAG ?= 1.8.0
SCL ?= gf180mcu_as_sc_mcu7t3v3

AVAILABLE_SLOTS = 1x1 0p5x1 1x0p5 0p5x0p5
DEFAULT_SLOT = 1x1

# Slot can be any of AVAILABLE_SLOTS
SLOT ?= $(DEFAULT_SLOT)

ifeq ($(SLOT),default)
    SLOT = $(DEFAULT_SLOT)
endif

ifeq ($(filter $(SLOT),$(AVAILABLE_SLOTS)),)
    $(error $(SLOT) does not exist in AVAILABLE_SLOTS: $(AVAILABLE_SLOTS))
endif

.DEFAULT_GOAL := help

# Select design to test
SIM_DUTS = $(strip $(DUT))

# System Verilog sources
SV_SRCS := $(shell find src -name "*.sv")

# Simulation configuration
CONFIG ?= voxel_default
CONFIG_FILE := configs/$(CONFIG).txt

# iCE40 FPGA Flow Wrapper
ICE40_MAKEFILE := ice40/ice40.mk

# Default testbench module name is <DUT>_tb, but you can override it:
# Example: make sim DUT=voxel_bin_core_parallel TB=voxel_bin_core
TB ?= $(DUT)

help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z0-9_-]+:[^#]*## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":[^#]*## "}; {printf "  %-20s %s\n", $$1, $$2}'
.PHONY: help

all: librelane ## Build the project (runs LibreLane)
.PHONY: all

clone-pdk: ## Clone the GF180MCU PDK repository
	rm -rf $(MAKEFILE_DIR)/gf180mcu
	git clone https://github.com/wafer-space/gf180mcu.git $(MAKEFILE_DIR)/gf180mcu --depth 1 --branch ${PDK_TAG}
.PHONY: clone-pdk

install-3v3-scl: ## Install the 3.3V standard cell library into the PDK
	git submodule update --init libs/gf180mcu_as_sc_mcu7t3v3 libs/gf180mcu_ocd_ip_sram
	cp -r $(MAKEFILE_DIR)/libs/gf180mcu_as_sc_mcu7t3v3/pdk/libs.ref/gf180mcu_as_sc_mcu7t3v3 $(PDK_ROOT)/$(PDK)/libs.ref/
	cp -r $(MAKEFILE_DIR)/libs/gf180mcu_as_sc_mcu7t3v3/pdk/libs.tech/librelane $(PDK_ROOT)/$(PDK)/libs.tech/
	cp -r $(MAKEFILE_DIR)/libs/gf180mcu_as_sc_mcu7t3v3/pdk/libs.tech/magic $(PDK_ROOT)/$(PDK)/libs.tech/
	cp $(MAKEFILE_DIR)/librelane/gf180mcu_as_sc_mcu7t3v3_config.tcl $(PDK_ROOT)/$(PDK)/libs.tech/librelane/gf180mcu_as_sc_mcu7t3v3/config.tcl
.PHONY: install-3v3-scl

librelane: ## Run LibreLane flow (synthesis, PnR, verification)
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --save-views-to $(MAKEFILE_DIR)/final --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk
.PHONY: librelane

librelane-nodrc: ## Run LibreLane flow without DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --save-views-to $(MAKEFILE_DIR)/final --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip KLayout.Antenna --skip KLayout.DRC --skip Magic.DRC
.PHONY: librelane-nodrc

librelane-klayoutdrc: ## Run LibreLane flow without magic DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --save-views-to $(MAKEFILE_DIR)/final --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip Magic.DRC
.PHONY: librelane-klayoutdrc

librelane-magicdrc: ## Run LibreLane flow without KLayout DRC checks
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --save-views-to $(MAKEFILE_DIR)/final --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --skip KLayout.DRC
.PHONY: librelane-magicdrc

librelane-openroad: ## Open the last run in OpenROAD
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --last-run --flow OpenInOpenROAD
.PHONY: librelane-openroad

librelane-klayout: ## Open the last run in KLayout
	librelane librelane/slots/slot_${SLOT}.yaml librelane/config.yaml --pdk ${PDK} --pdk-root ${PDK_ROOT} --scl ${SCL} --manual-pdk --last-run --flow OpenInKLayout
.PHONY: librelane-klayout

librelane-padring: ## Only create the padring
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/padring.py librelane/slots/slot_${SLOT}.yaml librelane/config.yaml
.PHONY: librelane-padring

lint: ## Lint all SystemVerilog files in src
	verilator --lint-only \
	          -Wall \
	          -Wno-fatal \
	          -flife lint.vlt \
	          $(SV_SRCS)
.PHONY: lint

sim: ## Run RTL simulation with cocotb (DUT=chip_top runs chip_top tb)
	@if [ -z "$(DUT)" ]; then \
		echo "Error: You must specify DUT=<module_name>"; \
		echo "Example: make sim DUT=voxel_bin_top"; \
		exit 1; \
	elif [ "$(DUT)" = "chip_top" ]; then \
		$(MAKE) sim-chip-top; \
	else \
		for d in $(SIM_DUTS); do \
			echo "===================================================="; \
			echo " Running DUT=$$d with CONFIG=$(CONFIG)"; \
			echo "===================================================="; \
			if [ ! -f "cocotb/$(TB)_tb.py" ]; then \
				echo "Skipping $$d (no testbench found: cocotb/$(TB)_tb.py)"; \
				continue; \
			fi; \
			rm -rf cocotb/sim_build/$$d; \
			SRCS=$$(grep -v '^[[:space:]]*$$' src/rtl.f | grep -v '^[[:space:]]*#' | tr -d '\r' | tr '\n' ' '); \
			PARAMS=$$(PYTHONPATH=cocotb SIM_CONFIG=$(CONFIG_FILE) python3 -m util.config_parser $$d); \
			export SIM_CONFIG=$(CONFIG_FILE); \
			TOPLEVEL=$$d \
			TOPLEVEL_LANG=verilog \
			COCOTB_TEST_MODULES=$(TB)_tb \
			VERILOG_SOURCES="$$SRCS" \
			COMPILE_ARGS="$$PARAMS" \
			WAVES=1 \
			SIM_BUILD=cocotb/sim_build/$$d \
			PYTHONPATH=cocotb \
			make -f $$(cocotb-config --makefiles)/Makefile.sim results.xml; \
		done \
	fi
.PHONY: sim

sim-fast: ## Run voxel_bin_core sim with small fast-sim config (8x8 grid, N=8, 4 bins)
	$(MAKE) sim DUT=voxel_bin_core CONFIG=voxel_sim_fast
.PHONY: sim-fast

sim-all: ## Test all the modules against Makefile compile args
	$(MAKE) sim DUT=sram_wrapper CONFIG=sram_wrapper
	$(MAKE) sim DUT=input_fifo
	$(MAKE) sim DUT=evt2_decoder
	$(MAKE) sim DUT=voxel_gesture_classifier
	$(MAKE) sim DUT=voxel_mac_engine
	$(MAKE) sim DUT=voxel_binning
	$(MAKE) sim DUT=voxel_bin_core
.PHONY: sim-all

SLOT_UPPER    := $(shell echo $(SLOT) | tr 'a-z' 'A-Z')
CHIP_TOP_SRCS := src/chip_top.sv src/chip_core.sv src/soc.sv src/spi_wrapper.sv \
    			 src/control_fsm.sv src/evt2_decoder.sv src/sram_wrapper.sv src/input_fifo.sv \
    			 src/selectable_debug.sv src/voxel_bin_core.sv src/voxel_binning.sv \
    			 src/voxel_gesture_classifier.sv src/voxel_mac_engine.sv \
    			 third_party/verilog_spi/spi_module.v third_party/verilog_spi/pos_edge_det.v third_party/verilog_spi/neg_edge_det.v \
    			 ip/gf180mcu_ws_ip__id/vh/gf180mcu_ws_ip__id.v \
    			 ip/gf180mcu_ws_ip__logo/vh/gf180mcu_ws_ip__logo.v
CHIP_TOP_PDK_IO := $(PDK_ROOT)/$(PDK)/libs.ref/gf180mcu_fd_io/verilog/gf180mcu_fd_io.v

# Use real PDK IO/SRAM models when available, otherwise fall back to behavioral stubs
CHIP_TOP_IO_SRCS := $(if $(wildcard $(CHIP_TOP_PDK_IO)),\
    $(PDK_ROOT)/$(PDK)/libs.ref/gf180mcu_fd_io/verilog/gf180mcu_fd_io.v \
    $(PDK_ROOT)/$(PDK)/libs.ref/gf180mcu_fd_io/verilog/gf180mcu_ws_io.v \
    $(PDK_ROOT)/$(PDK)/libs.ref/gf180mcu_fd_ip_sram/verilog/gf180mcu_fd_ip_sram__sram512x8m8wm1.v,\
    sim/io_stubs.v)

sim-chip-top: ## Run chip_top RTL simulation with cocotb
	@echo "IO sources: $(CHIP_TOP_IO_SRCS)"
	rm -rf cocotb/sim_build/chip_top
	TOPLEVEL=chip_top \
	TOPLEVEL_LANG=verilog \
	COCOTB_TEST_MODULES=chip_top_tb \
	VERILOG_SOURCES="$(CHIP_TOP_SRCS) $(CHIP_TOP_IO_SRCS)" \
	COMPILE_ARGS="-DSLOT_$(SLOT_UPPER) -I$(MAKEFILE_DIR)/src" \
	WAVES=1 \
	SIM_BUILD=cocotb/sim_build/chip_top \
	PYTHONPATH=cocotb \
	make -f $$(cocotb-config --makefiles)/Makefile.sim results.xml
.PHONY: sim-chip-top

sim-gl: ## Run gate-level simulation with cocotb (Icarus; after copy-final)
	cd cocotb; GL=1 PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py
.PHONY: sim-gl

sim-gl-verilator: ## Run gate-level simulation with Verilator (faster than Icarus)
	cd cocotb; LD_LIBRARY_PATH="" SIM=verilator GL=1 PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} python3 chip_top_tb.py
.PHONY: sim-gl-verilator

# Run the 4 gesture classifications in parallel, one process per core.
# Each gets its own SIM_BUILD dir, log file, and results.xml.
#
# Uses Icarus, NOT Verilator, because Verilator does not propagate cocotb
# writes to top-level `inout` ports (clk_PAD/rst_n_PAD/input_PAD) through
# the IO-pad model's `assign Y = PAD` to the chip-internal post-pad signals.
# In Verilator GL the chip stays in reset and the SPI never moves. Icarus
# handles inout deposits correctly. ~7h wall time per gesture; 4 in
# parallel ≈ 7h total.
sim-gl-parallel: ## Run all 4 gestures in parallel with Icarus (1 core per gesture)
	@mkdir -p logs
	@echo "Launching 4 parallel GL classify runs (gestures 0-3, Icarus) …"
	@set -e ; for g in 0 1 2 3 ; do \
		LD_LIBRARY_PATH="" SIM=icarus GL=1 \
		  PDK_ROOT=${PDK_ROOT} PDK=${PDK} SLOT=${SLOT} \
		  GESTURE_INDICES=$$g \
		  COCOTB_TEST_FILTER=test_classify_all_gestures \
		  SIM_BUILD=$(MAKEFILE_DIR)/cocotb/sim_build_gl_g$$g \
		  RESULTS_XML=$(MAKEFILE_DIR)/logs/results_gl_g$$g.xml \
		  python3 $(MAKEFILE_DIR)/cocotb/chip_top_tb.py \
		  > $(MAKEFILE_DIR)/logs/gls_gesture_$$g.log 2>&1 & \
		echo "  PID $$! → gesture $$g, log: logs/gls_gesture_$$g.log" ; \
	done ; \
	wait
	@echo
	@echo "All gesture runs finished. Summary:"
	@for g in 0 1 2 3 ; do \
		echo "  Gesture $$g: $$(grep -oE 'PASS=[0-9]+ FAIL=[0-9]+ SKIP=[0-9]+' logs/gls_gesture_$$g.log | tail -1)" ; \
	done
.PHONY: sim-gl-parallel

sim-view: ## View simulation waveforms in GTKWave
	gtkwave cocotb/sim_build/chip_top.fst
.PHONY: sim-view

copy-final: ## Copy final output files from the last run
	rm -rf final/
	cp -r librelane/runs/${RUN_TAG}/final/ final/
.PHONY: copy-final

render-image: ## Render an image from the final layout (after copy-final)
	mkdir -p img/
	PDK_ROOT=${PDK_ROOT} PDK=${PDK} python3 scripts/lay2img.py final/gds/${TOP}.gds img/${TOP}.png --width 2048 --oversampling 4
.PHONY: render-image

ice40: ## Run ice40 FPGA build
	$(MAKE) -C ice40 -f ice40.mk ARCH=$(ARCH)

ice40-prog: ## Program ice40 board
	$(MAKE) -C ice40 -f ice40.mk prog ARCH=$(ARCH)

ice40-timing: ## Timing report for ice40 build
	$(MAKE) -C ice40 -f ice40.mk timing ARCH=$(ARCH)

ice40-clean: ## Cleans out all ice40 logic
	$(MAKE) -C ice40 -f ice40.mk clean

clean: ## Cleans the generated files
	rm -rf results.xml sim_build/
.PHONY: clean
