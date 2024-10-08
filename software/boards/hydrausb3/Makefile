RM := rm -rf

# Check and choose riscv compiler either closed source from MounRiver Studio riscv-none-embed" or open source one GCC riscv-none-elf 
# For open source GCC riscv-none-elf see https://github.com/hydrausb3/riscv-none-elf-gcc-xpack/releases/
COMPILER_PREFIX := $(shell command -v riscv-none-embed-gcc >/dev/null 2>&1 && echo "riscv-none-embed" || true)
COMPILER_PREFIX := $(if $(COMPILER_PREFIX),$(COMPILER_PREFIX),$(shell command -v riscv-none-elf-gcc >/dev/null 2>&1 && echo "riscv-none-elf" || true))

ifeq ($(COMPILER_PREFIX),riscv-none-embed)
    MARCH_OPT := -march=rv32imac
else ifeq ($(COMPILER_PREFIX),riscv-none-elf)
    MARCH_OPT := -march=rv32imac_zicsr
else
    $(error Unknown COMPILER_PREFIX: $(COMPILER_PREFIX))
endif

# Define option(s) defined in pre-processor compiler option(s)
DEFINE_OPTS = -DDEBUG=1
# Optimisation option(s)
OPTIM_OPTS = -O3
# Debug option(s)
DEBUG = -g

BUILD_DIR = ./build

PROJECT = $(BUILD_DIR)/sucrela_fw
BSP_DIR     = ../../../wch-ch56x-bsp

RVMSIS_DIR  = $(BSP_DIR)/rvmsis
RVMSIS_SRCS = $(wildcard $(RVMSIS_DIR)/*.c)
OBJS       += $(patsubst $(RVMSIS_DIR)/%.c,$(BUILD_DIR)/%.o,$(RVMSIS_SRCS))

DRV_DIR   = $(BSP_DIR)/drv
DRV_SRCS  = $(wildcard $(DRV_DIR)/*.c)
OBJS     += $(patsubst $(DRV_DIR)/%.c,$(BUILD_DIR)/%.o,$(DRV_SRCS))

BOARD_DIR   = $(BSP_DIR)/board
BOARD_SRCS  = $(BOARD_DIR)/hydrausb3_v1.c
OBJS     += $(patsubst $(BOARD_DIR)/%.c,$(BUILD_DIR)/%.o,$(BOARD_SRCS))

USB_DIR   = $(BSP_DIR)/usb/usb_devbulk
USB_SRCS  = $(wildcard $(USB_DIR)/*.c)
OBJS        += $(patsubst $(USB_DIR)/%.c,$(BUILD_DIR)/%.o,$(USB_SRCS))

USER_DIR  = ./
USER_SRCS = $(wildcard $(USER_DIR)/*.c)
OBJS     += $(patsubst $(USER_DIR)/%.c,$(BUILD_DIR)/%.o,$(USER_SRCS))

# All of the sources participating in the build are defined here
OBJS += $(BUILD_DIR)/startup_CH56x.o
DEPS  = $(subst .o,.d,$(OBJS))
LIBS  =

SOC_GENERATED_HEADERS = ../../../build/gsd_orangecrab/software/include
UARTBONE_DIR  = ../../libuartbone
UARTBONE_SRCS = $(UARTBONE_DIR)/uartbone.c
OBJS         += $(patsubst $(UARTBONE_DIR)/%.c,$(BUILD_DIR)/%.o,$(UARTBONE_SRCS))

BASE_OPTS = $(MARCH_OPT) -mabi=ilp32 -msmall-data-limit=8 $(OPTIM_OPTS) -Wno-discarded-qualifiers -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
C_OPTS    = $(BASE_OPTS) $(DEBUG) $(DEFINE_OPTS)\
              $(INCLUDES) -std=gnu99 -MMD -MP -MT"$(@)"
LD_OPTS   = -T "$(BSP_DIR)/ld/.ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs --specs=nosys.specs

INCLUDES = \
  -I"$(RVMSIS_DIR)" \
  -I"$(DRV_DIR)" \
  -I"$(BOARD_DIR)" \
  -I"$(USB_DIR)" \
  -I"$(USER_DIR)" \
  -I"$(BUILD_DIR)" \
  -I"$(UARTBONE_DIR)"

# Add inputs and outputs from these tool invocations to the build variables
SECONDARY_FLASH += $(PROJECT).hex $(PROJECT).bin
SECONDARY_LIST  += $(PROJECT).lst
SECONDARY_SIZE  += $(PROJECT).siz
SECONDARY_MAP   += $(PROJECT).map

SECONDARY_OUTPUTS = $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(SECONDARY_MAP)
secondary-outputs: $(SECONDARY_OUTPUTS)

# All Target
all: $(PROJECT).elf secondary-outputs

.PRECIOUS: $(BUILD_DIR)/. $(BUILD_DIR)%/.

$(BUILD_DIR)/.:
	mkdir -p $@

$(BUILD_DIR)%/.:
	mkdir -p $@

.SECONDEXPANSION:

$(BUILD_DIR)/startup_CH56x.o: $(BSP_DIR)/startup/startup_CH56x.S | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -x assembler -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/csr.h: $(SOC_GENERATED_HEADERS)/generated/csr.h
	grep 'define CSR_' $< > $@

$(BUILD_DIR)/%.o: $(USER_DIR)/%.c $(BUILD_DIR)/csr.h | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(UARTBONE_DIR)/%.c | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(RVMSIS_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(DRV_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(BOARD_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(USB_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	mkdir -p $(@D)
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

# Tool invocations
$(PROJECT).elf: $(OBJS)
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	$(COMPILER_PREFIX)-gcc $(BASE_OPTS) $(LD_OPTS) -o "$(PROJECT).elf" $(OBJS) $(LIBS)
	@echo ' '

$(PROJECT).hex: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	$(COMPILER_PREFIX)-objcopy -O ihex "$(PROJECT).elf"  "$(PROJECT).hex"
	@echo ' '

$(PROJECT).bin: $(PROJECT).elf
	-@echo 'Create Flash Image BIN'
	-$(COMPILER_PREFIX)-objcopy -O binary "$(PROJECT).elf"  "$(PROJECT).bin"
	-@echo ' '

$(PROJECT).lst: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	$(COMPILER_PREFIX)-objdump --source --all-headers --demangle --line-numbers --wide "$(PROJECT).elf" > "$(PROJECT).lst"
	@echo ' '

$(PROJECT).siz: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	$(COMPILER_PREFIX)-size --format=berkeley "$(PROJECT).elf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(DEPS) $(SECONDARY_OUTPUTS) $(PROJECT).elf
	-@echo ' '

.PHONY: all clean dependents
