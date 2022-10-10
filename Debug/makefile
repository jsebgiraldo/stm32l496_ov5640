################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/BSP/STM32L496G-Discovery/subdir.mk
-include Core/Src/APIs/subdir.mk
-include Core/Src/subdir.mk
-include Core/Inc/BSP/Components/wm8994/subdir.mk
-include Core/Inc/BSP/Components/stmpe811/subdir.mk
-include Core/Inc/BSP/Components/stmpe1600/subdir.mk
-include Core/Inc/BSP/Components/st7789h2/subdir.mk
-include Core/Inc/BSP/Components/st7735/subdir.mk
-include Core/Inc/BSP/Components/st25dv/subdir.mk
-include Core/Inc/BSP/Components/ov9655/subdir.mk
-include Core/Inc/BSP/Components/mfxstm32l152/subdir.mk
-include Core/Inc/BSP/Components/m24sr/subdir.mk
-include Core/Inc/BSP/Components/lsm6dsl/subdir.mk
-include Core/Inc/BSP/Components/lsm303dlhc/subdir.mk
-include Core/Inc/BSP/Components/lsm303c/subdir.mk
-include Core/Inc/BSP/Components/ls016b8uy/subdir.mk
-include Core/Inc/BSP/Components/lps22hb/subdir.mk
-include Core/Inc/BSP/Components/lis3mdl/subdir.mk
-include Core/Inc/BSP/Components/l3gd20/subdir.mk
-include Core/Inc/BSP/Components/hx8347i/subdir.mk
-include Core/Inc/BSP/Components/hx8347g/subdir.mk
-include Core/Inc/BSP/Components/hts221/subdir.mk
-include Core/Inc/BSP/Components/ft6x06/subdir.mk
-include Core/Inc/BSP/Components/ft5336/subdir.mk
-include Core/Inc/BSP/Components/ft3x67/subdir.mk
-include Core/Inc/BSP/Components/cy8c4014lqi/subdir.mk
-include Core/Inc/BSP/Components/cs43l22/subdir.mk
-include Core/Inc/BSP/Components/cs42l51/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Camera
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Camera.elf \

MAP_FILES += \
Camera.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Camera.list \


# All Target
all: main-build

# Main-build Target
main-build: Camera.elf secondary-outputs

# Tool invocations
Camera.elf Camera.map: $(OBJS) $(USER_OBJS) C:\Sebas\STM32\STM32L496G-DISCO\projects\Camera\STM32L496AGIX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Camera.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Sebas\STM32\STM32L496G-DISCO\projects\Camera\STM32L496AGIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Camera.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Camera.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Camera.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Camera.elf Camera.list Camera.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
