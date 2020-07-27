################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
ELFSIZE += \
hello_ultra96_u5_freertos.elf.size \


# All Target
all: hello_ultra96_u5_freertos.elf secondary-outputs

# Tool invocations
hello_ultra96_u5_freertos.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM R5 gcc linker'
	armr5-none-eabi-gcc -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -Wl,-T -Wl,../src/lscript.ld -L../../ultra96_r5_freertos_bsp_1/psu_cortexr5_0/lib -o "hello_ultra96_u5_freertos.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

hello_ultra96_u5_freertos.elf.size: hello_ultra96_u5_freertos.elf
	@echo 'Invoking: ARM R5 Print Size'
	armr5-none-eabi-size hello_ultra96_u5_freertos.elf  |tee "hello_ultra96_u5_freertos.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(S_UPPER_DEPS)$(C_DEPS)$(ELFSIZE) hello_ultra96_u5_freertos.elf
	-@echo ' '

secondary-outputs: $(ELFSIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
