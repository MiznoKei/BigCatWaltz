################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/bluetooth.c \
../code/imu.c \
../code/menu.c \
../code/motor.c \
../code/pid.c \
../code/servo.c \
../code/sys.c 

COMPILED_SRCS += \
./code/bluetooth.src \
./code/imu.src \
./code/menu.src \
./code/motor.src \
./code/pid.src \
./code/servo.src \
./code/sys.src 

C_DEPS += \
./code/bluetooth.d \
./code/imu.d \
./code/menu.d \
./code/motor.d \
./code/pid.d \
./code/servo.d \
./code/sys.d 

OBJS += \
./code/bluetooth.o \
./code/imu.o \
./code/menu.o \
./code/motor.o \
./code/pid.o \
./code/servo.o \
./code/sys.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/01Univ/11_RaceCar/01_BigCatWaltz_v3.24/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/bluetooth.d ./code/bluetooth.o ./code/bluetooth.src ./code/imu.d ./code/imu.o ./code/imu.src ./code/menu.d ./code/menu.o ./code/menu.src ./code/motor.d ./code/motor.o ./code/motor.src ./code/pid.d ./code/pid.o ./code/pid.src ./code/servo.d ./code/servo.o ./code/servo.src ./code/sys.d ./code/sys.o ./code/sys.src

.PHONY: clean-code

