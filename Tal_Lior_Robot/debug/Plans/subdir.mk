################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Plans/Plan.cpp \
../Plans/PlnObstacleAvoid.cpp 

OBJS += \
./Plans/Plan.o \
./Plans/PlnObstacleAvoid.o 

CPP_DEPS += \
./Plans/Plan.d \
./Plans/PlnObstacleAvoid.d 


# Each subdirectory must supply rules for building sources it contributes
Plans/%.o: ../Plans/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/player-2.0 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


