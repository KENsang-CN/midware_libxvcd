APP := xvcd_test
APP_EXTENSION := .elf

PWD := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
C_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../test | grep -E ^[^.]*.c$}}
C_OBJS_LIST := ${patsubst %.c, %.o, $(C_SRC_LIST)}
CPP_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../test | grep -E ^[^.]*.cpp$}}
CPP_OBJS_LIST := ${patsubst %.cpp, %.o, $(CPP_SRC_LIST)}

CC := gcc
CPPC := g++
LD := gcc

CC_FLAGS := -I$(PWD)/../../../include -Wall -c -fmessage-length=0 -g3 -O0
CPPC_FLAGS := $(CC_FLAGS)
LD_FLAGS := \
	-L$(PWD)/../../../library/gcc/ \
	-L$(PWD)/../../library/gcc/ \
	-lxvcd -lftd2xx -lpthread 

all: banner $(APP)$(APP_EXTENSION)

banner:
	@echo c source list: $(C_SRC_LIST)
	@echo c++ source list: $(CPP_SRC_LIST) 
	@echo build list: $(C_OBJS_LIST) $(CPP_OBJS_LIST)
	@echo ''

$(C_OBJS_LIST): %.o: $(PWD)/../../../test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: gcc compiler'
	$(CC) $(CC_FLAGS) -c -o $@ $<
	@echo 'Finished building: $<'
	@echo ''

$(CPP_OBJS_LIST): %.o: $(PWD)/../../../test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: g++ compiler'
	$(CPPC) $(CPPC_FLAGS) -c -o $@ $<
	@echo 'Finished building: $<'
	@echo ''

$(APP)$(APP_EXTENSION): $(C_OBJS_LIST) $(CPP_OBJS_LIST)
	@echo 'Building target: $@'
	@echo 'Invoking: gcc linker'
	$(LD) -o $@ $^ $(LD_FLAGS)
	@echo 'Finished building target: $@'
	@echo ''
	
clean:
	@rm -rf $(PWD)/*.o $(PWD)/*.dll
	@rm -rf $(APP)$(APP_EXTENSION)
