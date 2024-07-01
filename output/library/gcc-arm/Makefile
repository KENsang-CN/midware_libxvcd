LIBRARY_PREFIX := lib
LIBRARY := xvcd
LIBRARY_EXTENSION := .a

PWD := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
C_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../source | grep -E ^[^.]*.c$}}
C_OBJS_LIST := ${patsubst %.c, %.o, $(C_SRC_LIST)}
CPP_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../source | grep -E ^[^.]*.cpp$}}
CPP_OBJS_LIST := ${patsubst %.cpp, %.o, $(CPP_SRC_LIST)}

CC := arm-linux-gnueabihf-gcc
CPPC := arm-linux-gnueabihf-g++
ar := arm-linux-gnueabihf-ar

CC_FLAGS := -DCOMPILING_XPROG -DXPROG_DLL \
	-I$(PWD)/../../../library/gcc-arm \
	-I$(PWD)/../../../include -Wall -c -fmessage-length=0 -O2
CPPC_FLAGS := $(CC_FLAGS)
AR_FLAGS := 

all: banner $(LIBRARY_PREFIX)$(LIBRARY)$(LIBRARY_EXTENSION)

banner:
	@echo c source list: $(C_SRC_LIST)
	@echo c++ source list: $(CPP_SRC_LIST)
	@echo build list: $(C_OBJS_LIST) $(CPP_OBJS_LIST)
	@echo ''

$(C_OBJS_LIST): %.o: $(PWD)/../../../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: gcc compiler'
	$(CC) $(CC_FLAGS) -c -o $@ $<
	@echo 'Finished building: $<'
	@echo ''

$(CPP_OBJS_LIST): %.o: $(PWD)/../../../source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: g++ compiler'
	$(CPPC) $(CPPC_FLAGS) -c -o $@ $<
	@echo 'Finished building: $<'
	@echo ''

$(LIBRARY_PREFIX)$(LIBRARY)$(LIBRARY_EXTENSION): $(C_OBJS_LIST) $(CPP_OBJS_LIST)
	@echo 'Building target: $@'
	@echo 'Invoking: gcc archiver'
	$(AR) -r $@ $^ $(AR_FLAGS)
	@echo 'Finished building target: $@'
	@echo ''
	
clean:
	@rm -rf $(PWD)/*.o
	@rm -rf $(LIBRARY_PREFIX)$(LIBRARY)$(LIBRARY_EXTENSION)
