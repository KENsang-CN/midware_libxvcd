LIBRARY_PREFIX := lib
LIBRARY := xvcd
LIBRARY_EXTENSION := .dll

PWD := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
C_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../source | grep -E ^[^.]*.c$}}
C_OBJS_LIST := ${patsubst %.c, %.o, $(C_SRC_LIST)}
CPP_SRC_LIST := ${notdir ${shell ls $(PWD)/../../../source | grep -E ^[^.]*.cpp$}}
CPP_OBJS_LIST := ${patsubst %.cpp, %.o, $(CPP_SRC_LIST)}

CC := $(MINGW64)\gcc.exe
CPPC := $(MINGW64)\g++
LD := $(MINGW64)\gcc

CC_FLAGS := -DCOMPILING_XVCD -DXVCD_DLL \
	-I$(PWD)/../../../include -Wall -c -fmessage-length=0 -g3 -O0
CPPC_FLAGS := $(CC_FLAGS)
LD_FLAGS := -shared -static -Wl,--output-def=$(LIBRARY_PREFIX)$(LIBRARY).def \
	-L$(PWD)/../../../library/mingw64/ -lftd2xx -lws2_32 -lpthread

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
	@echo 'Invoking: gcc linker'
	$(LD) -o $@ $^ $(LD_FLAGS)
	@echo 'Finished building target: $@'
	@echo ''
	
clean:
	@rm -rf $(PWD)/*.o $(PWD)/*.def
	@rm -rf $(LIBRARY_PREFIX)$(LIBRARY)$(LIBRARY_EXTENSION)

