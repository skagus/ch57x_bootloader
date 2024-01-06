
ifeq ($(BUILD_PRINT), TRUE)
	ECHO = 
else
	ECHO = @
endif

################ Generated Files ##############
VER_FILE = version.h
GEN_LDS = $(TARGET).lds

################ Object ##############
OBJ =	$(CSRC:%.c=$(OBJDIR)/%.c.o) \
		$(CPPSRC:%.cpp=$(OBJDIR)/%.p.o) \
		$(ASRC:%.S=$(OBJDIR)/%.s.o) \
		$(DATA:%.bin=$(OBJDIR)/%.b.o)

LST =	$(OBJ:%.o=%.lst)

DEP =	$(OBJ:%.o=%.d)

DEF_OPT = $(patsubst %,-D%,$(DEFINE))

LIB_OPT =	$(patsubst %,-L$(PRJ_TOP)/%,$(PRJ_LIB_DIR)) \
			$(patsubst %,-L%,$(EXT_LIB_DIR)) \
			$(patsubst %,-l%,$(LIB_FILE))

INCLUDE =	$(patsubst %,-I$(PRJ_TOP)/%,$(PRJ_INC)) \
			$(patsubst %,-I%,$(EXT_INC))

################  Common Options for C, C++, ASM
DEP_FLAGS = -MMD -MP # dependancy generation.

#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
FLAGS  = -march=$(MCU)
#FLAGS += -mabi=ilp32
#FLAGS += -mabi=lp64
FLAGS += -mcmodel=medany
FLAGS += -msmall-data-limit=8
FLAGS += -std=gnu99 # c89, c99, gnu89, gnu99

FLAGS += -funsigned-char
FLAGS += -funsigned-bitfields
FLAGS += -fshort-enums
#FLAGS += -fpack-struct  # CAUTION: RISC-V can't handle un-aligned load/save.
FLAGS += -fmessage-length=0

FLAGS += $(OPTI)
#FLAGS += -msave-restore 
FLAGS += -ffunction-sections
FLAGS += -fdata-sections 
#FLAGS += -fno-unit-at-a-time
#FLAGS += -mshort-calls
FLAGS += -g
FLAGS += -Wa,-adhlns=$(@:.o=.lst)
FLAGS += -gdwarf-2
FLAGS += -Wno-comment

ifeq ($(BUILD_STRICT), TRUE)
FLAGS += -Wall
FLAGS += -Wunused
FLAGS += -Wextra
FLAGS += -Wstrict-prototypes
#CFLAGS += -Wundef # for undefined macro evaluation
FLAGS += -Wunreachable-code
FLAGS += -Wno-attributes # 
endif

################  C Options
CFLAGS  = $(FLAGS)
CFLAGS += $(DEP_FLAGS)
CFLAGS += $(DEF_OPT)
CFLAGS += $(INCLUDE)
# CFLAGS += -fstack-usage  # show stack usage for each function.
CFLAGS += -fcallgraph-info  # make call graph information.
#CFLAGS += -fdump-tree-optimized #
#CFLAGS += -fdump-rtl-dfinish  # 
################  C++ Options
CPPFLAGS  = $(FLAGS)
CPPFLAGS += $(DEP_FLAGS)
CPPFLAGS += $(DEF_OPT)
CPPFLAGS += $(INCLUDE)

################  Assembler Options
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex 
#       dump that will be displayed for a given single line of source input.
ASFLAGS  = $(FLAGS)
#ASFLAGS += -Wa,-gstabs,--listing-cont-lines=100
ASFLAGS += $(DEP_FLAGS)
ASFLAGS += $(DEF_OPT)
ASFLAGS += $(INCLUDE)

################  Linker Options
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDS_OPT = $(patsubst %, -T $(PRJ_TOP)/%, $(LINK_SCRIPT))

LDFLAGS = -g
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(LDS_OPT)
LDFLAGS += -fstack-usage
LDFLAGS += -nostartfiles
#LDFLAGS += -nodefaultlibs
LDFLAGS += -nolibc
#LDFLAGS += -nostdlib
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += -Wl,--print-gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -march=$(MCU)
LDFLAGS += --specs=nosys.specs
#LDFLAGS += --specs=nano.specs
#LDFLAGS += -mcmodel=medany
#LDFLAGS += -msmall-data-limit=8
LDFLAGS += -fmessage-length=0
LDFLAGS += -ffunction-sections
LDFLAGS += -fdata-sections
#LDFLAGS += -fsigned-char
#LDFLAGS += -fno-common
#LDFLAGS += -mabi=ilp32
#LDFLAGS += -mabi=lp64
################### Action #################
# add @ for silent.
ifeq ($(GNU_PREFIX),)
	GNU_PREFIX = riscv-none-sss-
endif

CC 		= $(GNU_PREFIX)gcc
OBJCOPY	= $(GNU_PREFIX)objcopy
OBJDUMP	= $(GNU_PREFIX)objdump
SIZE	= $(GNU_PREFIX)size
AR		= $(GNU_PREFIX)ar rcs
NM		= $(GNU_PREFIX)nm

SHELL = sh
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp

INFO 	= @echo Making: $@
DIR_CHK = @mkdir -p $(@D)

################ Object file
$(OBJDIR)/%.b.o : $(PRJ_TOP)/%.bin
	$(INFO)
	$(DIR_CHK)
	$(OBJCOPY) -I binary -O elf32 $< $@

$(OBJDIR)/%.c.o : $(PRJ_TOP)/%.c
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(CC) -c $(CFLAGS) $< -o $@ 

$(OBJDIR)/%.p.o : $(PRJ_TOP)/%.cpp
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(CC) -c $(CPPFLAGS) $< -o $@ 

$(OBJDIR)/%.s.o : $(PRJ_TOP)/%.S
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(CC) -c $(ASFLAGS) $< -o $@

################### File Creation #################
all: elf bin lst hex size

clean: clean_obj clean_target

relink: clean_target elf bin lst hex size

$(GEN_LDS): $(PRJ_TOP)/$(LINK_SCRIPT)
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(CC) -E -P -x c $(FLAGS) $(DEF_OPT) $(INCLUDE) $< > $@

$(PRJ_TOP)/$(VER_FILE):
	$(INFO)
	@echo VER_STRING: $(VER_STRING)
	@echo "#define VERSION		\"$(VER_STRING)\"" > $(PRJ_TOP)/$(VER_FILE)

%.elf: $(OBJ) $(GEN_LDS)
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(CC) $(LDFLAGS) $(OBJ) $(LIB_OPT) -o $@ 

%.hex: %.elf
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(OBJCOPY) -O ihex  $< $@

%.bin: %.elf
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(OBJCOPY) -O binary $< $@

%.lss: %.elf
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(OBJDUMP) -h -S -z -w $< > $@

%.sym: %.elf
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(NM) -S -l -n --format=sysv $< > $@

%.a: $(OBJ)
	$(INFO)
	$(DIR_CHK)
	$(ECHO)$(AR) $@ $^

################ Actions.
TARGET_ALL=	$(TARGET).elf $(TARGET).a \
			$(TARGET).hex $(TARGET).bin \
			$(TARGET).lss $(TARGET).sym $(TARGET).map

rm_ver:
	rm -f $(PRJ_TOP)/$(VER_FILE)

version: rm_ver $(PRJ_TOP)/$(VER_FILE)

obj: $(OBJ)
lib: $(TARGET).a
elf: $(TARGET).elf
bin: $(TARGET).bin $(TARGET).hex
lst: $(TARGET).lss $(TARGET).sym

size: $(TARGET).elf
	$(INFO)
	@$(SIZE) --format=gnu $<
	@$(SIZE) --format=gnu $< > $(TARGET).size
	@$(SIZE) --format=sysv --radix=16 --common $< >> $(TARGET).size

# Display compiler version information.
gccversion : 
	$(ECHO)$(CC) --version

conf :
	@echo GNU_PREFIX : $(GNU_PREFIX)
	@echo ------------------------ Flags ------------------------
	@echo CFLAGS : $(CFLAGS)
	@echo LDFLAGS: $(LDFLAGS)
	@echo ASFLAGS: $(ASFLAGS)
	@echo LD FILE: $(LINK_SCRIPT)
	@echo ------------------------ Files ------------------------
	@echo ASM Files : $(ASMSRC)
	@echo C Files   : $(CSRC)
	@echo C++ Files : $(CPPSRC)

clean_target:
	$(REMOVE) $(TARGET).elf $(TARGET).a \
			$(TARGET).hex $(TARGET).bin \
			$(TARGET).lss $(TARGET).sym \
			$(TARGET).map

clean_obj:
	$(REMOVEDIR) $(OBJDIR)

relink: clean_target all

# Include the dependency files.
-include $(patsubst %.c,$(OBJDIR)/%.c.d,$(CSRC))
-include $(patsubst %.cpp,$(OBJDIR)/%.p.d,$(CPPSRC))
-include $(patsubst %.S,$(OBJDIR)/%.S.d,$(ASRC))

# Listing of phony targets.
.PHONY : all obj elf bin hex lst size gccversion clean version
