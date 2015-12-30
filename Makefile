SOURCE_DIR  = src
NDCONV_EXE  = navdconv
CFLAGS      = -O3 -std=c99
TARGET_ARCH = -arch i386 -arch x86_64
GITVERSION  = $(shell find . -name ".git" -type d -exec git describe --long --always --dirty=/m --abbrev=1 --tags \;)

override CFLAGS      += -I$(SOURCE_DIR) -DNDCONV_EXE="\"$(NDCONV_EXE)\""
override NDC_SOURCES  =   $(SOURCE_DIR)/tools/navdconv.c
override NDC_OBJECTS  =   $(addsuffix .o,$(basename $(notdir $(NDC_SOURCES))))
override LIB_HEADERS  =   $(wildcard $(SOURCE_DIR)/lib/*.h)
override LIB_SOURCES  =   $(wildcard $(SOURCE_DIR)/lib/*.c)
override LIB_OBJECTS  =   $(addsuffix .o,$(basename $(notdir $(LIB_SOURCES))))
override COM_HEADERS  =   $(wildcard $(SOURCE_DIR)/common/*.h)
override COM_SOURCES  =   $(wildcard $(SOURCE_DIR)/common/*.c)
override COM_OBJECTS  =   $(addsuffix .o,$(basename $(notdir $(COM_SOURCES))))
override CPT_HEADERS  =   $(SOURCE_DIR)/compat/compat.h
override CPT_SOURCES  =   $(SOURCE_DIR)/compat/compat.c
override CPT_OBJECTS  =   $(addsuffix .o,$(basename $(notdir $(CPT_SOURCES))))
override WMM_HEADERS  =   $(wildcard $(SOURCE_DIR)/wmm/*.h)
override WMM_SOURCES  =   $(wildcard $(SOURCE_DIR)/wmm/*.c)
override WMM_OBJECTS  =   $(addsuffix .o,$(basename $(notdir $(WMM_SOURCES))))

all: navdconv

navdconv: ndcobj libobj comobj compat wmmobj
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGET_ARCH) -o $(NDCONV_EXE) $(NDC_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS) $(LDLIBS)

ndcobj: $(NDC_SOURCES)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c $(NDC_SOURCES)

libobj: $(LIB_SOURCES) $(LIB_HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c $(LIB_SOURCES)

comobj: $(COM_SOURCES) $(COM_HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c $(COM_SOURCES)

compat: $(CPT_SOURCES) $(CPT_HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c $(CPT_SOURCES)

wmmobj: $(WMM_SOURCES) $(WMM_HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c $(WMM_SOURCES)

linux:
	$(MAKE) -f Makefile.linux

mingw:
	$(MAKE) -f Makefile.mingw

.PHONY: version
version:
ifneq ($(strip $(GITVERSION)),)
override CFLAGS += -DNDT_VERSION="\"$(GITVERSION)\""
endif

.PHONY: clean
clean:
	@ $(MAKE) -f Makefile.linux clean
	@ $(MAKE) -f Makefile.mingw clean
	$(RM) $(NDCONV_EXE)
	$(RM) $(NDC_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS)
