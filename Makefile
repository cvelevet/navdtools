SOURCE_DIR = src
SOURCE_SDK = XPSDK213
NDCONV_EXE = navdconv
NAVP_XPDLL = navP.xpl
NDTINCLUDE = -I$(SOURCE_DIR)
SDKINCLUDE = -I$(SOURCE_SDK)/CHeaders
SDKLDPATHS = -F$(SOURCE_SDK)/Libraries/Mac
SDKLDLINKS = -framework XPLM -framework XPWidgets
CFLAGS     = -O3 -std=c99
TARGETARCH = -arch i386 -arch x86_64
GITVERSION = $(shell find . -name ".git" -type d -exec git describe --long --always --dirty=/m --abbrev=1 --tags \;)

NDC_DEFINES = -DNDCONV_EXE="\"$(NDCONV_EXE)\""
NDC_SOURCES = $(SOURCE_DIR)/tools/navdconv.c
NDC_OBJECTS = $(addsuffix .o,$(basename $(notdir $(NDC_SOURCES))))
LIB_HEADERS = $(wildcard $(SOURCE_DIR)/lib/*.h)
LIB_SOURCES = $(wildcard $(SOURCE_DIR)/lib/*.c)
LIB_OBJECTS = $(addsuffix .o,$(basename $(notdir $(LIB_SOURCES))))
COM_HEADERS = $(wildcard $(SOURCE_DIR)/common/*.h)
COM_SOURCES = $(wildcard $(SOURCE_DIR)/common/*.c)
COM_OBJECTS = $(addsuffix .o,$(basename $(notdir $(COM_SOURCES))))
CPT_HEADERS = $(SOURCE_DIR)/compat/compat.h
CPT_SOURCES = $(SOURCE_DIR)/compat/compat.c
CPT_OBJECTS = $(addsuffix .o,$(basename $(notdir $(CPT_SOURCES))))
WMM_HEADERS = $(wildcard $(SOURCE_DIR)/wmm/*.h)
WMM_SOURCES = $(wildcard $(SOURCE_DIR)/wmm/*.c)
WMM_OBJECTS = $(addsuffix .o,$(basename $(notdir $(WMM_SOURCES))))
NVP_XPLUGIN = -dynamiclib -fvisibility=hidden
NVP_DEFINES = -DXPLM200 -DXPLM210 -DAPL=1 -DIBM=0 -DLIN=0
NVP_HEADERS = $(wildcard $(SOURCE_DIR)/plugins/navP/*.h)
NVP_SOURCES = $(wildcard $(SOURCE_DIR)/plugins/navP/*.c)
NVP_OBJECTS = $(addsuffix .o,$(basename $(notdir $(NVP_SOURCES))))

all: navdconv navp

navp: nvpobj libobj comobj compat wmmobj
	$(CC) $(NVP_XPLUGIN) $(SDKLDPATHS) $(SDKLDLINKS) $(SDKINCLUDE) $(NDTINCLUDE) $(NVP_DEFINES) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGETARCH) -o $(NAVP_XPDLL) $(NVP_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS) $(LDLIBS)

nvpobj: $(NVP_SOURCES) $(NVP_HEADERS)
	$(CC) $(SDKINCLUDE) $(NDTINCLUDE) $(NVP_DEFINES) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(NVP_SOURCES)

navdconv: ndcobj libobj comobj compat wmmobj
	$(CC) $(NDTINCLUDE) $(NDC_DEFINES) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGETARCH) -o $(NDCONV_EXE) $(NDC_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS) $(LDLIBS)

ndcobj: $(NDC_SOURCES)
	$(CC) $(NDTINCLUDE) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(NDC_SOURCES)

libobj: $(LIB_SOURCES) $(LIB_HEADERS)
	$(CC) $(NDTINCLUDE) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(LIB_SOURCES)

comobj: $(COM_SOURCES) $(COM_HEADERS)
	$(CC) $(NDTINCLUDE) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(COM_SOURCES)

compat: $(CPT_SOURCES) $(CPT_HEADERS)
	$(CC) $(NDTINCLUDE) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(CPT_SOURCES)

wmmobj: $(WMM_SOURCES) $(WMM_HEADERS)
	$(CC) $(NDTINCLUDE) $(CFLAGS) $(CPPFLAGS) $(TARGETARCH) -c $(WMM_SOURCES)

linux:
	$(MAKE) -f Makefile.linux

mingw:
	$(MAKE) -f Makefile.mingw

.PHONY: version
version:
ifneq ($(strip $(GITVERSION)),)
NDC_DEFINES += -DNDT_VERSION="\"$(GITVERSION)\""
endif

.PHONY: clean
clean:
	@ $(MAKE) -f Makefile.linux clean
	@ $(MAKE) -f Makefile.mingw clean
	$(RM) $(NDCONV_EXE)  $(NDC_OBJECTS)
	$(RM) $(NAVP_XPDLL)  $(NVP_OBJECTS)
	$(RM) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS)
