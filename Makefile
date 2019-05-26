SOURCE_DIR = src
SOURCE_SDK = XPSDK213
NDCONV_EXE = navdconv
NAVP_XPDLL = navP.xpl
ND_INCLUDE = -I$(SOURCE_DIR)
XP_INCLUDE = -I$(SOURCE_SDK)/CHeaders
XP_LD_LIBS = -F$(SOURCE_SDK)/Libraries/Mac -framework XPLM -framework XPWidgets -framework OpenGL
XPCPPFLAGS = -DXPLM200 -DXPLM210 -DAPL=1 -DIBM=0 -DLIN=0
CFLAGS     = -O3 -std=c99 -mmacosx-version-min=10.9
TARGETARCH = -arch x86_64
CC         = clang
CPPFLAGS   =

LIBACU_DIR = libacfutils-redist
LIBACU_LIB = -L$(LIBACU_DIR)/mac64/lib -lacfutils -lcairo -lfreetype -lpixman-1 -lz
LIBACU_INC = -I$(LIBACU_DIR)/include -I$(LIBACU_DIR)/mac64/include -I$(LIBACU_DIR)/mac64/include/freetype2

NDCCPPFLAGS =
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
NVPCPPFLAGS =
NVP_LDFLAGS = -dynamiclib -fvisibility=hidden
NVP_HEADERS = $(wildcard $(SOURCE_DIR)/plugins/navP/*.h)
NVP_SOURCES = $(wildcard $(SOURCE_DIR)/plugins/navP/*.c)
NVP_OBJECTS = $(addsuffix .o,$(basename $(notdir $(NVP_SOURCES))))
GIT_VERSION = $(shell find . -name ".git" -type d -exec git describe --long --always --dirty=/m --abbrev=1 --tags \;)
NDC_VERSION = -DNDCONV_EXE="\"$(NDCONV_EXE)\""
NDT_VERSION =

all: navdconv navp

navp: nvpobj libobj comobj compat wmmobj
	$(CC) $(ND_INCLUDE) $(XP_INCLUDE) $(LIBACU_INC) $(XPCPPFLAGS) $(CPPFLAGS) $(NVPCPPFLAGS) $(NDT_VERSION) $(CFLAGS) $(NVP_LDFLAGS) $(LDFLAGS) $(TARGETARCH) -o $(NAVP_XPDLL) $(NVP_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS) $(XP_LD_LIBS) $(LIBACU_LIB) $(LDLIBS)

nvpobj: $(NVP_SOURCES) $(NVP_HEADERS)
	$(CC) $(ND_INCLUDE) $(XP_INCLUDE) $(LIBACU_INC) $(XPCPPFLAGS) $(CPPFLAGS) $(NVPCPPFLAGS) $(NDT_VERSION) $(CFLAGS) $(TARGETARCH) -c $(NVP_SOURCES)

navdconv: ndcobj libobj comobj compat wmmobj
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(NDC_VERSION) $(CFLAGS) $(LDFLAGS) $(TARGETARCH) -o $(NDCONV_EXE) $(NDC_OBJECTS) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS) $(LDLIBS)

ndcobj: $(NDC_SOURCES)
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(CFLAGS) $(TARGETARCH) -c $(NDC_SOURCES)

libobj: $(LIB_SOURCES) $(LIB_HEADERS)
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(CFLAGS) $(TARGETARCH) -c $(LIB_SOURCES)

comobj: $(COM_SOURCES) $(COM_HEADERS)
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(CFLAGS) $(TARGETARCH) -c $(COM_SOURCES)

compat: $(CPT_SOURCES) $(CPT_HEADERS)
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(CFLAGS) $(TARGETARCH) -c $(CPT_SOURCES)

wmmobj: $(WMM_SOURCES) $(WMM_HEADERS)
	$(CC) $(ND_INCLUDE) $(NDCCPPFLAGS) $(CPPFLAGS) $(CFLAGS) $(TARGETARCH) -c $(WMM_SOURCES)

linux:
	$(MAKE) -f Makefile.linux

mingw:
	$(MAKE) -f Makefile.mingw navdconv yfmsonly

mingw2:
	$(MAKE) -f Makefile.mingw navdconv yfmsnavp

.PHONY: version
version:
ifneq ($(strip $(GIT_VERSION)),)
NDC_VERSION += -DNDT_VERSION="\"$(GIT_VERSION)\""
NDT_VERSION += -DNDT_VERSION="\"$(GIT_VERSION)\""
endif

.PHONY: clean
clean:
	@ $(MAKE) -f Makefile.linux clean
	@ $(MAKE) -f Makefile.mingw clean
	$(RM) $(NDCONV_EXE)  $(NDC_OBJECTS)
	$(RM) $(NAVP_XPDLL)  $(NVP_OBJECTS)
	$(RM) $(LIB_OBJECTS) $(COM_OBJECTS) $(CPT_OBJECTS) $(WMM_OBJECTS)
