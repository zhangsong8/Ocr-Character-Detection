################OPTION###################
TOOLCHAIN_DIR = 
#COMPILE_PREFIX = arm-arago-linux-gnueabi-

include Rule.mak

CCOMPILE = $(TOOLCHAIN_DIR)$(COMPILE_PREFIX)gcc
CPPCOMPILE = $(TOOLCHAIN_DIR)$(COMPILE_PREFIX)g++

#LINK = $(CCOMPILE)
LINK = $(CPPCOMPILE)

BINFILE = libOcrDetect.so

COMPILEOPTION = -c
INCLUDEDIR = -I. 
INCLUDEDIR += -I./NNIE
INCLUDEDIR += -I./NNIE201/
INCLUDEDIR += -I./NNIE201/inc

OUTDIR = ./../../ocr_detect_lib/NNIE_LIBS_SO/lib/
#OUTDIR1 = ./lib64/
#LIBDIRS = -L$(OUTDIR) 

OBJS  = ./OcrDetectExport.o

#COMPILEOPTION += -O3 -g
COMPILEOPTION += -O3 -rdynamic -fstack-protector-all -fPIC 
LINKOPTION = -shared -o $(OUTDIR)$(BINFILE) -Wl,-R,$(OUTDIR)

#opencv
#INCLUDEDIR += -I./../../ocr_detect_include/OPENCVX_LIBS_SO/include
#INCLUDEDIR += -I./../../ocr_detect_include/OPENCVF_LIBS_SO/include
#INCLUDEDIR += -I./../../ocr_detect_include/OPENCV_LIBS_SO/include
#INCLUDEDIR += -I./../../ocr_detect_include/OPENCV_LIBS_SO/include/opencv
#INCLUDEDIR += -I./../../ocr_detect_include/OPENCV_LIBS_SO/include/opencv2


#SHAREDLIB += -lcaffe 
# SHAREDLIB += -lSampleNNIE 

# SHAREDLIB += ../NNIE201/hi_lib/lib_hiae.a
# SHAREDLIB += ../NNIE201/hi_lib/libisp.a
# SHAREDLIB += ../NNIE201/hi_lib/lib_hidehaze.a
# SHAREDLIB += ../NNIE201/hi_lib/lib_hidrc.a
# SHAREDLIB += ../NNIE201/hi_lib/lib_hildci.a
# SHAREDLIB += ../NNIE201/hi_lib/libsns_imx477.a
# SHAREDLIB += ../NNIE201/hi_lib/libsns_imx290.a
# SHAREDLIB += ../NNIE201/hi_lib/libsns_imx334.a
# SHAREDLIB += ../NNIE201/hi_lib/libsns_imx334_slave.a
# SHAREDLIB += ../NNIE201/hi_lib/libsns_imx277_slvs.a
# SHAREDLIB += ../NNIE201/hi_lib/lib_hiawb.a

# SHAREDLIB += ../NNIE201/hi_lib/libmpi.a
# SHAREDLIB += ../NNIE201/hi_lib/libhdmi.a
# SHAREDLIB += ../NNIE201/hi_lib/libdpu_rect.a
# SHAREDLIB += ../NNIE201/hi_lib/libdpu_match.a
# SHAREDLIB += ../NNIE201/hi_lib/libhiavslut.a
# SHAREDLIB += ../NNIE201/hi_lib/libdnvqe.a
# SHAREDLIB += ../NNIE201/hi_lib/libupvqe.a
# SHAREDLIB += ../NNIE201/hi_lib/libsecurec.a
# SHAREDLIB += ../NNIE201/hi_lib/libVoiceEngine.a
# SHAREDLIB += ../NNIE201/hi_lib/libive.a
# SHAREDLIB += ../NNIE201/hi_lib/libmd.a
# SHAREDLIB += ../NNIE201/hi_lib/libdsp.a
# SHAREDLIB += ../NNIE201/hi_lib/libnnie.a

#SHAREDLIB += -lm -ldl -lpthread -lstdc++
#SHAREDLIB += -lm -ldl -lpthread 


CPPFLAGS = --static 

OUTPUT = $(OUTDIR)$(BINFILE)


$(OUTPUT): $(OBJS) 
	$(LINK) $(LINKOPTION) $(LIBDIRS) $(OBJS) $(SHAREDLIB) $(STATICLIB)

clean: 
	rm -f $(OBJS)
	rm -f $(OUTPUT)
all: clean $(OUTPUT)
#	gcc -o $(OUTDIR)test-shmque-srv -Wl,-R,$(OUTDIR) test-shmque-srv.c -L$(OUTDIR) -lShmQue $(INCLUDEDIR) 
#	gcc -o $(OUTDIR)test-shmque-cli -Wl,-R,$(OUTDIR) test-shmque-cli.c -L$(OUTDIR) -lShmQue $(INCLUDEDIR)
.PRECIOUS:%.cpp %.c %.C
.SUFFIXES:
.SUFFIXES:  .c .o .cpp .ecpp .pc .ec .C .cc .cxx

.cpp.o:
	$(CPPCOMPILE) -c -o $*.o $(COMPILEOPTION) $(CPPFLAGS) $(INCLUDEDIR)  $*.cpp
	
.cc.o:
	$(CCOMPILE) -c -o $*.o $(COMPILEOPTION) $(INCLUDEDIR)  $*.cx

.cxx.o:
	$(CPPCOMPILE) -c -o $*.o $(COMPILEOPTION) $(INCLUDEDIR)  $*.cxx

.c.o:
	$(CCOMPILE) -c -o $*.o $(COMPILEOPTION) $(INCLUDEDIR) $*.c

.C.o:
	$(CPPCOMPILE) -c -o $*.o $(COMPILEOPTION) $(INCLUDEDIR) $*.C	

.ecpp.C:
	$(ESQL) -e $(ESQL_OPTION) $(INCLUDEDIR) $*.ecpp 
	
.ec.c:
	$(ESQL) -e $(ESQL_OPTION) $(INCLUDEDIR) $*.ec
	
.pc.cpp:
	$(PROC)  CPP_SUFFIX=cpp $(PROC_OPTION)  $*.pc

