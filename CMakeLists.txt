
cmake_minimum_required(VERSION 3.2)

SET(CMAKE_SYSTEM_NAME Linux)

SET (CMAKE_C_COMPILER "/opt/hisi-linux/x86-arm/arm-himix200-linux/bin/arm-himix200-linux-gcc")
SET(CMAKE_CXX_COMPILER "/opt/hisi-linux/x86-arm/arm-himix200-linux/bin/arm-himix200-linux-g++")
#SET (CMAKE_C_COMPILER "/opt/hisi-linux/x86-arm/aarch64-himix100-linux/bin/aarch64-himix100-linux-gcc")
#SET(CMAKE_CXX_COMPILER "/opt/hisi-linux/x86-arm/aarch64-himix100-linux/bin/aarch64-himix100-linux-g++")

#SET (CMAKE_C_COMPILER "gcc")
#SET(CMAKE_CXX_COMPILER "g++")

SET(CMAKE_BUILE_TYPE RELEASE)
#SET(CMAKE_BUILE_TYPE DEBUG)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 




PROJECT(ocr_detect_test)


#INCLUDE_DIRECTORIES(/media/shuang/0D2C17A90D2C17A9/1-work-space/2019/Hi3559a_C020/Hi3559AV100R001C02SPC020/Hi3559AV100R001C02SPC020/01.software/board/Hi3559AV100_SDK_V2.0.1.0/package/mpp/out/linux/multi-core/include)
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(./ocr_detect_include/HI_LIBS/include)
#INCLUDE_DIRECTORIES(/media/t/disk1/jie_shou_ren_ming/xujunke/Embedded_system/Hi3519A/Hi3519A V100R001C02SPC010/01.software/board/Hi3519AV100_SDK_V2.0.1.0/smp/a53_linux/mpp/include)
#***INCLUDE_DIRECTORIES(/media/t/disk1/jie_shou_ren_ming/xujingjing/vehicle_detect-master/code/VehicleDetection/NNIE201)
#***INCLUDE_DIRECTORIES(/media/t/disk1/jie_shou_ren_ming/xujingjing/vehicle_detect-master/code/VehicleDetection/NNIE201/inc)
INCLUDE_DIRECTORIES(./ocr_detect_include/NNIE_LIBS_SO/OcrExport)
#INCLUDE_DIRECTORIES(/media/t/disk1/jie_shou_ren_ming/xujingjing/vehicle_detect-master/code/VehicleDetection)
#INCLUDE_DIRECTORIES(/media/shuang/0D2C17A90D2C17A9/1-work-space/2019/Hi3559a_C020/Hi3559AV100R001C02SPC020/Hi3559AV100R001C02SPC020/01.software/board/Hi3559AV100_SDK_V2.0.1.0/package/mpp/sample/common)
#INCLUDE_DIRECTORIES(/media/shuang/0D2C17A90D2C17A9/1-work-space/2019/Hi3559a_C020/Hi3559AV100R001C02SPC020/Hi3559AV100R001C02SPC020/01.software/board/Hi3559AV100_SDK_V2.0.1.0/package/mpp/sample/svp/multi-core/nnie/sample_nnie_software)
#INCLUDE_DIRECTORIES(/media/shuang/0D2C17A90D2C17A9/1-work-space/2019/Hi3559a_C020/Hi3559AV100R001C02SPC020/Hi3559AV100R001C02SPC020/01.software/board/Hi3559AV100_SDK_V2.0.1.0/package/mpp/sample/svp/multi-core/common)

INCLUDE_DIRECTORIES(./ocr_detect_include/OPENCV_LIBS_SO/include)
INCLUDE_DIRECTORIES(./ocr_detect_include/OPENCV_LIBS_SO/include/opencv)
INCLUDE_DIRECTORIES(./ocr_detect_include/OPENCV_LIBS_SO/include/opencv2)
INCLUDE_DIRECTORIES(./ocr_detect_include/OPENCVF_LIBS_SO/include/)
INCLUDE_DIRECTORIES(./ocr_detect_include/OPENCVX_LIBS_SO/include/)

FILE(GLOB_RECURSE CURRENT_SOURCES_2 common/*.c)
#FIND_LIBRARY(RUNTIME_LIB rt  /mnt/hgfs/Linux_share/arm-opencv/hi3559a_opencv249/lib NO_DEFAULT_PATH)
#FILE(GLOB_RECURSE CURRENT_SOURCES_3 software/*.c)

set(SRC)
list(APPEND SRC main.cpp)


#set(SRC_SO)
#list(APPEND SRC_SO landmark/facelandmark.cpp face/facebox.cpp facefeature/facefeature.cpp)
#FILE(GLOB_RECURSE NNIE_LIBS2_SO lib_samplennie/*.so)
#FILE(GLOB_RECURSE HI_LIBS /media/t/disk1/jie_shou_ren_ming/xujingjing/hi_lib/*.so)
FILE(GLOB_RECURSE NNIE_LIBS_SO ./ocr_detect_lib/NNIE_LIBS_SO/lib/*.so)
FILE(GLOB_RECURSE HI_LIBS ./ocr_detect_lib/HI_LIBS/lib/*.so)
#FILE(GLOB_RECURSE HI_LIBS /media/t/disk1/jie_shou_ren_ming/chenlijun/lib/*)
#FILE(GLOB_RECURSE CURRENT_LIBS_SO thrift/lib/*.so)
#FILE(GLOB_RECURSE ADD_LIBS_SO thrift/Add/*.so)
FILE(GLOB_RECURSE OPENCV_LIBS_SO ./ocr_detect_lib/OPENCV_LIBS_SO/lib/*.so)
FILE(GLOB_RECURSE OPENCVF_LIBS_SO ./ocr_detect_lib/OPENCVF_LIBS_SO/lib/*.so)
FILE(GLOB_RECURSE OPENCVX_LIBS_SO ./ocr_detect_lib/OPENCVX_LIBS_SO/lib/*.so)
#FILE(GLOB_RECURSE SQLITE_LIBS_SO sqlite3/lib/*.so*)

ADD_DEFINITIONS(
-fno-aggressive-loop-optimizations -ffunction-sections -fdata-sections -O2 -DISP_V2
)

ADD_DEFINITIONS( -D_REENTRANT -fno-aggressive-loop-optimizations )

ADD_DEFINITIONS(
-fstack-protector -Wl,-z,relro -Wl,-z,noexecstack -O3 -fPIC -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE	
)

#add_library(facedetect SHARED facedetection.h facedetection.cpp ${CURRENT_SOURCES_2} ${SRC_SO})   #STATIC  or SHARED
#target_link_libraries(facedetect -lpthread -lrt -lm -ldl -lstdc++ ${CURRENT_LIBS_SO} ${OPENCV_LIBS_SO})

#SRC赋值，追加
#set(SRC)
#list(APPEND SRC createxXML_1.cpp tinyxml2.cpp) 
#list(APPEND SRC c.cpp d.cpp) 
#message("SRC = "${SRC}) 


add_executable(ocr_detect_test main.cpp ${CURRENT_SOURCES_2} ${CURRENT_SOURCES_3} )


target_link_libraries(ocr_detect_test -lpthread -lrt -lm -ldl -lstdc++ ${NNIE_LIBS_SO} ${HI_LIBS} ${OPENCV_LIBS_SO} ${OPENCVX_LIBS_SO} ${OPENCVF_LIBS_SO} )
#-lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lippicv -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core
#-lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_video -lopencv_imgproc -lopencv_photo -lopencv_ml -lopencv_flann -lopencv_core

