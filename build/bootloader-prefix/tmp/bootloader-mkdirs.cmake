# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/dmytr/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/tmp"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/src"
  "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/dmytr/Desktop/ESP36C6_Zigbee_PZEM-004t/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
