# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/nemet/esp/esp-idf/components/bootloader/subproject"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix/tmp"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix/src/bootloader-stamp"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix/src"
  "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/NARVAL/esp32_projektek/hd_eeprom/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
