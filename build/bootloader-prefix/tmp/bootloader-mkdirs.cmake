# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/dprograms/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/tmp"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/src/bootloader-stamp"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/src"
  "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/axio/projects/byok/tusb_integration_trial2/byok_repo/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
