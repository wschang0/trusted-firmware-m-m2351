rm -f -r ../build
mkdir ..\build
cd ..\build
cmake ../ -G"Unix Makefiles" -DPROJ_CONFIG="D:/MCU/TF-M/trusted-firmware-m/configs/ConfigRegressionIPCTfmLevel2.cmake" -DTARGET_PLATFORM=M2351 -DCOMPILER=ARMCLANG -DCMAKE_BUILD_TYPE=DEBUG -DITS_RAM_FS=OFF -DBL2=False -DBUILD_PLAT_TEST=False -DBUILD_DWARF_VERSION=2 -DMBEDTLS_DEBUG=ON
make

