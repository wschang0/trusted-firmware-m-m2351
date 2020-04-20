rm -f -r ../build
mkdir ..\build
cd ..\build
cmake  ../ -G"Unix Makefiles" -DPROJ_CONFIG="D:/MCU/TF-M/trusted-firmware-m/ConfigRegression.cmake" -DTARGET_PLATFORM=M2351 -DCOMPILER=ARMCLANG -DCMAKE_BUILD_TYPE=Debug -DMBEDTLS_DEBUG=OFF -DBL2=True -DBUILD_PLAT_TEST=False
make

