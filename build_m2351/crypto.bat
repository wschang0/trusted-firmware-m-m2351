rm -f -r ../build
mkdir ..\build
cd ..\build
cmake ../ -G"Unix Makefiles" -DPROJ_CONFIG="D:/MCU/TF-M/trusted-firmware-m/ConfigPsaApiTestIPCTfmLevel2.cmake" -DTARGET_PLATFORM=M2351 -DCOMPILER=ARMCLANG -DCMAKE_BUILD_TYPE=Debug -DMBEDTLS_DEBUG=ON -DPSA_API_TEST_SECURE_STORAGE=OFF -DPSA_API_TEST_CRYPTO=ON -DBL2=False -DBUILD_PLAT_TEST=False
make
