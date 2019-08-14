rm -f -r ../build
mkdir ..\build
cd ..\build
cmake ../ -G"Unix Makefiles" -DPROJ_CONFIG="D:/MCU/TF-M/trusted-firmware-m/ConfigPsaApiTest.cmake" -DTARGET_PLATFORM=M2351 -DCOMPILER=ARMCLANG -DCMAKE_BUILD_TYPE=Debug -DMBEDTLS_DEBUG=ON -DPSA_API_TEST_SECURE_STORAGE=OFF -DPSA_API_TEST_CRYPTO=OFF -DPSA_API_TEST_ATTESTATION=ON -DBL2=False -DBUILD_PLAT_TEST=False
make
