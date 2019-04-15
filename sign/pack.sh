#!/bin/bash


cp atsha204_raw.ko atsha204.ko
./sign-file sha512 signing_key.priv signing_key.x509 atsha204.ko

if [ $? ] ;then
    echo "pack success"
    hexdump -C atsha204.ko | tail
fi


cp atsha204.ko atsha204.ko


