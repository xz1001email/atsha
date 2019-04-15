#!/bin/bash

ret=0
if [ -f atsha204.ko ] ;then
    ./sign/sign-file sha512 sign/signing_key.priv sign/signing_key.x509 atsha204.ko
    ret=$?
else
    echo "atsha204.ko not exsist!"
    exit
fi

if [ $ret ] ;then
    echo "pack success"
    hexdump -C atsha204.ko | tail
fi

