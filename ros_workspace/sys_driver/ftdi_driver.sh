#!/bin/bash

cp ./libftd2xx-x86_64-1.4.27.tgz /tmp
tar -xzvf /tmp/libftd2xx-x86_64-1.4.27.tgz -C /tmp

cp /tmp/release/ftd2xx.h /usr/local/include
cp /tmp/release/WinTypes.h /usr/local/include

cp /tmp/release/build/libftd2xx.so.1.4.27 /usr/local/lib
chmod 0755 /usr/local/lib/libftd2xx.so.1.4.27
ln -sf /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so

ldconfig -v
