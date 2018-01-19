#!/bin/bash
clear
passphrase="123"

read -p "You need not run this script if already connect to WIFI, YES for continue [YES/NO]? " answer
case $answer in
YES)
    echo "Install WIFI driver start.";;
*)
    echo "Exit."
    exit 0;;
esac

echo ${passphrase} | sudo -S su >/dev/null 2>&1
sudo apt-get update

echo ${passphrase} | sudo -S su >/dev/null 2>&1
sudo apt-get install firmware-b43-installer

echo ${passphrase} | sudo -S su >/dev/null 2>&1
sudo apt-get install bcmwl-kernel-source

echo ${passphrase} | sudo -S su >/dev/null 2>&1
sudo apt-get install broadcom-sta-dkms

exit $?
