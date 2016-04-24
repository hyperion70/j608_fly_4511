Kernel 3.4.111 for Fly IQ4511 or XOLO x8 (mt6592)
=================================================

Build Command ---> kernel ---> boot.img

cd j608_fly_4511
./mk -o=TARGET_BUILD_VARIANT=user j608_fly n k
./pack_bootimage.sh


lk.bin:

./mk -o=TARGET_BUILD_VARIANT=user j608_fly n lk
