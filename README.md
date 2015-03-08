Fly IQ4511 or XOLO x8
===============

Build Command

kernel:

cd j608_fly_4511
./mk -o=TARGET_BUILD_VARIANT=user j608_fly n k

Then, to create the boot.img:
./pack_bootimage.sh


lk.bin:

./mk -o=TARGET_BUILD_VARIANT=user j608_fly n lk
