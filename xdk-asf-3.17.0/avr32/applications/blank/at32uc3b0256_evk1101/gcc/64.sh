dfu-programmer at32uc3b064 erase --debug 100
echo "###################################"
dfu-programmer at32uc3b064 flash evk1101-demo.hex --suppress-bootloader-mem --debug 100
echo "###################################"
dfu-programmer at32uc3b064 start
