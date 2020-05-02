This is a fork from [Keystone's bootloader](https://github.com/keystone-enclave/freedom-u540-c000-bootloader) and modified to use with developed [TEE-Hardware](https://github.com/ckdur/tee-hardware).

To use this repo separately with hardware:
- For original SiFive build:
```
# make sure that the BOARD & TEEHW vars are not set
$ unset BOARD
$ unset TEEHW

# then, just normal 'make'
$ make
```
- For freedom (U540) platform: (do this when inside the freedom repo)
```
# this will sets you up with the environment vars
# you might want to modify the params in the file yourself to fit in with your machine settings
$ . setenv-freedom.sh

# then, just 'make'
$ make
```
- For TEE-hardware platform:
```
# this will sets you up with the environment vars
# again, you might want to modify the params in the file yourself to fit in with your machine settings
$ . setenv-teehw.sh

# then, just 'make'
$ make
```

