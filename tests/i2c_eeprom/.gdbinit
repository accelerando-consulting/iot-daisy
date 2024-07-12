file .pio/build/evb/firmware.elf
target extended-remote localhost:3333
load
monitor reset
break main
continue
