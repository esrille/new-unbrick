obj-m := eub5_backlight.o eub5_battery.o eub5_mobo.o eub5_rtc.o eub5_touch.o
eub5_backlight-y := drivers/video/backlight/eub5_backlight.o
eub5_battery-y := drivers/power/supply/eub5_battery.o
eub5_mobo-y := drivers/mfd/eub5_mobo.o
eub5_rtc-y := drivers/rtc/eub5_rtc.o
eub5_touch-y := drivers/input/touchscreen/eub5_touch.o

ccflags-y += -I$(src)/include

dtbo-y += arch/arm/boot/dts/overlays/eub5_mobo.dtbo
targets += dtbs dtbs_install
targets += $(dtbo-y)
always-y := $(dtbo-y)
clean-files := *.dtbo

