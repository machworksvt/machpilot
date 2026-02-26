
dtc -@ -I dts -O dtb -o /boot/dtb/overlays/i2c_clock_stretch_en.dtbo i2c_clock_stretch_en.dts

FILE="/boot/extlinux/extlinux.conf"
TEXT="FDTOVERLAYS /boot/dtb/overlays/i2c_clock_stretch_en.dtbo"
MATCH="MENU LABEL real-time kernel\n\tLINUX Image.real-time\n\tINITRD /boot/initrd"

if grep -q $TEXT $FILE; then
    echo "Overlay Already Added"

else
    echo "Adding Overlay to Boot Config"
    sed -i -z "s|$MATCH|$MATCH\n\t$TEXT|" $FILE
fi

echo "Rebooting"
echo "......"

reboot