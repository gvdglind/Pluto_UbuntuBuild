#!/bin/sh
# script to set io directions and so on to accecc some gpio's like buzzer, tamper...
#
cd /sys/class/gpio
chmod +666 export
#
# buzzer 
echo 2 > export
echo out > gpio2/direction
# tamper
echo 41 > export
echo in > gpio41/direction
# boolsel button
echo 71 > export
echo in > gpio71/direction
# board rev ...
echo 89 > export
echo in > gpio89/direction
#
echo 87 > export
echo in > gpio87/direction
#
echo 88 > export
echo in > gpio88/direction
#
echo 8 > export
echo in > gpio8/direction
#
echo 1 > export
echo in > gpio1/direction
#
echo 43 > export
echo in > gpio43/direction
#
echo 1 > /proc/gpio/out/buzzer
sleep 0.5
echo 0 > /proc/gpio/out/buzzer
#
echo 300000 > /proc/gpio/out/statusled
sleep 0.5
#
echo 003000 > /proc/gpio/out/statusled
sleep 0.5
#
echo 000030 > /proc/gpio/out/statusled
sleep 0.5
#
echo 000000 > /proc/gpio/out/statusled
#
exit 0
