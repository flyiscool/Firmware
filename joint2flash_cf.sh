#!/bin/bash

function dec2hex()
{ 
    printf "%x" $1
}

# get the Firmare absolut path
base_path=$(cd "$(dirname "$0")";pwd)

outputapptxt=$base_path/build/nuttx_coolfly-f1_default/coolfly-f1_app.bin
outputtmp=$base_path/apptmp.bin


skycpu0=$1
# skycpu1=$base_path/boot/...
# skycpu2=$base_path/boot/ar8020_skycpu2.bin
# outcfgbin=$base_path/boot/cfgdata.bin

skycpu2=$base_path/boot/ar8020_skycpu2.bin
outcfgbin=$base_path/boot/cfgdata.bin

echo "cf_log: -----------------------------------------"
echo "cf_log: Making the image package, please wait ..."

#get the length of cpu0cpu1/skycpu2.bin
skycpu0length=`stat --format=%s $skycpu0`
skycpu1length=0
skycpu2length=`stat --format=%s $skycpu2`
cfgbinlenggth=`stat --format=%s $outcfgbin`

echo "skycpu0length = $skycpu0length, skycpu1length = $skycpu1length, skycpu2length = $skycpu2length, cfgbinlenggth = $cfgbinlenggth"

if [ -f $outputtmp ]; then
    rm $outputtmp
fi

if [ -f $outputapptxt ]; then
    rm $outputapptxt
fi

imagedate=`date "+%Y%m%d%H%M%S"`

# 添加 imagedate 信息
echo -n -e \\x0 >> $outputtmp
for i in {0..6}
do
        shiftlen=$[ i * 2 + 1]
        tmp=`expr substr "$imagedate" $shiftlen 2`
        echo -n -e \\x$tmp >> $outputtmp
done


#add "0" to the 8byte for Reserve0
dd if=/dev/zero of=zero.image bs=8 count=1
cat zero.image >> $outputtmp

#add "0" to the 8byte for Reserve1
dd if=/dev/zero of=zero.image bs=8 count=1
cat zero.image >> $outputtmp

#add "0" to the 8byte for Reserve2
dd if=/dev/zero of=zero.image bs=8 count=1
cat zero.image >> $outputtmp

#store cpu0 length 4byte
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu0length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done

#store cpu1 length 4byte
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu1length $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done

#store cpu2 length 4byte
# cpu2 stable size 88k, 64k for itcm2 & 24k for itcm2 external
skycpu2Stablelength=$((88 * 1024))
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $skycpu2Stablelength $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done

#store cfg.bin length 4byte
for i in {0..3}
do
        shiftlen=$[ i * 8 ]
        tmp=`echo $cfgbinlenggth $shiftlen | awk '{print rshift($1,$2)}'`
        tmp=`echo $tmp | awk '{print and($1,255)}'`
        tmphex=$(dec2hex $tmp)
        echo -n -e \\x$tmphex >> $outputtmp
done

#add "0" to the 4K offset
zerolengthboot=$((4096 - 48))
dd if=/dev/zero of=zero.image bs=$zerolengthboot count=1
cat zero.image >> $outputtmp

#add "0" to the 60K offset for the baseband config and foctory setting or user setting
dd if=/dev/zero of=zero.image bs=61440 count=1
cat zero.image >> $outputtmp

#add cpu0 image
cat $skycpu0 >> $outputtmp

#add cpu1 image
# cat $skycpu1 >> $outputtmp

#add cpu2 image
cat $skycpu2 >> $outputtmp

zerolength=$(($skycpu2Stablelength - $skycpu2length))
echo "zerolength = $zerolength"
dd if=/dev/zero of=zero.image bs=$zerolength count=1
cat zero.image >> $outputtmp

#add config bin
cat $outcfgbin >> $outputtmp 

cat $outputtmp >> $outputapptxt
chmod +x $outputapptxt

rm zero.image
rm $outputtmp

echo "cf_log: -----------bin is ok----------------------------"