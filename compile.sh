#!/bin/bash

# The source file is provided as $1
# Convert this to a target

srcfile=$1

# We have to mimic the placement strategy of the Makefile a bit here:
if [[ $srcfile =~ ^grbl ]]; then
		srcfile=${srcfile#*grbl/}
fi
if [[ $srcfile =~ ^stm32f407 ]]; then
		srcfile=${srcfile#*stm32f407/}
fi

if [[ $srcfile =~ .*\.c ]]; then
		tgtfile=build/${srcfile%.c}.o
elif [[ $srcfile =~ .*\.h ]]; then
		tgtfile=build/${srcfile%.h}.o
else
		echo "$srcfile is not a valid source file!"
		exit
fi

make -j -f stm32.Makefile $tgtfile

