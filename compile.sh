#!/bin/bash

# The source file is provided as $1
# Convert this to a target

usage() {
		echo "compile.sh [-r] <sourcefile>"
}

while getopts "r" arg; do
		case $arg in
				r)
						rebuild=1
						;;
		esac
done

shift $((OPTIND-1))
srcfile=$1

# We have to mimic the src->build directory placement strategy of the Makefile a bit here:
if [[ $srcfile =~ ^grbl ]]; then
		srcfile=${srcfile#*grbl/}
fi
if [[ $srcfile =~ ^stm32f407 ]]; then
		srcfile=${srcfile#*stm32f407/}
fi

# Now convert the source fn to an object fn (.[ch]=.o)
if [[ $srcfile =~ .*\.c ]]; then
		tgtfile=build/${srcfile%.c}.o
elif [[ $srcfile =~ .*\.h ]]; then
		tgtfile=build/${srcfile%.h}.o
else
		echo "$srcfile is not a valid source file!"
		exit
fi

if [[ -n $rebuild ]]; then
		echo "Rebuilding on request..."
		echo rm $tgtfile
		rm $tgtfile
fi

# Make the target
make -j -f stm32.Makefile $tgtfile

