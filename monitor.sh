#!/usr/bin/env bash
port="$(ls /dev/ttyUSB* 2>/dev/null)"

[[ -e $port ]] || {
	echo "Error: No valid port found" >&2
	exit 1
}
[[ -d $IDF_PATH ]] || {
	echo "Error: IDF_PATH is not set correctly!" >&2
	exit 1
}
echo "Using port $port ..."

(
	. $IDF_PATH/export.sh
	stty -F $port raw 115200
	cat $port
)
