#!/usr/bin/env bash

function print_usage() {
	RED='\033[0;31m'
	BLUE='\033[0;34m'
	BOLD='\033[1m'
	NONE='\033[0m'

	echo -e "\n${RED}Usage${NONE}:
	.${BOLD}/manipulator.sh${NONE} [OPTION]"

	echo -e "\n${RED}Options${NONE}:
	${BLUE}build{NONE}: build all
	${BLUE}build_perception${NONE}: new added, to build object detect function
	${BLUE}build_drivers${NONE}: new added, to build binocular camera driver
	${BLUE}build_calibration${NONE}: new added, to build binocular camera calibration
	"
}

function build_drivers() {
	cd modules/drivers/binocular_ws/
	catkin_make
	cd -
}

function build_perception() {
	cd modules/perception/object/binocular_camera/
	catkin_make
	cd -
}

function build_calibration() {
	if [ ! -d "modules/calibration/stereo_calibration/build" ]; then
 		mkdir -p modules/calibration/stereo_calibration/build
 	fi
 	cd modules/calibration/stereo_calibration/build
 	cmake ..
 	make -j
 	cd -
}

function main() {
	local cmd=$1
  	shift
	case $cmd in
		build_drivers)
			build_drivers
			;;
		build_perception)
			build_perception
			;;
		build_calibration)
			build_calibration
			;;
		build)
			build_drivers
			build_perception
			build_calibration
			;;
		*)
			print_usage
			;;
	esac
}

main $@