#!/usr/bin/env bash

function print_usage() {
	RED='\033[0;31m'
	BLUE='\033[0;34m'
	BOLD='\033[1m'
	NONE='\033[0m'

	echo -e "\n${RED}Usage${NONE}:
	.${BOLD}/manipulator.sh${NONE} [OPTION]"

	echo -e "\n${RED}Options${NONE}:
	${BLUE}build_perception${NONE}: new added, to build object detect function
	${BLUE}build_drivers${NONE}: new added, to build binocular camera driver
	"
}

function build_drivers() {
	cd modules/drivers/binocular_ws/
	catkin_make
}

function build_perception() {
	cd modules/perception/object/binocular_camera/
	catkin_make
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
		*)
			print_usage
			;;
	esac
}

main $@