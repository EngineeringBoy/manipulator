#!/usr/bin/env bash
./modules/calibration/stereo_calibration/build/calibrate -w 11 -h 8 -n 45 -s 0.03 -d "data/3/" -i "left" -o "data/tmp/cam_left.yml" -e "png"
./modules/calibration/stereo_calibration/build/calibrate -w 11 -h 8 -n 45 -s 0.03 -d "data/3/" -i "right" -o "data/tmp/cam_right.yml" -e "png"
./modules/calibration/stereo_calibration/build/calibrate_stereo -n 45 -u data/tmp/cam_left.yml -v data/tmp/cam_right.yml -L data/3/ -R data/3/ -l left -r right -o modules/drivers/binocular_ws/src/mini_binocular/calib_file/cam_stereo_190319.yml