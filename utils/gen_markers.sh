mkdir -p  ../testsdata/markers
ln -s -f ../testsdata/markers/  markerdata
./aruco_create_marker 0 ./markerdata/aruco_0000.jpg 640
./aruco_create_marker 63 ./markerdata/aruco_0063.jpg 640
./aruco_create_marker 127 ./markerdata/aruco_0127.jpg 640
./aruco_create_marker 254 ./markerdata/aruco_0254.jpg 640
./aruco_create_marker 511 ./markerdata/aruco_0511.jpg 640
./aruco_create_marker 1023 ./markerdata/aruco_1023.jpg 640

./aruco_create_marker 383 ./markerdata/aruco_0383.jpg 640
./aruco_create_marker 766 ./markerdata/aruco_0766.jpg 640

