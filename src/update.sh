# copy files into car directory in order to get everything working
echo "Copying manage_modified.py"
cp manage_modified.py ~/projects/d3/
echo "Copying parts"
cd parts && cp lidar_distance_calculator.py lidar_filters.py lidar_processor.py cruise_controller.py ~/projects/donkeycar/donkeycar/parts

echo "Please go to ~/projects/d3 and run command: python manage_modified.py drive"