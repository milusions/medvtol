colcon build
gnome-terminal --tab -- bash -c "source install/setup.sh && ros2 launch medvtol_companion companion.launch.py; exec bash"
./update_px4_model.sh
cd ~/PX4-Autopilot
rm -rf build/
gnome-terminal --tab -- bash -c "micro-xrce-dds-agent udp4 -p 8888; exec bash"
sleep 10
google-chrome "http://localhost:8890/medvtol_control_panel.html" &
export GZ_SIM_RESOURCE_PATH=~/medvtol_ws/install/medvtol_description/share:$GZ_SIM_RESOURCE_PATH
PX4_GZ_WORLD=medical PX4_GZ_MODEL_POSE="0,5,1,0,0,0" make px4_sitl gz_medvtol
cd ~/medvtol_ws/







