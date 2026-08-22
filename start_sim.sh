./update_px4_model.sh
cd ~/PX4-Autopilot
rm -rf build/
export GZ_SIM_RESOURCE_PATH=~/medvtol_ws/install/medvtol_description/share:$GZ_SIM_RESOURCE_PATH
PX4_GZ_WORLD=medical PX4_GZ_MODEL_POSE="0,3,2,0,0,0" make px4_sitl gz_medvtol
cd ~/medvtol_ws/