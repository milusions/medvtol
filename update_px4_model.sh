cp px4_files/medvtol/model.config ~/PX4-Autopilot/Tools/simulation/gz/models/medvtol
cp px4_files/medvtol/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/medvtol
cp px4_files/4200_gz_medvtol ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp -r ~/PX4-Autopilot/Tools/simulation/gz/models/*/ ~/PX4-Autopilot/Tools/simulation/gz/models_backup/
cp -r px4_files/models/*/ ~/PX4-Autopilot/Tools/simulation/gz/models/
cp px4_files/medical.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds