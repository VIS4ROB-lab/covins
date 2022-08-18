#!/bin/bash
pathDatasetEuroc='/home/manthan/Downloads/covins_vins/Euroc/MH' #Example, it is necesary to change it by the dataset path
#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH04 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_04_difficult ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH04.txt dataset-MH04_monoi

sleep 15
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate4.txt

echo "Launching MH05 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_05_difficult ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH05.txt dataset-MH05_monoi

sleep 15
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate5.txt

echo "Done..."

sleep 10

# rosservice call /covins_savemap 0


# echo "Waiting for Global BA to finish"
# rosservice call /covins_gba 0 1

