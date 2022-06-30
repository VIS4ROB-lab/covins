#!/bin/bash
pathDatasetEuroc='/home/manthan/Downloads/covins_vins/Euroc' #Example, it is necesary to change it by the dataset path
#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_01_easy ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

sleep 5
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate1.txt

echo "Launching MH02 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_02_easy ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_monoi

sleep 5
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate2.txt

echo "Launching MH03 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_03_medium ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt dataset-MH03_monoi

sleep 30
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate3.txt

echo "Launching MH04 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_04_difficult ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH04.txt dataset-MH04_monoi

sleep 15
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate4.txt

echo "Launching MH05 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_05_difficult ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH05.txt dataset-MH05_monoi

sleep 15
cp ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate.txt ~/ws/covins_ws/src/covins/covins_backend/output/stamped_traj_estimate5.txt

echo "Done..."

sleep 30

#rosservice call /covins_savemap 0


# echo "Waiting for Global BA to finish"
# rosservice call /covins_gba 0 1

