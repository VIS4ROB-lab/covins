#!/bin/bash
pathDatasetEuroc='/home/manthan/Downloads/covins_vins/Euroc/MH' #Example, it is necesary to change it by the dataset path
#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_01_easy ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

sleep 10

echo "Launching MH02 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_02_easy ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_monoi

sleep 20

echo "Launching MH03 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_03_medium ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt dataset-MH03_monoi

sleep 30

echo "Done"
