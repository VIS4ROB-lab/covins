#!/bin/bash
pathDatasetEuroc='/home/pschmuck/data/euroc' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH03 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_03_medium ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt dataset-MH03_monoi
