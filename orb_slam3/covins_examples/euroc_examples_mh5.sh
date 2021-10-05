#!/bin/bash
pathDatasetEuroc='/home/pschmuck/data/euroc' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH05 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_05_difficult ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH05.txt dataset-MH05_monoi
