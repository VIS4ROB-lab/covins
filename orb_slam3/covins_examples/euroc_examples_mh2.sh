#!/bin/bash
pathDatasetEuroc='/home/pschmuck/data/euroc' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH02 with Monocular-Inertial sensor"
./../Examples/Monocular-Inertial/mono_inertial_euroc ./../Vocabulary/ORBvoc.txt ./../Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH_02_easy ./../Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt dataset-MH02_monoi
