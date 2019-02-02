// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<2, 2, 2> MakeDrivetrainPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 2, 2> MakeDrivetrainControllerCoeffs();
frc::StateSpaceObserverCoeffs<2, 2, 2> MakeDrivetrainObserverCoeffs();
frc::StateSpaceLoop<2, 2, 2> MakeDrivetrainLoop();
