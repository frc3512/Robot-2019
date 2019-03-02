// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/StateSpaceController.h>
#include <frc/controller/StateSpaceObserver.h>
#include <frc/controller/StateSpacePlant.h>

#include "Constants.hpp"
#include "control/ElevatorClimbCoeffs.hpp"
#include "control/ElevatorCoeffs.hpp"
#include "control/TrapezoidalMotionProfile.hpp"
#include "logging/CsvLogger.hpp"

namespace frc3512 {

class ElevatorController {
public:
    // State tolerances in meters and meters/sec respectively.
    static constexpr double kPositionTolerance = 0.05;
    static constexpr double kVelocityTolerance = 2.0;

    ElevatorController();

    ElevatorController(const ElevatorController&) = delete;
    ElevatorController& operator=(const ElevatorController&) = delete;

    void Enable();
    void Disable();

    void SetScoringIndex();
    void SetClimbingIndex();

    void SetGoal(double goal);

    /**
     * Sets the references.
     *
     * @param position Position of the carriage in meters.
     * @param velocity Velocity of the carriage in meters per second.
     */
    void SetReferences(units::meter_t position,
                       units::meters_per_second_t velocity);

    bool AtReferences() const;

    bool AtGoal() const;

    /**
     * Sets the current encoder measurement.
     *
     * @param measuredPosition Position of the carriage in meters.
     */
    void SetMeasuredPosition(double measuredPosition);

    /**
     * Returns the control loop calculated voltage.
     */
    double ControllerVoltage() const;

    /**
     * Returns the estimated position.
     */
    double EstimatedPosition() const;

    /**
     * Returns the estimated velocity.
     */
    double EstimatedVelocity() const;

    /**
     * Returns the error between the position reference and the position
     * estimate.
     */
    double PositionError() const;

    /**
     * Returns the error between the velocity reference and the velocity
     * estimate.
     */
    double VelocityError() const;

    /**
     * Returns the current reference set by the profile
     */
    double PositionReference();

    /**
     * Executes the control loop for a cycle.
     */
    void Update(void);

    /**
     * Resets any internal state.
     */
    void Reset(void);

    void SetClimbingProfile();

    void SetScoringProfile();

private:
    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_Y;
    TrapezoidalMotionProfile::State m_goal;

    TrapezoidalMotionProfile::Constraints scoringConstraints{kElevatorMaxV,
                                                             kElevatorMaxA};
    TrapezoidalMotionProfile::Constraints climbingConstraints{kElevatorClimbV,
                                                              kElevatorClimbA};
    TrapezoidalMotionProfile::Constraints m_activeConstraints =
        scoringConstraints;

    TrapezoidalMotionProfile m_positionProfile{scoringConstraints,
                                               {0_m, 0_mps}};

    TrapezoidalMotionProfile::State m_profiledReference;

    frc::StateSpacePlant<2, 1, 1> m_plant = [&] {
        frc::StateSpacePlant<2, 1, 1> plant{MakeElevatorPlantCoeffs()};
        plant.AddCoefficients(MakeElevatorClimbPlantCoeffs());
        return plant;
    }();

    frc::StateSpaceController<2, 1, 1> m_controller = [&] {
        frc::StateSpaceController<2, 1, 1> controller{
            MakeElevatorControllerCoeffs(), m_plant};
        controller.AddCoefficients(MakeElevatorClimbControllerCoeffs());
        return controller;
    }();

    frc::StateSpaceObserver<2, 1, 1> m_observer = [&] {
        frc::StateSpaceObserver<2, 1, 1> observer{MakeElevatorObserverCoeffs(),
                                                  m_plant};
        observer.AddCoefficients(MakeElevatorClimbObserverCoeffs());
        return observer;
    }();

    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;

    CsvLogger elevatorLogger{"/home/lvuser/Elevator.csv",
                             "Time,EstPos,PosRef,Voltage"};
};

}  // namespace frc3512
