// Copyright (c) 2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units.h>

/**
 * A trapezoidal-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * Initialization:
 * @code{.cpp}
 * TrapezoidalMotionProfile::Constraints constraints{kMaxV, kMaxA};
 * double previousProfiledReference = initialReference;
 * @endcode
 *
 * Run on update:
 * @code{.cpp}
 * TrapezoidalMotionProfile profile{constraints, unprofiledReference,
 *                                  previousProfiledReference};
 * previousProfiledReference = profile.Calculate(timeSincePreviousUpdate);
 * @endcode
 *
 * where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `Calculate()` returns the
 * unprofiled reference unchanged.
 *
 * Otherwise, a timer can be started to provide monotonic values for
 * `Calculate()` and to determine when the profile has completed via
 * `IsFinished()`.
 */
class TrapezoidalMotionProfile {
public:
    class Constraints {
    public:
        units::meters_per_second_t maxVelocity = 0_mps;
        units::meters_per_second_squared_t maxAcceleration = 0_mps_sq;
    };

    class State {
    public:
        units::meter_t position = 0_m;
        units::meters_per_second_t velocity = 0_mps;
        bool operator==(const State& rhs) const {
            return position == rhs.position && velocity == rhs.velocity;
        }
    };

    /**
     * Construct a TrapezoidalMotionProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal        The desired state when the profile is complete.
     * @param initial     The initial state (usually the current state).
     */
    TrapezoidalMotionProfile(Constraints constraints, State goal,
                             State initial = State{0_m, 0_mps});

    /**
     * Calculate the correct position and velocity for the profile at a time t
     * where the beginning of the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     */
    State Calculate(units::second_t t) const;

    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param target The target distance.
     */
    units::second_t TimeLeftUntil(units::meter_t target) const;

    /**
     * Returns the total time the profile takes to reach the goal.
     */
    units::second_t TotalTime() const { return m_endDeccel; }

    /**
     * Returns true if the profile has reached the goal.
     *
     * The profile has reached the goal if the time since the profile started
     * has exceeded the profile's total time.
     *
     * @param t The time since the beginning of the profile.
     */
    bool IsFinished(units::second_t t) const { return t >= TotalTime(); }

private:
    /**
     * Returns true if the profile inverted.
     *
     * The profile is inverted if the velocity needs to decrease rather than
     * increase to reach the peak velocity from the initial velocity.
     *
     * @param initial     The initial state (usually the current state).
     * @param goal        The desired state when the profile is complete.
     * @param constraints The constraints on the profile, like maximum velocity.
     */
    bool ShouldFlipAcceleration(const State& initial, const State& goal,
                                const Constraints& constraints) const {
        // Calculate the distance travelled by a linear velocity ramp from the
        // initial to the final velocity and compare it to the desired distance.
        // If it is smaller, invert the profile.
        units::meters_per_second_t velocityChange =
            goal.velocity - initial.velocity;

        units::meter_t distanceChange = goal.position - initial.position;

        units::second_t t =
            units::math::abs(velocityChange) / constraints.maxAcceleration;
        bool isAccelerationFlipped =
            t * (velocityChange / 2 + initial.velocity) > distanceChange;
        return isAccelerationFlipped;
    }

    // Flip the sign of the velocity and position if the profile is inverted
    State Direct(const State& in) const {
        State result = in;
        result.position *= m_direction;
        result.velocity *= m_direction;
        return result;
    }

    // The direction of the profile, either 1 for forwards or -1 for inverted
    int m_direction;

    Constraints m_constraints;
    State m_initial;
    State m_goal;

    units::second_t m_endAccel, m_endFullSpeed, m_endDeccel;
};
