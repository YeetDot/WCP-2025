#pragma once

#include <rev/SparkMax.h>
#include <frc/drive/RobotDriveBase.h>

class SparkMax
{
public:
    SparkMax(int deviceID) : m_motor(deviceID, rev::spark::SparkLowLevel::MotorType::kBrushless) {};

    enum class EncoderType { Absolute, Relative };

    void Configure();

    void Set(double speed);
    double Get();
    void StopMotor();

    void SetSmartCurrentLimit(int limit);
    void SetFeedbackSensor(rev::spark::FeedbackSensor type);
    void SetInverted(bool isInverted);

    double GetVelocity();
    double GetPosition();
    void SetPID(double p, double i = 0, double d = 0, double ff = 0, int slot = 0);
    void SetReference(double reference, rev::spark::SparkLowLevel::ControlType controlType);

    rev::spark::FeedbackSensor GetEncoderType();
    void SetRelativeEncoderPosition(double position);
    void SetRelativePositionConversionFactor(double factor);
    void SetAbsolutePositionConversionFactor(double factor);

private:
    rev::spark::SparkMax m_motor;
    rev::spark::SparkAbsoluteEncoder& m_absoluteEncoder = m_motor.GetAbsoluteEncoder();
    rev::spark::SparkRelativeEncoder& m_relativeEncoder = m_motor.GetEncoder();
    rev::spark::SparkClosedLoopController& m_closedLoopController = m_motor.GetClosedLoopController();
    rev::spark::SparkBaseConfig m_config;
    rev::spark::FeedbackSensor m_usedEncoderType{rev::spark::FeedbackSensor::kPrimaryEncoder};
};