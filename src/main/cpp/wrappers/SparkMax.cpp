#include "wrappers/SparkMax.h"
#include <numbers>

void SparkMax::Configure()
{
    m_motor.Configure(m_config, rev::ResetMode::kResetSafeParameters,
                     rev::PersistMode::kNoPersistParameters);
}

void SparkMax::Set(double speed)
{
    m_motor.Set(speed);
}

double SparkMax::Get()
{
    return m_motor.Get();
}

void SparkMax::StopMotor()
{
    m_motor.StopMotor();
}

void SparkMax::SetSmartCurrentLimit(int limit)
{
    m_config.SmartCurrentLimit(limit);
}

void SparkMax::SetFeedbackSensor(rev::spark::FeedbackSensor type)
{
    m_config.closedLoop.SetFeedbackSensor(type);
    m_usedEncoderType = type;
}

void SparkMax::SetInverted(bool isInverted)
{
    m_config.Inverted(isInverted);
}

double SparkMax::GetVelocity()
{
    if (m_usedEncoderType == rev::spark::FeedbackSensor::kPrimaryEncoder) {
        return m_relativeEncoder.GetVelocity();
    } else if (m_usedEncoderType == rev::spark::FeedbackSensor::kAbsoluteEncoder) {
        return m_absoluteEncoder.GetVelocity();
    }
    return 0.0;
}

double SparkMax::GetPosition()
{
    if (m_usedEncoderType == rev::spark::FeedbackSensor::kPrimaryEncoder) {
        return m_relativeEncoder.GetPosition();
    } else if (m_usedEncoderType == rev::spark::FeedbackSensor::kAbsoluteEncoder) {
        return m_absoluteEncoder.GetPosition();
    }
    return 0.0;
}

void SparkMax::SetPID(double p, double i, double d, double ff, int slot)
{
    switch (slot)
    {
    case 1:
        m_config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot1);
        break;
    case 2:
        m_config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot2);
        break;
    case 3:
        m_config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot3);
        break;
    
    default:
        m_config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot0);
        break;
    }
}

void SparkMax::SetReference(double reference, rev::spark::SparkLowLevel::ControlType controlType)
{
    if (controlType == rev::spark::SparkLowLevel::ControlType::kPosition) {
        m_closedLoopController.SetSetpoint(reference, rev::spark::SparkLowLevel::ControlType::kPosition);
    } else if (controlType == rev::spark::SparkLowLevel::ControlType::kVelocity) {
        m_closedLoopController.SetSetpoint(reference, rev::spark::SparkLowLevel::ControlType::kVelocity);
    } else {
        // Default to duty cycle control if an unsupported control type is provided
        m_closedLoopController.SetSetpoint(2 * (std::atan(reference)) / std::numbers::pi, rev::spark::SparkLowLevel::ControlType::kDutyCycle);  
    }
}

rev::spark::FeedbackSensor SparkMax::GetEncoderType()
{
    return m_usedEncoderType;
}

void SparkMax::SetRelativeEncoderPosition(double position)
{
    m_relativeEncoder.SetPosition(position);
}

void SparkMax::SetRelativePositionConversionFactor(double factor)
{
    m_config.encoder.PositionConversionFactor(factor);
}

void SparkMax::SetAbsolutePositionConversionFactor(double factor)
{
    m_config.absoluteEncoder.PositionConversionFactor(factor);
}