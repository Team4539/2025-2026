package frc.robot;

public class Constants {
    
    //TODO: Fill in the placeholder values with the actual values

    public static final double joyDeadband = 0; 

    public static final class Elevator {
        public static final int ElevatorMotorID = 14; // motor value
        public static final int ElevatorEncoderAID = 0; // DIO Port 0 
        public static final int ElevatorEncoderBID = 1; // DIO Port 1
        public static final double ElevatorMaxHeight = -999999; // Will also be 0
        public static final double ElevatorMinHeight = 9999999; // Tuned for our robot
        public static final double ElevatorAboveGround = 4103; // Placeholder value

        /*Elevator heights */
        public static final double HomePosition = -5; // Placeholder value
        public static final double coralL1Position = 0; // Placeholder value
        public static final double coralL2Position = 10000; // Placeholder value
        public static final double coralL3Position = 17400; // Placeholder value
        public static final double coralL4Position = 30000; // Placeholder value
        public static final double alageL1Position = 0; // Placeholder value
        public static final double alageL2Position = 0; // Placeholder value
        public static final double alageGroundPosition = 0; // Placeholder value
        public static final double alageProcessorsPosition = 0; // Placeholder value
        public static final double coaralStationPosition = 10551; // Placeholder value
        public static final double ElevatorSAFE = 8500; // Head will not hit the elevator at this height
        public static final double ElevatornotTouching = 400;
        public static final double ElevatorGearRatio = 1;

        /*Elevator Motion magic configs */
        public static final double CruiseVelocity = 0; // velocity in units/100ms
        public static final double Acceleration = 0; // acceleration in units/100ms^2
        public static final double Jerk = 0; // Jerk in units/100ms^3
        public static double kS; // Voltage to Overcome Static Friction 
        public static double kV; // output per unit of target velocity (output/rps)
        public static double kA; // output per unit of target acceleration (output/rps^2)
        public static double kP; // output per unit of error in position (output/rotations)
        public static double kI; // output per unit of integraed error in position (output/(rotations*second))
        public static double kD; // output per unit of error in velocity (output/rps)
    }
    public static final class HeadRotator {
        public static final int HeadRotatorMotorID = 15; // Placeholder value
        public static final int HeadRotatorEncoderID = 2; // DIO Port 2
        public static final double HeadRotatorMaxAngle = 99; // Placeholder value
        public static final double HeadRotatorMinAngle = 0; // Placeholder value
        public static final double HeadSAFE = 44; // THIS IS CLEAR OF THE ELEVATOR
        public static final double HeadPastSAFE = 92; // this is where wheel can hit swerve module
        public static final double RotationPerDegree = 20.25; // it takes this many rotation to move 1 degree

        /*Rotation positions*/
        public static final double HomeRotation = 44.2;
        ; // Placeholder value
        public static final double algaeGroundRotation = 0; // Placeholder value
        public static final double algaeProcessorsRotation = 73
        ; // Placeholder value
        public static final double algaeReefRotation = 0; // Placeholder value
        public static final double algaeBargeRotation = 0; // Placeholder value
        public static final double coralReefAngledRotation = 48.0; // Placeholder value
        public static final double coralL4Rotation = 68.1; // Placeholder value
        public static final double coralStationRotation = 48.1 ; // Coral Instake Station Value
        public static double kS;
        public static double kV;
        public static double kA;
        public static double kP;
        public static double kI;
        public static double kD;
        public static final double HeadGearRatio = 1;
        public static final double CruiseVelocity = 0; // velocity in units/100ms
        public static final double Acceleration = 0; // acceleration in units/100ms^2
        public static final double Jerk = 0; // Jerk in units/100ms^3
        

    }
    public static final class HeadMechanisms {
        public static final int CoralManipulatorMotorID = 16; // Placeholder value
        public static final int AlageManipulatorMotorID = 17; // Placeholder value
        public static final double AlgaeIntakeSpeed = 1; // Placeholder value
        public static final double CoralIntakeSpeed = -.5; // Placeholder value
        public static final double CoralOuttakeSpeed = 1; // Placeholder value
        public static final double AlgaeOuttakeSpeed = -1; // Placeholder value
        public static final double AlgaeHoldSpeed = .2; // Placeholder value
    }
    public static final class Climber {
        public static final int ClimberMotorID = 0; // Placeholder value
        public static final double ClimberMaxSpeed = 0; // Placeholder value
    }
}
