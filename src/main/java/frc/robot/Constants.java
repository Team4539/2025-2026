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
        public static final double ElevatorSAFE = 8500; // Head will not hit the Elevator at this height
    }
    public static final class Carrige {
        public static final int CarrigeMotorID = 13; // Placeholder value
        public static final int CarrigeEncoderAID = 3; // DIO Port 3
        public static final int CarrigeEncoderBID = 4; // DIO Port 4
        public static final double CarrigeMaxHeight = 999999; // Placeholder value
        public static final double CarrigeMinHeight = -999999; // Placeholder value

        /*Carrige heights */
        public static final double HomePosition = 0; // Placeholder value
        public static final double coralL1Position = 0; // Placeholder value
        public static final double coralL2Position = 0; // Placeholder value
        public static final double coralL3Position = 0; // Placeholder value
        public static final double coralL4Position = 0; // Placeholder value
        public static final double alageL1Position = 0; // Placeholder value
        public static final double alageL2Position = 0; // Placeholder value
        public static final double alageGroundPosition = 0; // Placeholder value
        public static final double alageProcessorsPosition = 0; // Placeholder value
        public static final double CarrigeSAFE = 0; // Placeholder value
        public static final double CarriageUpHeight = 0; // Placeholder value
        public static final double ArmUnsafeMinHeight = 0; // Placeholder value
    }
       
        public static final class ArmRotator {
        public static final int ArmRotatorMotorID = 15; // Placeholder value
        public static final int ArmRotatorEncoderID = 2; // DIO Port 2
        public static final double ArmRotatorMaxAngle = 99; // Placeholder value
        public static final double ArmRotatorMinAngle = 0; // Placeholder value
        public static final double HeadSAFE = 44; // THIS IS CLEAR OF THE Elevator
        public static final double HeadPastSAFE = 92; // this is where wheel can hit swerve module
        public static final double RotationPerDegree = 20.25; // it takes this many rotation to move 1 degree

        /*Rotation positions*/
        public static final double HomeRotation = 44.2; // Placeholder value
        public static final double algaeGroundRotation = 0; // Placeholder value
        public static final double algaeProcessorsRotation = 73; // Placeholder value
        public static final double algaeReefRotation = 0; // Placeholder value
        public static final double algaeBargeRotation = 0; // Placeholder value
        public static final double coralReefAngledRotation = 48.0; // Placeholder value
        public static final double coralL4Rotation = 68.1; // Placeholder value
        public static final double coralStationRotation = 48.1 ; // Coral Instake Station Value
        public static final double HeadOutside = 0;


    }
    public static final class HeadMechanisms {
        public static final int HeadManipulatorMotorID = 16; // Placeholder value
        public static final int IntakeSpeed = 0; // Placeholder value
        public static final int OutTakeSpeed = 0; // Placeholder value
        public static final int AlgaeHoldSpeed = 0; // Placeholder value
    }
    public static final class Intake {
        public static final int IntakeMotorID = 0; // Placeholder value
        public static final int IntakeRotatorMotorID = 0; // Placeholder value
        public static final int IntakeRotatorEncoderID = 0; // Placeholder value
        public static final double IntakeRotatorMaxAngle = 0; // Placeholder value
        public static final double IntakeRotatorMinAngle = 0; // Placeholder value
        public static final double InsideRotation = 0; // Placeholder value
        public static final double OutsideRotation = 0; // Placeholder value
    }
    public static final class Climber {
        public static final int ClimberMotorID = 0; // Placeholder value
    }
}

