package frc.robot;

public class Constants {
    
    //TODO: Fill in the placeholder values with the actual values

    public static final double joyDeadband = 0; 

    public static final class Elevator {
        public static final int ElevatorMotorID = 14; // motor value
        public static final int ElevatorEncoderAID = 0; // DIO Port 0 
        public static final int ElevatorEncoderBID = 1; // DIO Port 1
        public static final double ElevatorMaxHeight = 0; // Will also be 0
        public static final double ElevatorMinHeight = 30000; // Tuned for our robot
        public static final double ElevatorAboveGround = 4103; // Placeholder value

        /*Elevator heights */
        public static final double HomePosition = 0; // Placeholder value
        public static final double coralL1Position = 0; // Placeholder value
        public static final double coralL2Position = 12000; // Placeholder value
        public static final double coralL3Position = 17000; // Placeholder value
        public static final double coralL4Position = 30000; // Placeholder value
        public static final double alageL1Position = 0; // Placeholder value
        public static final double alageL2Position = 0; // Placeholder value
        public static final double alageGroundPosition = 0; // Placeholder value
        public static final double alageProcessorsPosition = 0; // Placeholder value
        public static final double coaralStationPosition = 9449; // Placeholder value
    }
    public static final class HeadRotator {
        public static final int HeadRotatorMotorID = 15; // Placeholder value
        public static final int HeadRotatorEncoderID = 2; // DIO Port 2
        public static final double HeadRotatorMaxAngleOnGround = 90; // Placeholder value
        public static final double HeadRotatorMinAngleOnGround = 70; // Placeholder value
        public static final double HeadRotatorMaxAngleInAir = 80; // Placeholder value
        public static final double HeadRotatorMinAngleInAir = 64 ; // Placeholder value

        /*Rotation positions*/
        public static final double HomeRotation = 70; // Placeholder value
        public static final double algaeGroundRotation = 0; // Placeholder value
        public static final double algaeProcessorsRotation = 0; // Placeholder value
        public static final double algaeReefRotation = 0; // Placeholder value
        public static final double algaeBargeRotation = 0; // Placeholder value
        public static final double coralReefAngledRotation = 0; // Placeholder value
        public static final double coralL4Rotation = 90; // Placeholder value
        public static final double coralStationRotation = 49.6; // Coral Instake Station Value

    }
    public static final class HeadMechanisms {
        public static final int CoralManipulatorMotorID = 16; // Placeholder value
        public static final int AlageManipulatorMotorID = 17; // Placeholder value
    }
    public static final class Climber {
        public static final int ClimberMotorID = 0; // Placeholder value
        public static final double ClimberMaxSpeed = 0; // Placeholder value
    }
}
