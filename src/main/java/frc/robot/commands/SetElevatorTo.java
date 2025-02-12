package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorTo extends Command{
    private final ElevatorSubsystem m_elevator;
    private final PIDController pidController;
    private String m_command;
        private double m_setpoint;
        private double fixedOutput;
        private final boolean m_needSelection;
        private final Joystick m_joystick = new Joystick(2);
        private final POVButton PovUP= new POVButton(m_joystick, 0);
        private final POVButton PovRight = new POVButton(m_joystick, 90);
        private final POVButton PovLeft = new POVButton(m_joystick, 270);
        private final POVButton PovDown = new POVButton(m_joystick, 180);
    
        public SetElevatorTo(ElevatorSubsystem subsystem,  double setpoint, String command, boolean needSelection) {
            addRequirements(subsystem);
            m_elevator = subsystem;
            if (command == null) {
                m_command = "Disabled";
            }
            else {
                m_command = command;
            }
            m_setpoint = setpoint;
            pidController = new PIDController(.6, 10, 0.02);
            m_needSelection = needSelection;
        }
        
    
        public void periodic() {
            SmartDashboard.putBoolean("POV up", PovUP.getAsBoolean());
            SmartDashboard.putBoolean("POV right", PovRight.getAsBoolean());
            SmartDashboard.putBoolean("POV left", PovLeft.getAsBoolean());
            SmartDashboard.putBoolean("POV down", PovDown.getAsBoolean());
            
        }
    
        @Override
        public void execute() {
            if (m_needSelection == false) {
                double Height = m_elevator.GetElevatorHeight();
                double output = pidController.calculate(Height, m_setpoint)/1000;
                if (output > 1) {
                    fixedOutput = 1;
                }
                else fixedOutput = output;
    
                if (Height > m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else if (Height <  m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else{
                    m_elevator.SetElevator(0, m_command+" At Setpoint");
                }
            }
            else {
                if (PovUP.getAsBoolean()) {
                    m_setpoint = Constants.Elevator.coralL4Position;
                    m_command = "Coral L4";
                SmartDashboard.putBoolean("POV up", true);
                double Height = m_elevator.GetElevatorHeight();
                double output = pidController.calculate(Height, m_setpoint)/1000;
                if (output > 1) {
                    fixedOutput = 1;
                }
                else fixedOutput = output;
    
                if (Height > m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else if (Height <  m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else{
                    m_elevator.SetElevator(0, m_command+" At Setpoint");
                }


            }
            else if (PovRight.getAsBoolean()) {
                SmartDashboard.putBoolean("POV right", true);
                    m_setpoint = Constants.Elevator.coralL3Position;
                    m_command = "Coral L3";
                double Height = m_elevator.GetElevatorHeight();
                double output = pidController.calculate(Height, m_setpoint)/1000;
                if (output > 1) {
                    fixedOutput = 1;
                }
                else fixedOutput = output;
    
                if (Height > m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else if (Height <  m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else{
                    m_elevator.SetElevator(0, m_command+" At Setpoint");
                }
            }
            else if (PovLeft.getAsBoolean()) {
                SmartDashboard.putBoolean("POV left", true);
                    m_setpoint = Constants.Elevator.coralL2Position;
                    m_command = "Coral L2";
                double Height = m_elevator.GetElevatorHeight();
                double output = pidController.calculate(Height, m_setpoint)/1000;
                if (output > 1) {
                    fixedOutput = 1;
                }
                else fixedOutput = output;
    
                if (Height > m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else if (Height <  m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else{
                    m_elevator.SetElevator(0, m_command+" At Setpoint");
                }

            }
            else if (PovDown.getAsBoolean()) {
                SmartDashboard.putBoolean("POV down", true);
                m_setpoint = Constants.Elevator.HomePosition;
                m_command = "Home";
                double Height = m_elevator.GetElevatorHeight();
                double output = pidController.calculate(Height, m_setpoint)/1000;
                if (output > 1) {
                    fixedOutput = 1;
                }
                else fixedOutput = output;

                if (Height > m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else if (Height <  m_setpoint) {
                    m_elevator.SetElevator(-fixedOutput, m_command);
                }
                else{
                    m_elevator.SetElevator(0, m_command+" At Setpoint");
                }
            }
            else {
                SmartDashboard.putBoolean("POV up", false);
                SmartDashboard.putBoolean("POV right", false);
                SmartDashboard.putBoolean("POV left", false);
                SmartDashboard.putBoolean("POV down", false);
            }
        }

    }
    public void end(boolean interrupted) {
        m_elevator.SetElevator(0, "Disabled");
    }
}
