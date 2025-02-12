package frc.robot.commands;

import frc.robot.subsystems.HeadRotationSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants;

public class SetHeadTo extends Command {
    private final HeadRotationSubsystem m_head;
    private final PIDController pidController;
    private String m_command;
    private double m_setpoint;
    private double m_fixedOutput;
    private final boolean m_needSelection;
    private final Joystick m_joystick = new Joystick(2);
    private final POVButton PovUP = new POVButton(m_joystick, 0);
    private final POVButton PovRight = new POVButton(m_joystick, 90);
    private final POVButton PovLeft = new POVButton(m_joystick, 270);
    private final POVButton PovDown = new POVButton(m_joystick, 180);

    public SetHeadTo(HeadRotationSubsystem subsystem, double setpoint, String command, boolean needSelection) {
        addRequirements(subsystem);
        m_head = subsystem;
        if (command == null) {
            m_command = "Disabled";
        } else {
            m_command = command;
        }
        m_setpoint = setpoint;
        pidController = new PIDController(1, 0 , 0.02);
        m_needSelection = needSelection;
    }

    @Override
    public void execute() {
       // if (!m_needSelection) {
            double Rotation = m_head.GetHeadEncodor();
            double output = pidController.calculate(Rotation, m_setpoint);
            if (output > 1) {
                m_fixedOutput = 1;
            } else {
                m_fixedOutput = output;
            }

            if (Rotation > m_setpoint) {
                m_head.SetHeadRotation(m_fixedOutput, m_command);
                DriverStation.reportError("I went past my setpoint", false);
            } else if (Rotation < m_setpoint) {
                m_head.SetHeadRotation(m_fixedOutput, m_command);
                DriverStation.reportWarning("I am going to my setpoint", false);
            } else {
                m_head.SetHeadRotation(0, m_command + " At Setpoint");
            }
        // } else {
        //     if (PovUP.getAsBoolean()) {
        //         m_setpoint = Constants.HeadRotator.coralL4Rotation;
        //         m_command = "Coral L4";
        //         double Rotation = m_head.GetHeadEncodor();
        //         double output = pidController.calculate(Rotation, m_setpoint) / 1000;
        //         if (output > 1) {
        //             m_fixedOutput = 1;
        //         } else {
        //             m_fixedOutput = output;
        //         }
    
        //         if (Rotation > m_setpoint) {
        //             m_head.SetHeadRotation(-m_fixedOutput, m_command);
        //         } else if (Rotation < m_setpoint) {
        //             m_head.SetHeadRotation(m_fixedOutput, m_command);
        //         } else {
        //             m_head.SetHeadRotation(0, m_command + " At Setpoint");
        //         }
        //         SmartDashboard.putBoolean("POV up", true);
        //     } else if (PovRight.getAsBoolean()) {
        //         m_setpoint = Constants.HeadRotator.coralReefAngledRotation;
        //         m_command = "Coral L3";
        //         double Rotation = m_head.GetHeadEncodor();
        //         double output = pidController.calculate(Rotation, m_setpoint) / 1000;
        //         if (output > 1) {
        //             m_fixedOutput = 1;
        //         } else {
        //             m_fixedOutput = output;
        //         }
    
        //         if (Rotation > m_setpoint) {
        //             m_head.SetHeadRotation(-m_fixedOutput, m_command);
        //         } else if (Rotation < m_setpoint) {
        //             m_head.SetHeadRotation(m_fixedOutput, m_command);
        //         } else {
        //             m_head.SetHeadRotation(0, m_command + " At Setpoint");
        //         }
        //         SmartDashboard.putBoolean("POV right", true);
        //     } else if (PovLeft.getAsBoolean()) {
        //         m_setpoint = Constants.HeadRotator.coralReefAngledRotation;
        //         m_command = "Coral L2";
        //         double Rotation = m_head.GetHeadEncodor();
        //         double output = pidController.calculate(Rotation, m_setpoint) / 1000;
        //         if (output > 1) {
        //             m_fixedOutput = 1;
        //         } else {
        //             m_fixedOutput = output;
        //         }
    
        //         if (Rotation > m_setpoint) {
        //             m_head.SetHeadRotation(-m_fixedOutput, m_command);
        //         } else if (Rotation < m_setpoint) {
        //             m_head.SetHeadRotation(m_fixedOutput, m_command);
        //         } else {
        //             m_head.SetHeadRotation(0, m_command + " At Setpoint");
        //         }
        //         SmartDashboard.putBoolean("POV left", true);
        //     } else if (PovDown.getAsBoolean()) {
        //         m_setpoint = Constants.HeadRotator.HomeRotation;
        //         m_command = "Home";
        //         double Rotation = m_head.GetHeadEncodor();
        //         double output = pidController.calculate(Rotation, m_setpoint) / 1000;
        //         if (output > 1) {
        //             m_fixedOutput = 1;
        //         } else {
        //             m_fixedOutput = output;
        //         }
    
        //         if (Rotation > m_setpoint) {
        //             m_head.SetHeadRotation(-m_fixedOutput, m_command);
        //         } else if (Rotation < m_setpoint) {
        //             m_head.SetHeadRotation(m_fixedOutput, m_command);
        //         } else {
        //             m_head.SetHeadRotation(0, m_command + " At Setpoint");
        //         }
        //         SmartDashboard.putBoolean("POV down", true);
        //     } else {
        //         SmartDashboard.putBoolean("POV up", false);
        //         SmartDashboard.putBoolean("POV right", false);
        //         SmartDashboard.putBoolean("POV left", false);
        //         SmartDashboard.putBoolean("POV down", false);
        //     }
        //}
    }

    @Override
    public void end(boolean interrupted) {
        m_head.SetHeadRotation(0, "Disabled");
    }
}
