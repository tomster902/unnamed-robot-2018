/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "WPILib.h"

#include "Commands/ExampleCommand.h"
#include "Commands/MyAutoCommand.h"

class Robot : public frc::IterativeRobot {
public:
	class PIDRotate : public PIDOutput {
		public:
			PIDRotate(MecanumDrive& drive, ADXRS450_Gyro& gyro) {
				// Initialize the driving and gyro controlling classes
				m_drive = &drive;
				m_gyro = &gyro;
			}

			void SetX(double x) {
				m_x = x;
			}

			void SetY(double y) {
				m_y = y;
			}

			void SetRotate(double rotate) {
				m_rotate = rotate;
			}

			void DriveCartesian(double x, double y, double rotate) {
				m_x = x;
				m_y = y;
				m_rotate = rotate;
			}

			/**
			 * Collects data from the PID loop.
			 */
			void PIDWrite(double output) override {
				double turningValue = (kAngleSetpoint - m_gyro->GetAngle()) * kP; // The turning value how fast the robot should be rotated
				turningValue = std::copysign(turningValue, output); // Turns in the direction of the PID output
				m_drive->DriveCartesian(m_x, m_y, turningValue + m_rotate); // Final drive code using frc's Mecanum Drive class.
			}
		private:
			const double kAngleSetpoint = 0.0;
			const double kP = 0.01;
			MecanumDrive *m_drive;
			ADXRS450_Gyro *m_gyro;
			double m_x = 0.0;
			double m_y = 0.0;
			double m_rotate = 0.0;
		};

	void RobotInit() override {
		/////////// QUESTIONABLE USEFULNESS //////////////
		m_chooser.AddDefault("Default Auto", &m_defaultAuto);
		m_chooser.AddObject("My Auto", &m_myAuto);
		//////////////////////////////////////////////////
		Talon *rightFrontWheel = new Talon(2); // Initialize the right-front wheel's speed controller on PWM channel 2
		Talon *leftFrontWheel = new Talon(1); // Initialize the left-front wheel's speed controller on PWM channel 1
		Talon *rightBackWheel = new Talon(4); // Initialize the right-backt wheel's speed controller on PWM channel 4
		Talon *leftBackWheel = new Talon(3); // Initialize the left-back wheel's speed controller on PWM channel 3
		m_logitech = new Joystick(0); // Define Logitech joystick being used at USB port #0 on the Drivers Station
		// Initialize Mecanum Drive with all the wheels
		m_mecanumDrive = new MecanumDrive(*leftFrontWheel, *leftBackWheel, *rightFrontWheel, *rightBackWheel);
		gyro = new ADXRS450_Gyro(); // Initialize the orientation correcting gyroscope
		rotate_controller = new PIDRotate(*m_mecanumDrive, *gyro); // Defines an instance the custom PIDOutput class
		gyro_controller = new PIDController(p, i, d, gyro, rotate_controller); // Initializes the PIDConroller with the gyro as an input
		// and the rotate mecanum axis as an output

		// Used for tuning on the Smart Dashboard
		SmartDashboard::PutNumber("P", p); // Tunes p
		SmartDashboard::PutNumber("I", i); // Tunes i
		SmartDashboard::PutNumber("D", d); // Tunes d
		SmartDashboard::PutNumber("F", f); // Tunes f

		// PIDController initalization code.
		gyro_controller->SetContinuous(); // The PIDController shouldn't stop.
		gyro_controller->SetSetpoint(setpoint); // Set the angle setpoint to, or straight ahead.
		gyro_controller->SetInputRange(-180.0, 180.0); // The possible range of angles the gyro can measure.
		gyro_controller->SetOutputRange(-K, K); // The speed of the rotation to eliminate error.
		gyro_controller->Enable(); // Start the PIDController loop.

		frc::SmartDashboard::PutData("Auto Modes", &m_chooser); // Default Code
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		std::string autoSelected = frc::SmartDashboard::GetString(
				"Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			m_autonomousCommand = &m_myAuto;
		} else {
			m_autonomousCommand = &m_defaultAuto;
		}

		m_autonomousCommand = m_chooser.GetSelected();

		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Cancel();
			m_autonomousCommand = nullptr;
		}
		gyro->Calibrate(); // Calibrates the gyro (VERY IMPORTANT)
		Wait(5); // Calibrate for 5 seconds to ensure accurate gyro measures.
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		double strafe = m_logitech->GetRawAxis(4); // The 4th axis on the controller controls strafing
		double forward = -m_logitech->GetRawAxis(5); // The 4th axis on the controller controls moving forward and backwards
		double rotate = m_logitech->GetRawAxis(0); // The 0 axis on the controller controls manual rotation

		if (m_logitech->GetRawButton(2)) {
			while(m_logitech->GetRawButton(2));
			gyro_controller->Disable();

			double turningValue = (90.0 - gyro->GetAngle()) * 0.01; // The turning value how fast the robot should be rotated
			turningValue = std::copysign(turningValue, -1); // Turns in the direction of the PID output
			m_mecanumDrive->DriveCartesian(0, 0, turningValue); // Final drive code using frc's Mecanum Drive class.
			gyro->Reset();
			gyro_controller->Enable();

		}

		if (m_logitech->GetRawButton(1)) {
			while(m_logitech->GetRawButton(2));
			double turningValue = (-90.0 - gyro->GetAngle()) * 0.01; // The turning value how fast the robot should be rotated
			turningValue = std::copysign(turningValue, 1); // Turns in the direction of the PID output
			m_mecanumDrive->DriveCartesian(0, 0, turningValue); // Final drive code using frc's Mecanum Drive class.
			while (!(gyro->GetAngle() > 85 && gyro->GetAngle() < 95));
			gyro->Reset();
			gyro_controller->Enable();

		}

		// Controls the robot speed. K is the constant of controlling the acceleration.
		rotate*=K;
		forward*=K;
		strafe*=K;

		// Used for tuning
		gyro_controller->SetP(SmartDashboard::GetNumber("P", p));
		gyro_controller->SetI(SmartDashboard::GetNumber("I", i));
		gyro_controller->SetD(SmartDashboard::GetNumber("D", d));
		gyro_controller->SetF(SmartDashboard::GetNumber("F", f));

		rotate_controller->DriveCartesian(strafe, forward, rotate); // Drive function
	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	MecanumDrive *m_mecanumDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_logitech;
	ADXRS450_Gyro *gyro;
	PIDController *gyro_controller;
	PIDRotate *rotate_controller;
	frc::Command* m_autonomousCommand = nullptr;
	ExampleCommand m_defaultAuto;
	MyAutoCommand m_myAuto;
	frc::SendableChooser<frc::Command*> m_chooser;
	double setpoint = 0.0;
	const double K = 0.7;
	const double p = 1.0;
	const double i = 0.1;
	const double d = 0.1;
	const double f = 0.1;
};

START_ROBOT_CLASS(Robot)
