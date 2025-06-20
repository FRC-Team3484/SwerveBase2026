#ifndef OI_H
#define OI_H

#include <frc/XboxController.h>

#include "Constants.h"

class Driver_Interface {
    public:
        Driver_Interface();
        //  Swerve Controllers
        double GetThrottle();
        double GetStrafe();
        double GetRotation();

        bool GetResetHeading();
        bool GetBrake();
        bool GetBrakePressed();

        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();

        bool LowSpeed();
        void SetRumble(double Rumble);

        bool DriverOverride();

    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

class Operator_Interface {
    public:
        Operator_Interface();

        void SetRumble(double Rumble);
        bool GetIgnoreVision();
        
        int RawPOV();

    private:
        frc::XboxController _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};

class Testing_Interface {
    public:
        Testing_Interface();
    private:
        frc::XboxController _testing_controller{UserInterface::Testing::TESTING_CONTROLLER_PORT};
};
#endif