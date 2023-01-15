package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

public class Algorithms002 {
    // linear actuator; 3.15 seconds to reach the bottom to top
    // linear actuator; around 3.1 seconds from top to bottom
    public static final float mmPerInch = 25.4f;

    // values for the Mecanum wheels
    public static final float wheelCircumferenceMm = 301.59f;
    public static final float rotationPerRevolution = 2.314f;

    // dictates how fast the bot goes (0.05f is really slow, 0.1f is regular, more is faster)
    float controlMultiplier = 0.07f;

    public void Initialize() {

    }

    // this function determines the force of the linear actuator motor that moves the gripper arm up and down
    public double getLinearActuatorForce(double y) {
        if (y > 0) {
            return y;
        } else if (y < 0) {
            return y;
        } else {
            return 0;
        }
    }

    public double getGripperArmForce(double y) {
        if (y > 0) {
            return y;
        } else if (y < 0) {
            return y;
        } else {
            return 0;
        }
    }

    // i == 1 is left_front
    // i == 2 is right_front
    // i == 3 is left_back
    // i == 4 is right_back
    // z is the magnitude of the joystick's movement
    // x is the x axis of the left joystick
    // y is the y axis of the left joystick
    // x2 is the x axis of the right joystick
    public double getWheelForceManual(double x, double y, int i, double x2, double theta, double z) {
        // declares common values used multiple times in this function
        // absX is the absolute value of x
        // absY is the absolute value of y
        // rZ is UNKNOWN
        // movePower is the power of movement of the bot --> NEEDS CLARITY
        // rotationPower is the power of rotation of the bot --> NEEDS CLARITY
        // power is the final power to be returned
        double absX = Math.abs(x);
        double absY = Math.abs(y);
        double rZ = Math.abs(x2);
        double movePower = 0.0;
        double rotationPower = 0.0;
        double power;

        // this if statement returns 0 as no values are being inputted
        if (x == 0 && y == 0 && x2 == 0 && z == 0) {
            return 0;
        }

        // this if statement chain determines the value of movePower --> NEEDS CLARITY
        if (x != 0 || y != 0) {
            if (i == 1 || i == 4) {
                if (absX > 0) {
                    if (absX > absY) {
                        double t = TrueSign(x);
                        movePower = t;
                    } else {
                        double t = TrueSign(y);
                        movePower = t;
                    }
                } else if (absY > 0) {
                    double t = TrueSign(y);
                    movePower = t;
                }
            }
            if (i == 2 || i == 3) {
                if (absX > 0) {
                    if (absX > absY) {
                        double t = TrueSign(x);
                        movePower = -t;
                    } else {
                        double t = TrueSign(y);
                        movePower = t;
                    }
                } else if (absY > 0) {
                    double t = TrueSign(y);
                    movePower = t;
                }
            }
        }

        // this if statement chain determines the value of rotationPower --> NEEDS CLARITY
        if (x2 != 0) {
            if (i == 1 || i == 3) {
                if (x2 > 0) {
                    rotationPower = 1;
                } else if (x2 < 1) {
                    rotationPower = -1;
                } else {
                    rotationPower = 0;
                }
            } else if (i == 2 || i == 4) {
                if (x2 > 0) {
                    rotationPower = -1;
                } else if (x2 < 0) {
                    rotationPower = 1;
                } else {
                    rotationPower = 0;
                }
            }
        }

        // UNKNOWN
        rotationPower *= rZ;

        // this if statement chain determines the value of the power variable
        if (movePower != 0 && rotationPower == 0) {
            power = movePower;
        } else if (movePower == 0 && rotationPower != 0){
            power = rotationPower;
        } else {
            power = (movePower + rotationPower) / 2;
        }

        // this returns the power
        return power;
    }

    // forgor
    public double TrueSign(double n) {
        if (n > 0) {
            return 1;
        } else if (n == 0) {
            return 0;
        } else {
            return -1;
        }
    }

    double angleAdder = 0;
    public double theta(double x, double y, int q)
    {
        angleAdder = 0;
        double iAngle = (Math.PI * q / 2 + angleAdder);

        if(x == 0) {
            return iAngle / Math.PI;
        } else {
            double div = abs(y) / abs(x);
            double angle = Math.atan(abs(div)) + iAngle;

            if (angle >= 2 * Math.PI) {
                angle %= 2 * Math.PI;
            }

            return angle / Math.PI;
        }
    }

    public int getQuad(double x, double y) {
        if(x == 0 && y == 0)
            return 0;
        else if(x > 0 && y > 0)
            return 0;
        else if(x < 0 && y > 0)
            return 1;
        else if(x < 0 && y < 0)
            return 2;
        else if(x > 0 && y < 0)
            return 3;
        else if(x > 0 && y == 0)
            return 0;
        else if(x == 0 && y > 0)
            return 0;
        else if(x < 0 && y == 0)
            return 1;
        else if(x == 0 && y < 0)
            return 2;
        else
            return 0;
    }
}