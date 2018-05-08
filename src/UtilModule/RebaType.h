#pragma once

#include <Eigen/Dense>

namespace Eigen
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

enum StopCondT{
	DEFINED_POSITION = 0,
	OPTIMIZEGP = 1,
	OTHERS = 2
};

enum ActionResultT{
	NOACTIVE = 0,
	RUNNING = 1,
	SUCCESSFUL = 2,
	UNEXPECTED = 3
};

enum RobotNameT{
	kuka_left,
	kuka_right
};

enum ToolNameT{
    none,
    sensing_pole,
    teensy_finger,
    tactool,
    myrmex_sensor,
    hingedtool
};

enum MotionDirT{
	UP = 0,
	DOWN = 1,
	BACK = 2,
	FORWARD = 3,
	APPROACH = 4,
	AWAY = 5,
	LEFT = 6,
	RIGHT = 7,
	RXP = 8,
	RXN = 9,
	RYP = 10,
	RYN = 11,
	RZP = 12,
	RZN = 13,
	STOP = 14,
    NOCONTROL = 15,
    VISGUIDE = 16
};

enum RobotModeT{
    NormalMode = 0,
    PsudoGravityCompensation = 1
};

enum ForceCtrlT{
    FTCTRL = 0,
    TACCTRL = 1
};

enum ContactPositionT{
    eff,
    ct,
    eff_tacform
} ;

enum TacSensorType{
    Myrmex,
    MIDtip
};

enum ManipuToolT{
    Notool,
    Tacbrush
};

struct RG_Pose{
    Eigen::Vector3d p;
    Eigen::Matrix3d o;
    Eigen::Matrix3d rel_o;
    RG_Pose(){
        p.setZero();
        o.setZero();
        rel_o.setZero();
    }
};

enum EXPDIR{
    NOMOVE = 0,
    XP = 1,
    XN = 2,
    YP = 3,
    YN = 4
};

enum ControllerT{
    //theta_dot = inv(J) * x_dot: least square solution
    LSSolution_Ctrl,
    //Wupoential as secondary task, originall created by Matthias
    WuPotential_Ctrl,
    //Joint limitation constraint as secondary task, from "open a door with a redundant impedance controlled robot"
    JLPotential_Ctrl,
    //Internal movement Potential as secondar task, main the desired pose in eef and change the joint angle configure to desired ref.
    IMPotential_Ctrl
};


// class ctrlpara{
// public:
// 	double kp;
// 	double ki;
// 	double kd;
// 	ctrlpara(){kp = 0.0; ki = 0.0; kd = 0.0;};
// };
// 
// class kukapara{
// public:
// 	double axis_stiffness[7];
// 	double axis_damping[7];
// 	kukapara(){
// 		for(int i = 0; i < 7; i++){
// 			axis_damping[i] = 0.2;axis_stiffness[i] = 500;}
// 	}
// };




