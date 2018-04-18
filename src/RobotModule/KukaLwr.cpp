#include "KukaLwr.h"
#include <boost/assign/list_of.hpp>
#include "WuPotential.h"
#include <unistd.h>
#include "ControllerModule/actcontroller.h"
#include "ControllerModule/CtrlParam.h"
#include "UtilModule/eigen_to_kdl.h"

#define initP_x 0.28
#define initP_y 0.3
#define initP_z 0.25

void KukaLwr::setReference (CBF::FloatVector new_ref){
    if (new_ref.size() != 6){
        std::cerr << "Passing vector of size " << new_ref.size() << " instead of 6" << std::endl;
        return;
    }
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    currentTaskTargetP->set_reference(new_ref);
    currentTaskReferenceP->set_reference(new_ref);
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
}


void KukaLwr::setReference (double* positions){
    CBF::FloatVector ref(6);
    for (int i=0; i < 6; i++)
        ref(i) = positions[i];
    setReference(ref);
}

void KukaLwr::setReference (CBF::Float x, CBF::Float y, CBF::Float z, CBF::Float ra, CBF::Float rb, CBF::Float rc){
    CBF::FloatVector ref(6);
    ref(0) = x;
    ref(1) = y;
    ref(2) = z;
    ref(3) = ra;
    ref(4) = rb;
    ref(5) = rc;
    setReference (ref);
}


void KukaLwr::update_robot_state(){
    KDL::JntArray q = JntArray (7);
    KDL::JntArray q2 = JntArray (7);
    KDL::Frame position;
    KDL::Rotation TM_kdl;
    KDL::Vector position_p_kdl;
    KDL::Frame baseposition;
    for (int i=0; i < LBR_MNJ; i++){
        q(i) = jnt_position_act[i];
    }
    for (int i=0; i < LBR_MNJ; i++){
        q2(i) = jnt_position_mea[i];
    }
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    worldToToolFkSolver->JntToCart(q,position);
    baseToToolFkSolver->JntToCart (q2,baseposition);
    Jac_kdl.resize(7);
    worldToToolJacSolver->JntToJac(q,Jac_kdl);
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
    TM_kdl = position.M;
    position_p_kdl = position.p;
    conversions::convert(TM_kdl,m_TM_eigen);
    conversions::convert(position_p_kdl, m_p_eigen);

    pose_frombase[0] = new_cartpos[0] = baseposition.M.data[0];
    pose_frombase[1] = new_cartpos[1] = baseposition.M.data[1];
    pose_frombase[2] = new_cartpos[2] = baseposition.M.data[2];
    pose_frombase[3] = new_cartpos[3] = baseposition.p(0);
    pose_frombase[4] = new_cartpos[4] = baseposition.M.data[3];
    pose_frombase[5] = new_cartpos[5] = baseposition.M.data[4];
    pose_frombase[6] = new_cartpos[6] = baseposition.M.data[5];
    pose_frombase[7] = new_cartpos[7] = baseposition.p(1);
    pose_frombase[8] = new_cartpos[8] = baseposition.M.data[6];
    pose_frombase[9] = new_cartpos[9] = baseposition.M.data[7];
    pose_frombase[10] = new_cartpos[10] = baseposition.M.data[8];
    pose_frombase[11] = new_cartpos[11] = baseposition.p(2);
}

Eigen::Vector3d KukaLwr::get_cur_vel(){
    Eigen::Vector3d vel;
    vel.setZero();

    vel(0) = (new_cartpos[3] - old_cartpos[3])/gettimecycle();
    vel(1) = (new_cartpos[7] - old_cartpos[7])/gettimecycle();
    vel(2) = (new_cartpos[11] - old_cartpos[11])/gettimecycle();
    for(int i = 0; i < 12; i++){
        old_cartpos[i] = new_cartpos[i];
    }
    return vel;
}

void KukaLwr::update_cbf_controller(){
    setReference(cart_command);
    CBF::FloatVector newResourceVector(7);
    for (int i=0; i < LBR_MNJ; i++){
        newResourceVector(i) = jnt_position_act[i];
    }
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    kukaResourceP->set(newResourceVector);
    primitiveControllerP->step();
    updates = kukaResourceP->get() - newResourceVector;
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
}

void KukaLwr::set_joint_command(RobotModeT m=NormalMode){
    if(m == NormalMode){
        double d_updates[7],pupdates[7];
        for(int i = 0; i < 7; i++){
            d_updates[i] = updates(i);
        }
        jlf->get_filtered_value(d_updates,pupdates);
        for(int i = 0; i < 7; i++){
            jnt_command[i] = jnt_position_act[i] + pupdates[i];
            okc_node->jnt_command[i] = jnt_command[i];
        }
    }
    if(m == PsudoGravityCompensation){
        for(int i = 0; i < 7; i++){
            jnt_command[i] = 0.5*(jnt_position_act[i] + okc_node->jnt_position_mea[i]);
            okc_node->jnt_command[i] = jnt_command[i];
        }
    }
        for(int i = 0; i < 12; i++){
            okc_node->new_cartpos[i] = new_cartpos[i];
        }
}
void KukaLwr::set_joint_command(){
        double d_updates[7],pupdates[7];
        for(int i = 0; i < 7; i++){
//            Jnt_rec<<updates(i)<<",";
            d_updates[i] = updates(i);
        }
        jlf->get_filtered_value(d_updates,pupdates);
        for(int i = 0; i < 7; i++){
//            Jnt_rec<<pupdates[i]<<",";
            jnt_command[i] = jnt_position_act[i] + pupdates[i];
            okc_node->jnt_command[i] = jnt_command[i];
        }
//        Jnt_rec<<std::endl;
}


void KukaLwr::no_move(){
    for(int i = 0; i < 7; i++){
        jnt_command[i] = jnt_position_act[i];
        okc_node->jnt_command[i] = jnt_command[i];
    }
}


void KukaLwr::setAxisStiffnessDamping (double* s, double* d){
    okc_node->set_stiffness(s,d);
}


void KukaLwr::update_robot_stiffness(){
    //before using this function, the stiffness parameter should be updated in the ActController
//    setAxisStiffnessDamping(stiff_ctrlpara.axis_stiffness, stiff_ctrlpara.axis_damping);
}

void KukaLwr::update_robot_cp_stiffness(Eigen::VectorXd cps,Eigen::VectorXd cpd){
    double s[6];
    double d[6];
    for(int i = 0; i < 6; i++){
        s[i] = cps[i];
        d[i] = cpd[i];
    }
    okc_node->set_cp_stiffness(s,d);
};

void KukaLwr::update_robot_cp_exttcpft(Eigen::VectorXd ft_kuka){
    double ft_okc[6];
    for(int i = 0; i < 6; i++){
        ft_okc[i] = ft_kuka[i];
    }
    okc_node->set_cp_ExtTcpFT(ft_okc);
}


void KukaLwr::switch2cpcontrol(){
    okc_node->switch_to_cp_impedance();
}

void KukaLwr::switch2jntcontrol(){
    okc_node->switch_to_jnt_impedance();
}

void KukaLwr::request_monitor_mode(){
    okc_node->request_monitor_mode();
}

double KukaLwr::gettimecycle(){
    double t;
    t = okc_node->cycle_time;
    return t;
}


bool KukaLwr::isFinished(){
    //Todo stop several million second
//    okc_sleep_cycletime(okc,robot_id);
    control_period = okc_node->cycle_time;
    usleep(1000*control_period);
    return (primitiveControllerP->finished());
}

//bool KukaLwr::isPseudoConverged(){
//    if (pseudo_converge_count > 100)
//        return true;
//    return false;
//}

void KukaLwr::waitForFinished(){
//    pseudo_converge_count = 0;
    while (!isFinished());
}

void KukaLwr::get_eef_ft(Eigen::Vector3d& f,Eigen::Vector3d& t){
    f[0] = okc_node->ft->x;
    f[1] = okc_node->ft->y;
    f[2] = okc_node->ft->z;
    t[0] = okc_node->ft->c;
    t[1] = okc_node->ft->b;
    t[2] = okc_node->ft->a;
}

void KukaLwr::calibForce(int sampletimes)
{
    Eigen::Vector3d newForce;
    Eigen::Vector3d cf,ct;
    Eigen::Vector3d sum,squaresum,mean,stddev;
    newForce.setZero();
    cf.setZero();
    ct.setZero();
    sum.setZero();
    squaresum.setZero();
    mean.setZero();
    stddev.setZero();

    for (int i=1;i < sampletimes+1; i++){
        get_eef_ft(cf,ct);
        for(int j = 0; j < 3; j++){
            newForce(j) += cf(j) / (-1.0*sampletimes);
            sum(j) += cf(j);
            squaresum(j) += sqr(cf(j));
            mean(j) = sum(j) / (double) i;
            if (i > 1){
                stddev(j) = sqrt ((1.0/(((double)i)-1.0))*(squaresum(j) - (sqr(sum(j))/(double)i)));
            }
        }
        okc_node->sleep_cycle_time();
        fprintf (stderr,"calib: %d\n",i);
    }
    forceCorr = newForce;
    forceCorrStdDev = stddev;
}

void KukaLwr::getTcpFtCalib (Eigen::Vector3d &cf){
    Eigen::Vector3d cfraw, ctraw;
    cfraw.setZero();
    ctraw.setZero();
    get_eef_ft(cfraw,ctraw);
    cf = cfraw + forceCorr;
}



void KukaLwr::get_joint_position_act(){
    for (int i=0;i < 7; i++){
        jnt_position_act[i] = okc_node->jnt_position_act[i];
    }
}

void KukaLwr::get_joint_position_mea(){
    for (int i=0;i < 7; i++){
        jnt_position_mea[i] = okc_node->jnt_position_mea[i];
    }
}

void KukaLwr::get_joint_position_mea(double *jnt){
    for (int i=0;i < 7; i++){
        jnt_position_mea[i] = okc_node->jnt_position_mea[i];
        *(jnt+i) = okc_node->jnt_position_mea[i];
    }
}

void KukaLwr::initSubordinateReference(CBF::FloatVector &f){
    //f[0] = M_PI/2.0;
    f[0] = 0.0;
    f[1] = 0.0;
    f[2] = 0.0;
    //f[3] = 0.0;
    f[3] = M_PI/-2.0;
    f[4] = 0.0;
    f[5] = 0.0;
    f[6] = 0.0;
}

void KukaLwr::initReference(CBF::FloatVector &f){
    if (kuka_right == rn){
        f[0] = initP_x;
        f[1] = initP_y;
        f[2] = initP_z;
        f[3] = 0.0;
        f[4] = -0.5*M_PI;
        f[5] = 0.0;
    }
    if (kuka_left == rn){
        f[0] = -1*initP_x;
        f[1] = initP_y;
        f[2] = initP_z;
        f[3] = 0.0;
        f[4] = M_PI/2;
        f[5] = 0.0;
    }
}

void KukaLwr::initKukaResource (){
    CBF::FloatVector* myResourceVector = new CBF::FloatVector (7);
    get_joint_position_act();
    for (int i=0;i < 7; i++)
        (*myResourceVector) (i) = jnt_position_act[i];
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    kukaResourceP->set((*myResourceVector));
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
}

bool KukaLwr::IsFTAssembled(){
    return false;
}

void KukaLwr::initChains(ToolNameT tn){
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    baseToTool = new Chain();
    worldToTool = new Chain();

/*
    DH representation referecen paper:Visual Estimation and Control of Robot Manipulating Systems(phd thesis)
*/
    toolname = tn;
    
    if (kuka_right == rn){
#ifdef DJALLIL_CONF
    worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.31,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.4,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.39,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,0.0,0.078,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.170))));
#else
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0.0823, 0.897, 0.2975))));
////        //initial orientation kuka base local y then local z
////        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotY(1.047)))));
////        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(0.5236)))));
//        // Orientation identical to ROS model from kuka_cal.xml (fixed axis roll pitch yaw  = x y z)
//        //04,10,2016, the parameters in this code is correct version, currently in ROS, the right arm (left arm--Guillaume defined)
//        //has the different parameters because it comes from the vision calbration result from Christof.
//        // the prameters of kuka_left is computed from the humanoids2015 tactile-based calibration result.
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(M_PI-2.28576)))));
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotY(0.8484)))));
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotX(0.71215)))));


        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0.0871, 0.9011, 0.2981))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(0.8490)))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotY(0.8512)))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotX(0.7227)))));

        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.31,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.4,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.39,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,0.0,0.078,0.0))));
        //0.618 is the length of ati
//         worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,0.0,0.078+0.0618,0.0))));
        //0.0618 = 0.0052+0.017+0.003+0.0286+0.008
        //please comment the next line code if you are doing the robot calibration.
        if(tn == sensing_pole){
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(3.1415)))));
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.170))));
        }
        if(tn == teensy_finger){
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.13605))));
        }
        if(tn == myrmex_sensor){
            //myrmex_sensor tool is assembled
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(3.1415)))));
             std::cout<<"myrmex_sensor in the end-effector on right arm"<<std::endl;
        }

        if(tn == tactool){
            //nothing tool is assembled
             std::cout<<"using tactile tool on right arm"<<std::endl;
        }

#endif
    }
    if (kuka_left == rn){
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(-0.0803, 0.8966, 0.2948))));
        //initial orientation kuka base local y then local z
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotY(-1.047)))));
//        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(2.6180)))));
        // Orientation identical to ROS model from kuka_cal.xml (fixed axis roll pitch yaw  = x y z)
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(2.2734)))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotY(0.8318)))));
        worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotX(-0.7071)))));

        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.31,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.4,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.39,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
        worldToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,0.0,0.078,0.0))));
        //please comment the next line code if you are doing the robot calibration.
        if(tn == sensing_pole){
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(3.1415)))));
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.170))));
        }
        if(tn == teensy_finger){
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.13605))));
        }
        if(tn == myrmex_sensor){
            //myrmex_sensor tool is assembled
            worldToTool->addSegment (Segment(Joint(Joint::None),Frame(Rotation(Rotation::RotZ(3.1415)))));
            std::cout<<"myrmex_sensor in the end-effector on left arm"<<std::endl;
        }
        if(tn == tactool){
            //nothing tool is assembled
             std::cout<<"using tactile tool on left arm"<<std::endl;
        }
    }
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.31,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.4,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.0,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,M_PI_2,0.39,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,-1.0*M_PI_2,0.0,0.0))));
    baseToTool->addSegment (Segment(Joint(Joint::RotZ),Frame(Frame::DH(0.0,0.0,0.078,0.0))));

    worldToToolFkSolver = new ChainFkSolverPos_recursive (*worldToTool);
    baseToToolFkSolver = new ChainFkSolverPos_recursive (*baseToTool);
    worldToToolIkSolver = new ChainIkSolverVel_pinv (*worldToTool);
    worldToToolJacSolver = new ChainJntToJacSolver (*worldToTool);
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
}

void KukaLwr::addSegmentinChain(Eigen::Matrix3d R,Eigen::Vector3d p){
    Vector kdl_p;
    Rotation kdl_R;
    conversions::convert(p,kdl_p);
    conversions::convert(R,kdl_R);
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    delete worldToToolFkSolver;
    delete  worldToToolJacSolver;
    worldToTool->addSegment (Segment(Joint(Joint::None),Frame(kdl_R,kdl_p)));
    worldToToolFkSolver = new ChainFkSolverPos_recursive (*worldToTool);
    worldToToolJacSolver = new ChainJntToJacSolver (*worldToTool);
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
}

void KukaLwr::backKukaChain(ToolNameT tn){
    if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not lock mutex");
        exit (EXIT_FAILURE);
    }
    delete baseToTool;
    delete worldToTool;
    delete worldToToolFkSolver;
    delete  worldToToolJacSolver;
    delete baseToToolFkSolver;
    delete worldToToolIkSolver;
    std::cout<<"in back chain sub-routine, rn and tn are "<<rn<<","<<tn<<std::endl;
    if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
        perror ("CbfPlanner: setReference(): could not unlock mutex");
        exit (EXIT_FAILURE);
    }
    initChains(tn);
}


void KukaLwr::initCbf (ControllerT ctrltype){
    if (kuka_left == rn){
        if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
            perror ("CbfPlanner: setReference(): could not lock mutex");
            exit (EXIT_FAILURE);
        }
        boost::shared_ptr<KDL::Chain> chain (new KDL::Chain(*worldToTool));
        currentTaskReferenceP = CBF::DummyReferencePtr (new CBF::DummyReference(1,6));
        currentTaskTargetP = CBF::DummyReferencePtr (new CBF::DummyReference(1,6));
        kukaResourceP = CBF::DummyResourcePtr (new CBF::DummyResource(7));
        currentSubordinateTaskReferenceP = CBF::DummyReferencePtr (new CBF::DummyReference(1,7));
        try {
            CBF::FloatVector myReferenceVector(6);
            CBF::FloatVector mySubordinateReferenceVector (7);

            initReference(myReferenceVector);
            currentTaskReferenceP->set_reference(myReferenceVector);
            currentTaskTargetP->set_reference(myReferenceVector);
            initSubordinateReference (mySubordinateReferenceVector);
            currentSubordinateTaskReferenceP->set_reference (mySubordinateReferenceVector);

            double limit = M_PI * (165.0 / 180.0);

            CBF::FloatVector vmins = CBF::FloatVector::Constant(7, -limit);
            CBF::FloatVector vmaxs = CBF::FloatVector::Constant(7,  limit);
//            vmins(5) = M_PI* (-150.0 / 180.0);
//            vmaxs(5) = M_PI* (150.0/ 180.0);
            vmins(1) = M_PI* (-100.0 / 180.0);
            vmaxs(1) = M_PI* (100.0/ 180.0);
            vmins(3) = M_PI* (-110.0 / 180.0);
            vmaxs(3) = M_PI* (110.0/ 180.0);
            vmins(5) = M_PI* (-110.0 / 180.0);
            vmaxs(5) = M_PI* (110.0/ 180.0);

            std::vector<ConvergenceCriterionPtr> vConvergenceCriteria = boost::assign::list_of
                    (ConvergenceCriterionPtr(new TaskSpaceDistanceThreshold(0.001)))
                    (ConvergenceCriterionPtr(new ResourceStepNormThreshold(0.001)));

            // create the subordinate controller
//             subordinateControllerP = SubordinateControllerPtr(new CBF::SubordinateController(
//                                                                   1.0,
//                                                                   vConvergenceCriteria,
//                                                                   currentSubordinateTaskReferenceP,
//                                                                   PotentialPtr(new WuPotential(vmins, vmaxs, 0.01)),
//                                                                   SensorTransformPtr(new CBF::IdentitySensorTransform(7)),
//                                                                   EffectorTransformPtr(new CBF::GenericEffectorTransform(7,7)),
//                                                                   std::vector<SubordinateControllerPtr>(),
//                                                                   CombinationStrategyPtr(new AddingStrategy())));

            std::vector<CBF::SubordinateControllerPtr> vSubOrdinateControllers;
//            vSubOrdinateControllers.push_back(subordinateControllerP);
            if(ctrltype == WuPotential_Ctrl){
                subordinateControllerP = SubordinateControllerPtr(new CBF::SubordinateController(
                                                                  1.0,
                                                                  vConvergenceCriteria,
                                                                  currentSubordinateTaskReferenceP,
//                                                                   PotentialPtr(new WuPotential(vmins, vmaxs, 0.2,0.01)),
                                                                  PotentialPtr(new WuPotential(vmins,vmaxs,0.2,0.01)),
                                                                  SensorTransformPtr(new CBF::IdentitySensorTransform(7)),
                                                                  EffectorTransformPtr(new CBF::GenericEffectorTransform(7,7)),
                                                                  std::vector<SubordinateControllerPtr>(),
                                                                  CombinationStrategyPtr(new AddingStrategy())));
                vSubOrdinateControllers.push_back(subordinateControllerP);
            }
            if(ctrltype == JLPotential_Ctrl){
                subordinateControllerP = SubordinateControllerPtr(new CBF::SubordinateController(
                                                                  1.0,
                                                                  vConvergenceCriteria,
                                                                  currentSubordinateTaskReferenceP,
//                                                                   PotentialPtr(new WuPotential(vmins, vmaxs, 0.2,0.01)),
                                                                  PotentialPtr(new JLPotential(vmins,vmaxs,0.2,5)),
                                                                  SensorTransformPtr(new CBF::IdentitySensorTransform(7)),
                                                                  EffectorTransformPtr(new CBF::GenericEffectorTransform(7,7)),
                                                                  std::vector<SubordinateControllerPtr>(),
                                                                  CombinationStrategyPtr(new AddingStrategy())));

                vSubOrdinateControllers.push_back(subordinateControllerP);
            }
            if(ctrltype == IMPotential_Ctrl){
                subordinateControllerP = SubordinateControllerPtr(new CBF::SubordinateController(
                                                                  1.0,
                                                                  vConvergenceCriteria,
                                                                  currentSubordinateTaskReferenceP,
//                                                                   PotentialPtr(new WuPotential(vmins, vmaxs, 0.2,0.01)),
                                                                  PotentialPtr(new IMPotential(0.2,0.1)),
                                                                  SensorTransformPtr(new CBF::IdentitySensorTransform(7)),
                                                                  EffectorTransformPtr(new CBF::GenericEffectorTransform(7,7)),
                                                                  std::vector<SubordinateControllerPtr>(),
                                                                  CombinationStrategyPtr(new AddingStrategy())));
                vSubOrdinateControllers.push_back(subordinateControllerP);
            }

            // create the composite potential
            xyzSquarePotential = CBF::SquarePotentialPtr(new CBF::SquarePotential(3,0.008));
            xyzSquarePotential->set_max_gradient_step_norm (99.0);
            CBF::AxisAnglePotentialPtr myAxisAnglePotentialP = CBF::AxisAnglePotentialPtr (new CBF::AxisAnglePotential(0.04));
            myAxisAnglePotentialP->set_max_gradient_step_norm (99.0);
            std::vector<CBF::PotentialPtr> myVectorOfPotentials;
            myVectorOfPotentials.push_back(xyzSquarePotential);
            myVectorOfPotentials.push_back(myAxisAnglePotentialP);
            CBF::CompositePotentialPtr myCompositePotentialP = CBF::CompositePotentialPtr (new CBF::CompositePotential(myVectorOfPotentials));
            // create the composite sensor transform
            std::vector<SensorTransformPtr> myVectorOfSensorTransforms = boost::assign::list_of
                    (CBF::SensorTransformPtr(new CBF::KDLChainPositionSensorTransform(chain)))
                    (CBF::SensorTransformPtr(new CBF::KDLChainAxisAngleSensorTransform(chain)));
            CBF::CompositeSensorTransformPtr myCompositeSensorTransformP =
                    CBF::CompositeSensorTransformPtr(new CBF::CompositeSensorTransform(myVectorOfSensorTransforms));

            // create final controller
            primitiveControllerP = PrimitiveControllerPtr(new CBF::PrimitiveController(
                                                              1.0,
                                                              vConvergenceCriteria,
                                                              currentTaskReferenceP,
                                                              myCompositePotentialP,
                                                              myCompositeSensorTransformP,
                                                              EffectorTransformPtr(new CBF::DampedGenericEffectorTransform(6,7, 0.001)),
                                                              vSubOrdinateControllers,
                                                              CombinationStrategyPtr(new AddingStrategy()),
                                                              kukaResourceP));
        }
        catch(...){
            std::cerr << "initCbf(): error: could not initialize CBF!" << std::endl;
            exit (EXIT_FAILURE);
        }
        if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
            perror ("CbfPlanner: initCbf(): could not unlock mutex");
            exit (EXIT_FAILURE);
        }
        //std::cout << "Distance Threshold " << primitiveControllerP->potential()->m_DistanceThreshold << std::endl;
        //primitiveControllerP->potential()->m_DistanceThreshold = 10000000.0;
        //subordinateControllerP->potential()->m_DistanceThreshold = 999999.9;
        //    noCbf = false;
    }


    if (kuka_right == rn){
        if (0 != pthread_mutex_lock (&primitiveControllerMutex)){
            perror ("CbfPlanner: initCbf(): could not lock mutex");
            exit (EXIT_FAILURE);
        }
        boost::shared_ptr<KDL::Chain> chain (new KDL::Chain(*worldToTool));
        currentTaskReferenceP = CBF::DummyReferencePtr (new CBF::DummyReference(1,6));
        currentTaskTargetP = CBF::DummyReferencePtr (new CBF::DummyReference(1,6));
        kukaResourceP = CBF::DummyResourcePtr (new CBF::DummyResource(7));
        currentSubordinateTaskReferenceP = CBF::DummyReferencePtr (new CBF::DummyReference(1,7));
        try {
            CBF::FloatVector myReferenceVector(6);
            CBF::FloatVector mySubordinateReferenceVector (7);

            initReference(myReferenceVector);
            currentTaskReferenceP->set_reference(myReferenceVector);
            currentTaskTargetP->set_reference(myReferenceVector);
            initSubordinateReference (mySubordinateReferenceVector);
            currentSubordinateTaskReferenceP->set_reference (mySubordinateReferenceVector);

            double limit = M_PI * (165.0 / 180.0);

            CBF::FloatVector vmins = CBF::FloatVector::Constant(7, -limit);
            CBF::FloatVector vmaxs = CBF::FloatVector::Constant(7,  limit);
            vmins(5) = M_PI* (-150.0 / 180.0);
            vmaxs(5) = M_PI* (150.0/ 180.0);

            std::vector<ConvergenceCriterionPtr> vConvergenceCriteria = boost::assign::list_of
                    (ConvergenceCriterionPtr(new TaskSpaceDistanceThreshold(0.001)))
                    (ConvergenceCriterionPtr(new ResourceStepNormThreshold(0.001)));

            // create the subordinate controller
            subordinateControllerP = SubordinateControllerPtr(new CBF::SubordinateController(
                                                                  1.0,
                                                                  vConvergenceCriteria,
                                                                  currentSubordinateTaskReferenceP,
                                                                  PotentialPtr(new WuPotential(vmins, vmaxs, 0.01)),
                                                                  SensorTransformPtr(new CBF::IdentitySensorTransform(7)),
                                                                  EffectorTransformPtr(new CBF::GenericEffectorTransform(7,7)),
                                                                  std::vector<SubordinateControllerPtr>(),
                                                                  CombinationStrategyPtr(new AddingStrategy())));

            std::vector<CBF::SubordinateControllerPtr> vSubOrdinateControllers;
            // 		vSubOrdinateControllers.push_back(subordinateControllerP);

            // create the composite potential
            xyzSquarePotential = CBF::SquarePotentialPtr(new CBF::SquarePotential(3,0.008));
            xyzSquarePotential->set_max_gradient_step_norm (99.0);
            CBF::AxisAnglePotentialPtr myAxisAnglePotentialP = CBF::AxisAnglePotentialPtr (new CBF::AxisAnglePotential(0.04));
            myAxisAnglePotentialP->set_max_gradient_step_norm (99.0);
            std::vector<CBF::PotentialPtr> myVectorOfPotentials;
            myVectorOfPotentials.push_back(xyzSquarePotential);
            myVectorOfPotentials.push_back(myAxisAnglePotentialP);
            CBF::CompositePotentialPtr myCompositePotentialP = CBF::CompositePotentialPtr (new CBF::CompositePotential(myVectorOfPotentials));
            // create the composite sensor transform
            std::vector<SensorTransformPtr> myVectorOfSensorTransforms = boost::assign::list_of
                    (CBF::SensorTransformPtr(new CBF::KDLChainPositionSensorTransform(chain)))
                    (CBF::SensorTransformPtr(new CBF::KDLChainAxisAngleSensorTransform(chain)));
            CBF::CompositeSensorTransformPtr myCompositeSensorTransformP =
                    CBF::CompositeSensorTransformPtr(new CBF::CompositeSensorTransform(myVectorOfSensorTransforms));

            // create final controller
            primitiveControllerP = PrimitiveControllerPtr(new CBF::PrimitiveController(
                                                              1.0,
                                                              vConvergenceCriteria,
                                                              currentTaskReferenceP,
                                                              myCompositePotentialP,
                                                              myCompositeSensorTransformP,
                                                              EffectorTransformPtr(new CBF::DampedGenericEffectorTransform(6,7, 0.001)),
                                                              vSubOrdinateControllers,
                                                              CombinationStrategyPtr(new AddingStrategy()),
                                                              kukaResourceP));
        }
        catch(...){
            std::cerr << "initCbf(): error: could not initialize CBF!" << std::endl;
            exit (EXIT_FAILURE);
        }
        if (0 != pthread_mutex_unlock (&primitiveControllerMutex)){
            perror ("CbfPlanner: initCbf(): could not unlock mutex");
            exit (EXIT_FAILURE);
        }
        //std::cout << "Distance Threshold " << primitiveControllerP->potential()->m_DistanceThreshold << std::endl;
        //primitiveControllerP->potential()->m_DistanceThreshold = 10000000.0;
        //subordinateControllerP->potential()->m_DistanceThreshold = 999999.9;
        //    noCbf = false;
    }
}


KukaLwr::KukaLwr(RobotNameT robotname, ComOkc& com, ToolNameT tn, ControllerT ctrltype)
{
    if (0 != pthread_mutex_init(&primitiveControllerMutex,NULL)){
        perror ("CbfPlanner: could not initialize Mutex");
        exit (EXIT_FAILURE);
    }
    rn = robotname;
    okc_node = &com;
    initChains(tn);
    initCbf(ctrltype);
    control_period = 4;
    Jac_kdl = KDL::Jacobian (7);
    updates.resize(7);
    for(int i = 0; i < 7; i++){
        updates(i) = 0.0;
    }
    jlf = new JntLimitFilter(okc_node->cycle_time);
    forceCorr.setZero();
    forceCorrStdDev.setZero();
//    Jnt_rec.open("/tmp/data/vel.txt");
}
