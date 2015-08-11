#include "comrsb.h"
#include <UtilModule/kdl_to_eigen.h>
#include <kdl/frames.hpp>

std::mutex mutex_lefttac,mutex_righttac,mutex_vis,\
        mutex_pcs,mutex_markerpoints,mutex_gui;
double right_position3d[3],right_CPnormal[3],\
        right_position3d_Object[3],right_CPnormal_Object[3];
double left_position3d[3],left_CPnormal[3],\
        left_position3d_Object[3],left_CPnormal_Object[3];

kuka_msg left_kuka_msg, right_kuka_msg;
tacpcs_msg  left_tacpcs_msg, right_tacpcs_msg;
myrmex_msg left_myrmex, right_myrmex;
markered_object_msg obj;
dirt_points_msg dirt_ps;

bool is_get_left_tac_msg = false;
bool is_get_right_tac_msg = false;
bool is_get_vis_msg = false;
bool is_get_dirtps_msg = false;



void handle_lefttac(boost::shared_ptr<TacMsg> data) {
    mutex_lefttac.lock();
    left_myrmex.cogx = data->cpposition2d(0);
    left_myrmex.cogy = data->cpposition2d(1);
    left_myrmex.contactnum = data->contactnum();
    left_myrmex.contactflag = data->contactflag();
    left_myrmex.cf = data->contactforce();
    left_myrmex.lineorien = data->contactorien();
//    std::cout<<"contact info "<< left_cog_x <<","<<left_cog_y<<","<<left_contactflag<<","<<left_cf<<std::endl;
    is_get_left_tac_msg = true;
    mutex_lefttac.unlock();
}

void handle_righttac(boost::shared_ptr<TacMsg> data) {
    mutex_righttac.lock();
    right_myrmex.cogx = data->cpposition2d(0);
    right_myrmex.cogy = data->cpposition2d(1);
    right_myrmex.contactnum = data->contactnum();
    right_myrmex.contactflag = data->contactflag();
    right_myrmex.cf = data->contactforce();
    right_myrmex.lineorien = data->contactorien();
    is_get_right_tac_msg = true;
    mutex_righttac.unlock();
}

void handle_vis(boost::shared_ptr<VisMsg> data) {
    KDL::Rotation kdl_obj_orien;
    mutex_vis.lock();
    if(data->objnum() > 0){
        obj.p(0) = data->objposition(0);
        obj.p(1) = data->objposition(1);
        obj.p(2) = data->objposition(2);
        obj.roll = data->objorien(0);
        obj.pitch = data->objorien(1);
        obj.yaw = data->objorien(2);
//        std::cout<<"position: "<<position(0)<<","<<position(1)<<","<<position(2)<<std::endl;
        kdl_obj_orien = kdl_obj_orien.RPY(obj.roll,obj.pitch,obj.yaw);
        conversions::convert(kdl_obj_orien,obj.orientation);
        obj.marker_num = data->objnum();
    }
    else{
        obj.p.setZero();
        obj.orientation.setIdentity();
        obj.marker_num = 0;
    }
    is_get_vis_msg = true;
    mutex_vis.unlock();
}

void handle_MarkerPoints(boost::shared_ptr<MarkerPointsMsg> data) {
    mutex_markerpoints.lock();
//    std::cout<<"data I got"<<std::endl;
    if(data->markernum() > 0){
        dirt_ps.markerpoints_num = data->markernum();
//        std::cout<<"marker num: "<<markerpoints_num<<std::endl;
//        std::cout<<"marker infor ";
        for(int i = 0; i < 3; i++){
            dirt_ps.markerpoints_pos(i) = data->position3d(i);
            dirt_ps.markerpoints_nv(i) = data->normalvector(i);
//            std::cout<<markerpoints_pos[i]<<",";
        }
//        std::cout<<std::endl;

    }
    is_get_dirtps_msg = true;
    mutex_markerpoints.unlock();
}


void ComRSB::kuka_msg_send(kuka_msg &msg, std::string tacpart){
    Informer<RobotMsg>::DataPtr robot_msg(new RobotMsg());
    for(int i = 0 ; i < 3; i++){
        robot_msg->add_position(msg.p(i));
    }
    for(int i = 0 ; i < 3; i++){
        for(int j = 0; j < 3; j++){
            robot_msg->add_orien(msg.o(i,j));
        }
    }
    //add force/torque estimation value
    for(int i = 0; i < 6; i++)
        robot_msg->add_ft(msg.ft(i));
    inf_robot[tacpart]->publish(robot_msg);
}

void ComRSB::tacpcs_msg_send(tacpcs_msg& msg, std::string tacpart){
    Informer<PCsMsg>::DataPtr pcs_msg(new PCsMsg());
    for(int i = 0 ; i < 3; i++){
        pcs_msg->add_cpposition3d(msg.position3d[i]);
        pcs_msg->add_cppositionobject3d(msg.position3d_Object[i]);
        pcs_msg->add_cpnormalvector(msg.CPnormal[i]);
        pcs_msg->add_cpnormalobjectvector(msg.CPnormal_Object[i]);
    }
    inf_pcs[tacpart]->publish(pcs_msg);
}


bool ComRSB::tactile_receive(myrmex_msg& msg, std::string tacpart){
//    switch (tacpart){
//    case "leftmyrmex":
//        mutex_lefttac.lock();
//        msg = left_myrmex;
//        mutex_lefttac.unlock();
//        if(is_get_left_tac_msg == true){
//            is_get_left_tac_msg = false;
//            return true;
//        }else{
//            return false;
//        }
//        break;
//    case "rightmyrmex":
//        mutex_righttac.lock();
//        msg = right_myrmex;
//        mutex_righttac.unlock();
//        if(is_get_right_tac_msg == true){
//            is_get_right_tac_msg = false;
//            return true;
//        }else{
//            return false;
//        }
//        break;
//    default:
//        std::cout<<"the wrong robot tact part is used"<<std::endl;
//        return false;
//        break;

//    }
    if(tacpart == "leftmyrmex"){
        mutex_lefttac.lock();
        msg = left_myrmex;
        mutex_lefttac.unlock();
        if(is_get_left_tac_msg == true){
            is_get_left_tac_msg = false;
            return true;
        }else{
            return false;
        }
    }
    if(tacpart == "rightmyrmex"){
        mutex_righttac.lock();
        msg = right_myrmex;
        mutex_righttac.unlock();
        if(is_get_right_tac_msg == true){
            is_get_right_tac_msg = false;
            return true;
        }else{
            return false;
        }
    }

}

bool ComRSB::fiducialmarker_receive(markered_object_msg & msg){
    mutex_vis.lock();
    msg = obj;
    mutex_vis.unlock();
    if(is_get_vis_msg == true){
        is_get_vis_msg = false;
        return true;
    }
    else{
        return false;
    }

}
bool ComRSB::markerpoints_receive(dirt_points_msg& msg){
    mutex_markerpoints.lock();
    msg = dirt_ps;
    mutex_markerpoints.unlock();
    if(is_get_dirtps_msg == true){
        is_get_dirtps_msg = false;
        return true;
    }
    else{
        return false;
    }
}

void ComRSB::add_msg(RsbDataType& rdt){
    switch (rdt){
    case FiducialMarkerFeature:
        converterVis = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<VisMsg> >
                (new rsb::converter::ProtocolBufferConverter<VisMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterVis);
        scope_vis = new Scope("/VisMsg");
        lis_vis = factory->createListener(*scope_vis);
        lis_vis->addHandler(HandlerPtr(new DataFunctionHandler<VisMsg>(handle_vis)));
        break;
    case MarkerPointsFeature:
        converterMarkerPoints = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<MarkerPointsMsg> >
                 (new rsb::converter::ProtocolBufferConverter<MarkerPointsMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterMarkerPoints);
        scope_Markerpoints = new Scope("/MarkerPointsMsg");
        lis_Markerpoints = factory->createListener(*scope_Markerpoints);
        lis_Markerpoints->addHandler(HandlerPtr(new DataFunctionHandler<MarkerPointsMsg>(handle_MarkerPoints)));
        break;
    case LeftKukaEff:
        converterRobot = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<RobotMsg> >
                (new rsb::converter::ProtocolBufferConverter<RobotMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterRobot);
        inf_robot["leftkuka"] = factory->createInformer<RobotMsg> ("/left/RobotMsg");
        break;
    case RightKukaEff:
        converterRobot = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<RobotMsg> >
                (new rsb::converter::ProtocolBufferConverter<RobotMsg>());
//        rsb::converter::converterRepository<std::string>()->registerConverter(*converterRobot);
        inf_robot["rightkuka"] = factory->createInformer<RobotMsg> ("/right/RobotMsg");
        break;
    case LeftMyrmex:
        converterTac = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<TacMsg> > \
                (new rsb::converter::ProtocolBufferConverter<TacMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterTac);
        scope_tac["leftmyrmex"] = new Scope("/left/TacMsg");
        lis_tac["leftmyrmex"] = factory->createListener(*scope_tac["leftmyrmex"]);
        lis_tac["leftmyrmex"]->addHandler(HandlerPtr(new DataFunctionHandler<TacMsg>(handle_lefttac)));
        break;
    case RightMyrmex:
        converterTac = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<TacMsg> > \
                (new rsb::converter::ProtocolBufferConverter<TacMsg>());
//        rsb::converter::converterRepository<std::string>()->registerConverter(*converterTac);
        scope_tac["rightmyrmex"] = new Scope("/right/TacMsg");
        lis_tac["rightmyrmex"] = factory->createListener(*scope_tac["rightmyrmex"]);
        lis_tac["rightmyrmex"]->addHandler(HandlerPtr(new DataFunctionHandler<TacMsg>(handle_righttac)));
        break;
    case LeftTacPointClouds:
        converterPCs = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<PCsMsg> >
                (new rsb::converter::ProtocolBufferConverter<PCsMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterPCs);
        inf_pcs["leftmyrmex"] = factory->createInformer<PCsMsg> ("/left/PCsMsg");
        break;
    case RightTacPointClouds:
        converterPCs = new boost::shared_ptr< rsb::converter::ProtocolBufferConverter<PCsMsg> >
                (new rsb::converter::ProtocolBufferConverter<PCsMsg>());
        rsb::converter::converterRepository<std::string>()->registerConverter(*converterPCs);
        inf_pcs["rightmyrmex"] = factory->createInformer<PCsMsg> ("/right/PCsMsg");
        break;
    default:
        std::cout<<"you are adding the wrong msg"<<std::endl;
    }
}

void ComRSB::register_external(const std::string &scope, boost::function<void(boost::shared_ptr<std::string>)> &fun) {
    lis_gui[scope] = factory->createListener(*(new Scope(scope)));
    lis_gui[scope]->addHandler(HandlerPtr(new DataFunctionHandler<std::string>(fun)));
}

ComRSB::ComRSB()
{
    factory = &(getFactory());
}
