/*
    This file is part of VTFS--Visuo-Tactile-Force-Servoing.

    VTFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    VTFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CBF.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/
#ifndef COMRSB_H
#define COMRSB_H
#include "com.h"
#include <Eigen/Dense>

// #define SEND_WITH_RSB_
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/Listener.h>

#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include "ComModule/mydata.pb.h"

#include <string>
#include <map>
#include <mutex>
#include "UtilModule/msgcontenttype.h"

using namespace rsb;
// The generated protocol buffer class is in this namespace.
using namespace manip;

enum RsbDataType{
    FiducialMarkerFeature,
    MarkerPointsFeature,
    LeftKukaEff,
    RightKukaEff,
    LeftMyrmex,
    RightMyrmex,
    LeftTacPointClouds,
    RightTacPointClouds,
};

class ComRSB : public Com
{
public:
    ComRSB();
    bool tactile_receive(myrmex_msg& msg, std::string tacpart);
    bool fiducialmarker_receive(markered_object_msg & msg);
    bool markerpoints_receive(dirt_points_msg& msg);
    void add_msg(RsbDataType& rdt);
    void kuka_msg_send(kuka_msg& msg, std::string tacpart);
    void tacpcs_msg_send(tacpcs_msg& msg, std::string tacpart);
    void register_external(const std::string &scope, boost::function<void(boost::shared_ptr<std::string>)> &fun);
private:
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<TacMsg> >* converterTac;
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<VisMsg> >* converterVis;
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<MarkerPointsMsg> > *converterMarkerPoints;
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<PCsMsg> >* converterPCs;
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<RobotMsg> >* converterRobot;

    std::map<std::string,Informer<PCsMsg>::Ptr> inf_pcs;
    std::map<std::string,Informer<RobotMsg>::Ptr> inf_robot;
    std::map<std::string,Scope *> scope_tac;
    Scope* scope_vis;
    Scope* scope_Markerpoints;
    std::map<std::string,ListenerPtr> lis_tac;
    ListenerPtr lis_vis,lis_Markerpoints;
    std::map<std::string,ListenerPtr> lis_gui;
    Factory* factory;
};

#endif // COMRSB_H
