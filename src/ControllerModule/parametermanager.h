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
    along with VTFS.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/
#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

#include "CtrlParam.h"
#include "TaskModule/task.h"
#include "UtilModule/RebaType.h"
#include <map>
//load the parameter which are stored in xml file
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "boost/foreach.hpp"
using boost::property_tree::ptree;

class ParameterManager
{
public:
    ParameterManager();
    ParameterManager(const std::string,TacSensorType t=MIDtip);
    std::map<TACTaskNameT, taskctrlpara> tac_task_ctrl_param;
    std::map<VISTaskNameT, taskctrlpara> vis_task_ctrl_param;
    std::map<PROTaskNameT, taskctrlpara> pro_task_ctrl_param;
    std::map<FORCETaskNameT, taskctrlpara> force_task_ctrl_param;
    //    static ctrlpara o_ctrlpara;
    stiffpara stiff_ctrlpara;
private:
    void loadCtrlParam(std::string);
    void load(TACTaskNameT,ptree);
    void load(PROTaskNameT,ptree);
    void load(FORCETaskNameT fnt,ptree pt);
    std::map<TACTaskNameT, std::string> tac_map_task_name;
    std::map<VISTaskNameT, std::string> vis_map_task_name;
    std::map<PROTaskNameT, std::string> pro_map_task_name;
    std::map<FORCETaskNameT, std::string> force_map_task_name;
    std::map<int,std::string> map_name_dim;
    std::map<int,std::string> map_name_dim2;
    std::map<int,std::string> map_name_dim3;
    TacSensorType tst;
};

#endif // PARAMETERMANAGER_H
