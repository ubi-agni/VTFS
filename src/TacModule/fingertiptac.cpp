#include "fingertiptac.h"
#include <algorithm>
#include <fstream>
#include <string>

Eigen::Matrix3d GenerateLF(Eigen::Vector3d nv){
    //normalize the z vector
    Eigen::Vector3d x_nm,y_nm,z_nm;
    Eigen::Vector3d x,y;
    Eigen::Matrix3d RM;
    x.setZero();
    y.setZero();
    x_nm.setZero();
    y_nm.setZero();
    z_nm.setZero();
    RM.setZero();

    z_nm = nv/nv.norm();
    y(0) = z_nm(1);
    y(1) = -1*z_nm(0);
    y(2) = 0;
    y_nm = y/y.norm();
    x_nm = y_nm.cross(z_nm);

    for(int i = 0; i < 3; i++){
        RM(i,0) = x_nm(i);
    }
    for(int i = 0; i < 3; i++){
        RM(i,1) = y_nm(i);
    }
    for(int i = 0; i < 3; i++){
        RM(i,2) = z_nm(i);
    }
    return RM;
}

void FingertipTac::get_slope(Eigen::Vector3d s){
    slope = s;
}

void FingertipTac::slope_clear(){
    slope.setZero();
}

Eigen::MatrixXd FingertipTac::readMatrix(const char *fn){
    int cols = 0, rows = 0;
    double buff[1000000];

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(fn);
    while (! infile.eof())
    {
        std::string line;
        getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();
//    std::cout<<"row cols are "<<rows<<","<<cols<<std::endl;
    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];
//    std::cout<<"result are "<<result<<std::endl;

    return result;
}

ContactAreaT FingertipTac::WhichArea(int act_id){
    if((act_id == 0)||(act_id == 3)||(act_id == 8)||(act_id == 11)){
        //sensor frame z axis is defined is the same with fingertip frame -y axis.
        return Area1;
    }
    if((act_id == 1)||(act_id == 2)||(act_id == 5)){
        //sensor frame z axis is defined is the same with fingertip frame x axis.
        return Area2;
    }
    if((act_id == 6)||(act_id == 9)||(act_id == 10)){
        //sensor frame z axis is defined is the same with fingertip frame -x axis.
        return Area3;
    }
    if((act_id == 4)||(act_id == 7)){
        //sensor frame z axis is defined is the same with fingertip frame z axis.
        return Area4;
    }
    else
    {
        std::cout<<"area detector error"<<std::endl;
        return NoContact;
    }
}

Eigen::Vector3d FingertipTac::get_Center_nv(int areanum){
    Eigen::Vector3d nv2,sum_nv2;
    nv2.setZero();
    sum_nv2.setZero();
    switch (areanum){
    case 1:{
        sum_nv2 += data.fingertip_tac_nv.at(0)+data.fingertip_tac_nv.at(3)\
                +data.fingertip_tac_nv.at(8)+data.fingertip_tac_nv.at(11);
        nv2 = sum_nv2 / 4.0;
        break;
    }
    case 2:{
        sum_nv2 += data.fingertip_tac_nv.at(1)+data.fingertip_tac_nv.at(2)\
                +data.fingertip_tac_nv.at(5);
        nv2 = sum_nv2 / 3.0;
        break;
    }
    case 3:{
        sum_nv2 += data.fingertip_tac_nv.at(6)+data.fingertip_tac_nv.at(9)\
                +data.fingertip_tac_nv.at(10);
        nv2 = sum_nv2 / 3.0;
        break;
    }
    case 4:{
        sum_nv2 += data.fingertip_tac_nv.at(4)+data.fingertip_tac_nv.at(7);
        nv2 = sum_nv2 / 2.0;
        break;
    }
    default:{
        std::cout<<"you are input the wrong desired contact area,using area I as the default"<<std::endl;
        sum_nv2 += data.fingertip_tac_nv.at(0)+data.fingertip_tac_nv.at(3)\
                +data.fingertip_tac_nv.at(8)+data.fingertip_tac_nv.at(11);
        nv2 = sum_nv2 / 4.0;
        break;
    }
    }
        return nv2;
}
Eigen::Vector3d FingertipTac::get_Center_position(int areanum){
    Eigen::Vector3d position,sum_position;
    position.setZero();
    sum_position.setZero();
    switch (areanum){
    case 1:{
        sum_position += data.fingertip_tac_position.at(0)+data.fingertip_tac_position.at(3)\
                +data.fingertip_tac_position.at(8)+data.fingertip_tac_position.at(11);
        position = sum_position / 4.0;
        break;
    }
    case 2:{
        sum_position += data.fingertip_tac_position.at(1)+data.fingertip_tac_position.at(2)\
                +data.fingertip_tac_position.at(5);
        position = sum_position / 3.0;
        break;
    }
    case 3:{
        sum_position += data.fingertip_tac_position.at(6)+data.fingertip_tac_position.at(9)\
                +data.fingertip_tac_position.at(10);
        position = sum_position / 3.0;
        break;
    }
    case 4:{
        sum_position += data.fingertip_tac_position.at(4)+data.fingertip_tac_position.at(7);
        position = sum_position / 2.0;
        break;
    }
    default:{
        std::cout<<"you are input the wrong desired contact area,using area I as the default"<<std::endl;
        sum_position += data.fingertip_tac_position.at(0)+data.fingertip_tac_position.at(3)\
                +data.fingertip_tac_position.at(8)+data.fingertip_tac_position.at(11);
        position = sum_position / 4.0;
        break;
    }
    }
        return position;
}


FingertipTac::FingertipTac(int num)
{
    data.tac_num = num;
//    data.fingertip_tac_position.resize(num);
//    data.fingertip_tac_nv.resize(num);
//    data.fingertip_tac_pressure.resize(num);
    init_taxelmap();
    pos.setZero();
    nv.setZero();
    slope.setZero();
    pressure = 0.0;
    act_taxel_num = 0;
    contactarea = NoContact;
}

void FingertipTac::init_taxelmap(){
    Eigen::MatrixXd nv_mat,position_mat;
    nv_mat.setZero(12,3);
    position_mat.setZero(12,3);

    nv_mat = readMatrix("fingertip_taxel_nv.txt");
    position_mat = readMatrix("fingertip_taxel_position.txt");

    for(int i = 0; i < 12; i++){
        data.fingertip_tac_nv.push_back(nv_mat.row(i).transpose());
        std::cout<<"nv row "<<position_mat.row(i)<<std::endl;
        data.fingertip_tac_position.push_back(position_mat.row(i).transpose());
    }
//    std::cout<<"tacindex are "<<data.fingertip_tac_position.at(2)<<std::endl;
}

void FingertipTac::est_cop(tac_data t_data){
    double press_sum;
    Eigen::Vector3d pos_sum;
    pos_sum.setZero();
    get_act_taxelId(t_data);
    act_taxel_num = act_Ids.size();
    if(act_taxel_num > 0){
//        std::cout<<"p actived taxel num is "<<act_taxel_num<<std::endl;
        press.setZero(act_taxel_num);
        weighted_press.setZero(act_taxel_num);
//        std::cout<<"p pressure are ";
        for(int i = 0; i < act_taxel_num; i++){
            press[i] = data.fingertip_tac_pressure[act_Ids[i]];
//            std::cout<<data.fingertip_tac_pressure[act_Ids[i]]<<",";
        }
//        std::cout<<std::endl;
        press_sum = press.sum();
        weighted_press = press/press_sum;
//        std::cout<<"p pressure weight are ";
//        for(int i = 0; i < act_taxel_num; i++){
//            std::cout<<weighted_press[i]<<",";
//        }
//        std::cout<<std::endl;
//        std::cout<<"p taxel id ";
//        for(int i = 0; i < act_taxel_num; i++){
//            std::cout<<act_Ids[i]<<",";
//        }
//        std::cout<<std::endl;


        for(int i = 0; i < act_taxel_num; i++){
//            std::cout<<"p taxel position "<<act_Ids[i]<<" ";
            pos_sum += weighted_press[i] * data.fingertip_tac_position[act_Ids[i]];
//            std::cout<<data.fingertip_tac_position[act_Ids[i]](0)<<","\
//                           <<data.fingertip_tac_position[act_Ids[i]](1)<<","\
//                            <<data.fingertip_tac_position[act_Ids[i]](2)<<std::endl;
        }
//        std::cout<<std::endl;
        pos = pos_sum;
//        std::cout<<"pos is "<<pos(0)<<","<<pos(1)<<","<<pos(2)<<std::endl;
    }
    else{
        pos.setZero();
    }
}

void FingertipTac::est_nv(tac_data t_data){
    double press_sum;
    Eigen::Vector3d nv_sum;
    nv_sum.setZero();
    get_act_taxelId(t_data);
    act_taxel_num = act_Ids.size();
    if(act_taxel_num > 0){
        press.setZero(act_taxel_num);
        weighted_press.setZero(act_taxel_num);
        for(int i = 0; i < act_taxel_num; i++){
            press[i] = data.fingertip_tac_pressure[act_Ids[i]];
        }
        press_sum = press.sum();
        weighted_press = press/press_sum;
        for(int i = 0; i < act_taxel_num; i++){
            nv_sum += weighted_press[i] * data.fingertip_tac_nv[act_Ids[i]];
        }
        nv = nv_sum;
        nv = nv/nv.norm();
    }
    else{
        nv(0) = 1;
        nv(1) = 0;
        nv(2) = 0;
    }
}

void FingertipTac::est_pressure(tac_data t_data){
    double sum_press_act;
    sum_press_act = 0;
    get_act_taxelId(t_data);
    act_taxel_num = act_Ids.size();
    press.setZero(act_taxel_num);
    weighted_press.setZero(act_taxel_num);
    for(int i = 0; i < act_taxel_num; i++){
        press[i]= data.fingertip_tac_pressure[act_Ids[i]];
        sum_press_act += press[i];
    }
    if(act_taxel_num != 0){
        pressure = sum_press_act / act_taxel_num;
    }
    else{
        pressure = 0;
    }
}

void FingertipTac::est_ct_info(tac_data t_data){
    est_pressure(t_data);
    est_cop(t_data);
    est_nv(t_data);
}

int FingertipTac::est_ct_taxelId(tac_data t_data){
    int taxelid = 0;
    est_pressure(t_data);
    est_cop(t_data);
    est_nv(t_data);
    taxelid = find_near_taxelid(t_data);
    return taxelid;
}

bool FingertipTac::isContact(tac_data t_data){
    get_act_taxelId(t_data);
    act_taxel_num = act_Ids.size();
    if(act_taxel_num > 0){
        return true;
    }else{
        return false;
    }
}

int FingertipTac::find_near_taxelid(tac_data t_data){
    int id;
    Eigen::Vector3d dev;
    std::vector<double> dis_vec;
    dev.setZero();
    get_act_taxelId(t_data);
    act_taxel_num = act_Ids.size();
    for(int i = 0; i < act_taxel_num; i++){
        dev = pos - data.fingertip_tac_position[act_Ids[i]];
        dis_vec.push_back(dev.norm());
    }
    std::vector<double>::iterator result;
    result = std::min_element(dis_vec.begin(), dis_vec.end());
    id = act_Ids[std::distance(dis_vec.begin(), result)];
    return id;
}
void FingertipTac::get_act_taxelId(tac_data t_data){
    act_Ids.clear();
    for(int i = 0; i < t_data.tac_num; i++){
        if(t_data.fingertip_tac_pressure[i] >= 0.1){
            act_Ids.push_back(i);
        }
    }
}

void FingertipTac::clear_data(){
    data.fingertip_tac_pressure.clear();
    act_Ids.clear();
}