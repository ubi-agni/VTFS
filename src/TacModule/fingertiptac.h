#ifndef FINGERTIPTAC_H
#define FINGERTIPTAC_H
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#define MID_THRESHOLD 0.02

enum ContactAreaT{
    NoContact,
    Area1,
    Area2,
    Area3,
    Area4
};


struct tac_data{
    int tac_num;
    std::vector<double> fingertip_tac_pressure;
    std::vector<Eigen::Vector3d> fingertip_tac_position;
    std::vector<Eigen::Vector3d> fingertip_tac_nv;
};

class FingertipTac
{
public:
    FingertipTac(int num);
    //raw data after pressure threshold
    tac_data data;
    //actived taxel id set
    std::vector<int> act_Ids;
    //actived taxel number;
    int act_taxel_num;
    //center of pressure position
    Eigen::Vector3d pos;
    //center of pressure normal direction
    Eigen::Vector3d nv;
    //estimated pressure value
    double pressure;
    //estimated linear slope
    Eigen::Vector3d slope;
    //actived taxel pressure vector and corresponding weighted value
    Eigen::VectorXd press,weighted_press;
    //estimate the weighted(based on pressure) contact center
    void est_cop(tac_data t_data);
    //estimate the contact normal direction
    void est_nv(tac_data t_data);
    //estimate the pressure value;
    void est_pressure(tac_data t_data);
    //get all contact information, posittion normal direction and pressure
    void est_ct_info(tac_data t_data);
    //get the actived taxel Ids
    void get_act_taxelId(tac_data t_data);
    //get the nearest taxel Id to Cop of taxel blob
    int est_ct_taxelId(tac_data t_data);
    void clear_data();
    bool isContact(tac_data t_data);
    ContactAreaT contactarea;
    //detect which area the contact point is located in
    ContactAreaT WhichArea(int act_id);
    //get the desired position and normal direction according the contact area
    /*AreaI: (act_id == 0)||(act_id == 3)||(act_id == 8)||(act_id == 11)
     * AreaII (act_id == 1)||(act_id == 2)||(act_id == 5)
     * Area III (act_id == 6)||(act_id == 9)||(act_id == 10)
     * Area IV (act_id == 4)||(act_id == 7)
    */
    Eigen::Vector3d get_Center_position(int areanum);
    Eigen::Vector3d get_Center_nv(int areanum);
    //used for generateing the local contract frame.
    Eigen::Matrix3d GenerateLF(Eigen::Vector3d nv);
    //get the esimated slope of contact line
    void get_slope(Eigen::Vector3d s);
    void slope_clear();
private:
    //sub routine to find the min distance taxel id
    int find_near_taxelid(tac_data t_data);
    void init_taxelmap();
    Eigen::MatrixXd readMatrix(const char *fn);
};

#endif // FINGERTIPTAC_H
