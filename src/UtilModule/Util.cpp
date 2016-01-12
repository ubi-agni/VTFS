#include "Util.h"

Eigen::Vector3d euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm){
	Eigen::Vector3d CtrlAxisAngle;
	Eigen::Matrix3d DesiredMatrix;
	DesiredMatrix = euler2tm(la,tm);
	CtrlAxisAngle = tm2axisangle(DesiredMatrix);
	return CtrlAxisAngle;
}

Eigen::Vector3d g_euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm){
	Eigen::Vector3d CtrlAxisAngle;
	Eigen::Matrix3d DesiredMatrix;
	DesiredMatrix = g_euler2tm(la,tm);
	CtrlAxisAngle = tm2axisangle(DesiredMatrix);
	return CtrlAxisAngle;
}

Eigen::Matrix3d euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm){
	Eigen::Matrix3d DeltaMatrix_z;
	Eigen::Matrix3d DeltaMatrix_x;
	Eigen::Matrix3d DeltaMatrix_y;
	Eigen::Matrix3d DesiredMatrix;
	double DeltaAlfa,DeltaBeta,DeltaGama;
	
	
	DeltaMatrix_z.setIdentity();
	DeltaMatrix_x.setIdentity();
	DeltaMatrix_y.setIdentity();
	
	DeltaAlfa = la(0);
	DeltaBeta = la(1);
	DeltaGama = la(2);
	
	DeltaMatrix_z(0,0) = cos(DeltaGama);
	DeltaMatrix_z(0,1) = (-1)*sin(DeltaGama);
	DeltaMatrix_z(1,0) = sin(DeltaGama);
	DeltaMatrix_z(1,1) = cos(DeltaGama);
	
	DeltaMatrix_x(1,1) = cos(DeltaAlfa);
	DeltaMatrix_x(1,2) = (-1)*sin(DeltaAlfa);
	DeltaMatrix_x(2,1) = sin(DeltaAlfa);
	DeltaMatrix_x(2,2) = cos(DeltaAlfa);
	
	DeltaMatrix_y(0,0) = cos(DeltaBeta);
	DeltaMatrix_y(0,2) = sin(DeltaBeta);
	DeltaMatrix_y(2,0) = (-1)*sin(DeltaBeta);
	DeltaMatrix_y(2,2) = cos(DeltaBeta);
	
	DesiredMatrix = tm * DeltaMatrix_z * DeltaMatrix_y * DeltaMatrix_x;

	return DesiredMatrix;
}

Eigen::Matrix3d g_euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm){
	Eigen::Matrix3d DeltaMatrix_z;
	Eigen::Matrix3d DeltaMatrix_x;
	Eigen::Matrix3d DeltaMatrix_y;
	Eigen::Matrix3d DesiredMatrix;
	double DeltaAlfa,DeltaBeta,DeltaGama;
	
	DeltaMatrix_z.setIdentity();
	DeltaMatrix_x.setIdentity();
	DeltaMatrix_y.setIdentity();
	
	DeltaAlfa = la(0);
	DeltaBeta = la(1);
	DeltaGama = la(2);
	
	DeltaMatrix_z(0,0) = cos(DeltaGama);
	DeltaMatrix_z(0,1) = (-1)*sin(DeltaGama);
	DeltaMatrix_z(1,0) = sin(DeltaGama);
	DeltaMatrix_z(1,1) = cos(DeltaGama);
	
	DeltaMatrix_x(1,1) = cos(DeltaAlfa);
	DeltaMatrix_x(1,2) = (-1)*sin(DeltaAlfa);
	DeltaMatrix_x(2,1) = sin(DeltaAlfa);
	DeltaMatrix_x(2,2) = cos(DeltaAlfa);
	
	DeltaMatrix_y(0,0) = cos(DeltaBeta);
	DeltaMatrix_y(0,2) = sin(DeltaBeta);
	DeltaMatrix_y(2,0) = (-1)*sin(DeltaBeta);
	DeltaMatrix_y(2,2) = cos(DeltaBeta);
	
	DesiredMatrix = DeltaMatrix_x * DeltaMatrix_y * DeltaMatrix_z * tm;
	return DesiredMatrix;
}


Eigen::Vector3d tm2axisangle(Eigen::Matrix3d tm){
	Eigen::Vector3d CtrlAxisAngle;
	Eigen::Matrix3d DesiredMatrix;
	Eigen::Vector3d Desiredomega;
	double mtrace;
	double Desiredtheta;
	
	DesiredMatrix = tm;
	mtrace = DesiredMatrix.trace();
	if(mtrace < -1){
		mtrace = -1;
	}
	if(mtrace > 3){
		mtrace = 3;
	}
	Desiredtheta = acos((mtrace-1)/2);
	Desiredomega(0) = (DesiredMatrix(2,1) - DesiredMatrix(1,2));
	Desiredomega(1) = (DesiredMatrix(0,2) - DesiredMatrix(2,0));
	Desiredomega(2) = (DesiredMatrix(1,0) - DesiredMatrix(0,1));
	Desiredomega *= (1.0/(2*sin(Desiredtheta)));
	CtrlAxisAngle = Desiredomega.normalized()*Desiredtheta; 
	return CtrlAxisAngle;
}

std::pair<Eigen::Vector3d,double> tm2axisangle_4(Eigen::Matrix3d tm,bool& b){
    Eigen::Vector3d CtrlAxisAngle;
    Eigen::Matrix3d DesiredMatrix;
    Eigen::Vector3d Desiredomega;
    std::pair<Eigen::Vector3d,double> ax;
    double mtrace;
    double Desiredtheta;

    DesiredMatrix = tm;
    mtrace = DesiredMatrix.trace();
    if(mtrace < -1){
        mtrace = -1;
    }
    if(mtrace > 3){
        mtrace = 3;
    }
    if(fabs(mtrace-3) >= 0.00001){
        Desiredtheta = acos((mtrace-1)/2);
        Desiredomega(0) = (DesiredMatrix(2,1) - DesiredMatrix(1,2));
        Desiredomega(1) = (DesiredMatrix(0,2) - DesiredMatrix(2,0));
        Desiredomega(2) = (DesiredMatrix(1,0) - DesiredMatrix(0,1));
        Desiredomega *= (1.0/(2*sin(Desiredtheta)));
        ax.first = Desiredomega.normalized();
        ax.second = Desiredtheta;
        b = true;
    }
    else{
        ax.first(0) = 0;
        ax.first(1) = 0;
        ax.first(2) = 1;
        ax.second = 0;
        b = false;
    }
    return ax;
}


double _smooth_filter(std::deque<double> t){
  /* #include <numeric>
      double sum = std::accumulate(t.begin(),e.end(), 0.0);
      return sum/t.size();
      */
	double sum;
	sum = 0;
	for (std::deque<double>::iterator it = t.begin(); it!=t.end(); ++it)
		sum+= *it;
	return sum/t.size();
}


Eigen::Matrix3d GetSkrewFromVector(Eigen::Vector3d vec){
	Eigen::Matrix3d SkrewM = Eigen::Matrix3d::Zero();
	SkrewM(0,0) = 0;
	SkrewM(0,1) = -vec(2);
	SkrewM(0,2) = vec(1);
	SkrewM(1,0) = vec(2);
	SkrewM(1,1) = 0;
	SkrewM(1,2) = -vec(0);
	SkrewM(2,0) = -vec(1);
	SkrewM(2,1) = vec(0);
	SkrewM(2,2) = 0;
	return SkrewM;
}

Eigen::Matrix3d makeSkewSymmetric(Eigen::Vector3d vec){
	Eigen::Matrix3d v;
	v.setZero();
	v(0,1) = -vec(2);
	v(0,2) = vec(1);
	v(1,0) = vec(2);
	v(1,2) = -vec(0);
	v(2,0) = -vec(1);
	v(2,1) = vec(0);
	return v;
}

Eigen::Matrix3d AlignVec(Eigen::Vector3d cur, Eigen::Vector3d des){
	Eigen::Vector3d vec;
	Eigen::Matrix3d E,tm,v;
	double s,c;
	vec.setZero();
	tm.setZero();
	v.setZero();
	E.setIdentity();
	vec = cur.cross(des);
	s = vec.norm();
	c = cur.dot(des);
	v = makeSkewSymmetric(vec);
	tm = E + v + v * v * ((1-c)/(s*s));
	return tm;
}

long long timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time){
	struct timeval temp_diff;
	if(difference==NULL){
		difference=&temp_diff;
	}
	difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
	difference->tv_usec=end_time->tv_usec-start_time->tv_usec;
	/* Using while instead of if below makes the code slightly more robust. */
	while(difference->tv_usec<0){
		difference->tv_usec+=1000000;
		difference->tv_sec -=1;
	}
	return 1000000LL*difference->tv_sec+difference->tv_usec;
} /* timeval_diff() */


Eigen::Vector3d kdl2eigen_position(const KDL::Frame& f){
    Eigen::Vector3d m;
    m.setZero();
    for(int i = 0; i < 3; i++)
        m(i) = f.p(i);
    return m;

}
Eigen::Matrix3d kdl2eigen_orien(const KDL::Frame& f){
    Eigen::Matrix3d m;
    m.setZero();
    for(int i = 0; i < 3;i++){
        for(int j = 0; j < 3;j++){
            m(i,j) = f.M(i,j);
        }
    }
    return m;
}

void global2local(Eigen::Vector3d g, Eigen::Matrix3d M, Eigen::Vector3d &l){
    l = M.transpose() *g;
}

void local2global(Eigen::Vector3d l, Eigen::Matrix3d M, Eigen::Vector3d &g){
    g = M*l;
}

std::pair<Eigen::Vector3d,double> omega_transform(std::pair<Eigen::Vector3d,Eigen::Vector3d> r_ax,Eigen::Matrix3d R_rinit_linit){
    std::pair<Eigen::Vector3d,double> l_ax;
    l_ax.first = R_rinit_linit * r_ax.first;
    l_ax.second = r_ax.second(0);
    return l_ax;
}
void gen_hm(Eigen::Matrix3d m,Eigen::Vector3d p,Eigen::MatrixXd& hm){
    for(int i = 0; i < 3;i++){
        for(int j = 0; j < 3; j++)
            hm(i,j) = m(i,j);
    }
    for(int i = 0; i < 3; i++){
        hm(i,3) = p(i);
    }
}
Eigen::Vector3d skewtovector(Eigen::Matrix3d m){
    Eigen::Vector3d v;
    v.setZero();
    v(0) = m(2,1);
    v(1) = m(0,2);
    v(2) = m(1,0);
    return v;
}

extern Eigen::Matrix3d vectortoskew(Eigen::Vector3d v){
    Eigen::Matrix3d m;
    m.setZero();
    m(2,1) = v(0);
    m(0,2) = v(1);
    m(1,0) = v(2);

    m(1,2) = -1*v(0);
    m(2,0) = -1*v(1);
    m(0,1) = -1*v(2);
    return m;
}

extern bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}



extern Eigen::Matrix3d gen_ort_basis(Eigen::Vector3d v1){
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Matrix3d ort;
    //ad-hoc way to create orthogonal basis, v1--z, v2--x, v3--y
    //v1 = [a;b;c]; v2 = u = [0;-c;b]; and v3 = cross(v1,v2);
    v2.setZero();
    v3.setZero();
    ort.setZero();
    v2(1) = -v1(2);
    v2(2) = v1(1);
    if((fabs(v2(0))<1e-5)&&(fabs(v2(1))<1e-5)&&(fabs(v2(2))<1e-5)){
        std::cout<<"construct orthogonal basis failure"<<std::endl;
        v3.setZero();
    }
    else{
        v3 = v1.cross(v2);
    }
    //construct matrix from vector
    ort.col(0) = v2;
    ort.col(1) = v3;
    ort.col(2) = v1;
    return ort;
}

//extern Eigen::VectorXd MatrixtoQuaternion(Eigen::Matrix3d m){
//    Eigen::VectorXd quater;
//    quater.setZero(4);
//    doule x,y,z,w;

//    w = sqrt(1.0 + m(0,0) + m(1,1) + m(2,2)) / 2.0;
//    double w4 = (4.0 * w);
//    x = (m(2,1) - m(1,2)) / w4 ;
//    y = (m(0,2) - m(2,0)) / w4 ;
//    z = (m(1,0) - m(0,1)) / w4 ;

//    quater(0) = w;
//    quater(1) = x;
//    quater(2) = y;
//    quater(3) = z;
//    return quater;
//}

extern Eigen::Matrix4d gen_transform(Eigen::Vector3d omega, Eigen::Vector3d vel, double theta){
    //follow mls94(a mathematical introduction to robotic manipulation), page 42;
    Eigen::Matrix4d transform;
    Eigen::Matrix3d E;
    transform.setIdentity();
    E.setIdentity();
    transform.block<3,3>(0,0) = (vectortoskew(omega) * theta).exp();
    transform.block<3,1>(0,3) = (E - (vectortoskew(omega) * theta).exp())*(omega.cross(vel)) + omega * omega.transpose() * vel * theta;
    return transform;

}
