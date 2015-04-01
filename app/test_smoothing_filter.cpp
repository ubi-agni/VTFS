#include <UtilModule/TemporalSmoothingFilter.h>
#include <ICLGeom/GeomDefs.h>

using namespace icl::geom;

int main(){
  Vec v;
  TemporalSmoothingFilter<Vec> vf(10,Median,Vec(0,0,0,0));
  TemporalSmoothingFilter<double> vd(20,Average);
  
  for(int i=0;i<1000;++i){
    SHOW(vf.push(Vec(i,i,i,1)).transp());

    SHOW(vd.push(i));
  }
}
