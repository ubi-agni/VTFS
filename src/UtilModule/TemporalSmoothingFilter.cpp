#include <UtilModule/TemporalSmoothingFilter.h>
#include <ICLGeom/GeomDefs.h>
#include <Eigen/Dense>

template<class T>
T apply_median_smoothing(std::deque<T> &buffer){
  std::sort(buffer.begin(),buffer.end());
  return buffer[buffer.size()/2];
}
template<>
icl::geom::Vec apply_median_smoothing(std::deque<icl::geom::Vec> &buffer){
  std::vector<double> buf(buffer.size());
  icl::geom::Vec result;
  for(int d=0;d<4;++d){
    for(size_t i=0;i<buf.size();++i){
      buf[i] = buffer[i][d];
    }
    std::sort(buf.begin(),buf.end());
    result[d] = buf[buf.size()/2];
  }
  return result;
}

template<>
Eigen::Vector3d apply_median_smoothing(std::deque<Eigen::Vector3d> &buffer){
  std::vector<double> buf(buffer.size());
  Eigen::Vector3d result;
  for(int d=0;d<3;++d){
    for(size_t i=0;i<buf.size();++i){
      buf[i] = buffer[i][d];
    }
    std::sort(buf.begin(),buf.end());
    result[d] = buf[buf.size()/2];
  }
  return result;
}



template<class T>
TemporalSmoothingFilter<T>::TemporalSmoothingFilter(size_t timewindow, 
                                                    const FilterMethod &method,
                                                    const T &zeroValue) throw (std::logic_error):
  m_timewindow(timewindow),m_zeroValue(zeroValue),
  m_method(method){
  
  if(timewindow < 2){
    throw std::logic_error("TemporalSmoothingFilter: timewindow must be > 1");
  }
}

template<class T>
void TemporalSmoothingFilter<T>::clear(){
  m_buffer.clear();
}

template<class T>
T TemporalSmoothingFilter<T>::push(const T &t){
  m_buffer.push_back(t);
  if(m_buffer.size() > m_timewindow){
      m_buffer.pop_front();
  }
  if(m_method == Median){
    return apply_median_smoothing(m_buffer);
  }else{
    return std::accumulate(m_buffer.begin(),m_buffer.end(),m_zeroValue)/m_buffer.size();
  }
}





template class TemporalSmoothingFilter<float>;
template class TemporalSmoothingFilter<double>;
template class TemporalSmoothingFilter<icl::geom::Vec>;
template class TemporalSmoothingFilter<Eigen::Vector3d>;
