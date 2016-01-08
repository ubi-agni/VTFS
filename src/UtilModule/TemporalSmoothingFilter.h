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

#include <deque>
#include <algorithm>
#include <numeric>
#include <stdexcept>

enum FilterMethod{
  Median,
  Average,
};

/// Hey, thats only defined for doubles and for vectors!
template<class T>
class TemporalSmoothingFilter{
  size_t m_timewindow;
  std::deque<T> m_buffer;
  T m_zeroValue;
  FilterMethod m_method;  
  
  public:
  
  TemporalSmoothingFilter(size_t timewindow, 
                          const FilterMethod &method,
                          const T &zeroValue=T(0)) throw (std::logic_error);
  void clear();
  T push(const T &t);
};
