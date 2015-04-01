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

#include <ICLQt/GUI.h>
#include <ICLQt/Widget.h>
#include <ICLMath/LinearTransform1D.h>
#include <ICLMath/StraightLine2D.h>
#include <vector>

namespace icl{
  class VisTool : public Configurable{
    GUI gui;
    GUI props;
    
    int W,H;
    std::vector<Point32f> trace;
    
    struct TTool{
      LinearTransform1D tx, ty;
      inline Point32f operator()(const Point32f &p) const{
        return Point32f(tx(p.x), ty(p.y));
      }
    };
    
    public: 
    VisTool();
    
    QWidget *getRootWidget();
    icl::qt::ICLWidget *getWidget();
    void propertyChanged(const Property &p);
    
    void setCurrentPos(const Point32f &p);
    void setStartPoint(const Point32f &p);
    void setEndPoint(const Point32f &p);
    void setTraceVisible(bool on);
    
    void clearTrace();
    private:
    
    TTool getTTool() const;
    void drawCircle2(DrawHandle &draw, const TTool &t, const Point32f &p, const std::string &id);
    void drawCircle(DrawHandle &draw, const TTool &t, const std::string &id);
    void drawLine(DrawHandle &draw, const TTool &t, const Point32f &a, 
                  const Point32f &b, const std::string &id);
    void drawQuadrangle(DrawHandle &draw, const TTool &t, const Point32f ps[4], 
                        const std::string &id);
    Point32f cvt(const StraightLine2D::Pos &p);
    void update();
  };
}

