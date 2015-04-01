#include <ICLQt/Common.h>
#include <UtilModule/VisTool.h>
#include <ICLQt/IconFactory.h>

namespace icl{

  VisTool::VisTool(){
    W = 1000;
    H = 500;
    
    addProperty("colors.left","color","",Color(200,0,100));
    addProperty("colors.right","color","",Color(0,200,100));
    addProperty("colors.curr","color","",Color(0,100,200));
    addProperty("colors.trace","color","",Color(0,50,100));
    
    addProperty("colors.lines.connection","color","",Color(150,150,150));
    addProperty("colors.lines.top","color","",Color(200,0,100));
    addProperty("colors.lines.bottom","color","",Color(200,0,100));
    addProperty("colors.lines.trace","color","",Color(0,100,200));

    addProperty("colors.quads.top","color","",Color(200,0,100));
    addProperty("colors.quads.bottom","color","",Color(200,0,100));
    addProperty("colors.quads.center","color","",Color(50,50,50));

    addProperty("sizes.left","range","[1,100]",20);
    addProperty("sizes.right","range","[1,100]",20);
    addProperty("sizes.curr","range","[1,100]",30);
    addProperty("sizes.trace","range","[1,100]",10);
    
    addProperty("viewport.minX","range:spinbox","[-10000,10000]",0);
    addProperty("viewport.maxX","range:spinbox","[-10000,10000]",100);
    addProperty("viewport.minY","range:spinbox","[-10000,10000]",0);
    addProperty("viewport.maxY","range:spinbox","[-10000,10000]",100);

    
    addProperty("pos.left","Point32f","",Point32f(10,30));
    addProperty("pos.right","Point32f","",Point32f(90,50));
    addProperty("pos.curr","Point32f","",Point32f(30,40));
    
    addProperty("margin","range:spinbox", "[-10000, 10000]", 20);    
    addProperty("trace.show","flag","",true);
    addProperty("trace.empty","command","");
    addProperty("trace.mode","menu","points,line,both","both");
    
    setConfigurableID("VisTool");
    gui << Draw(Size(W,H)).handle("draw")
        << Create();

    props << Prop("VisTool") << Create();

    registerCallback(utils::function(this, &VisTool::propertyChanged));
    

    ImgQ icon = qt::cvt(IconFactory::create_image("empty"));
    color(255,255,255,255);
    for(int i=0;i<3;++i){
      line(icon, 10, 5*i+11, 22, 5*i+11);
    }
    DrawHandle draw = gui["draw"];
    draw->addSpecialButton("props", &icon, utils::function(props, &GUI::switchVisibility),
                           "show properties ..");
    
    trace.push_back(getPropertyValue("pos.curr"));
    update();
  }

  QWidget *VisTool::getRootWidget(){
    return gui.getRootWidget();
  }

  icl::qt::ICLWidget *VisTool::getWidget(){
    DrawHandle draw = gui["draw"];
    return *draw;
  }
  
  void VisTool::propertyChanged(const Property &p){
    if(p.name == "trace.empty"){
      trace.clear();
      trace.push_back(getPropertyValue("pos.curr"));
    }else if(p.name == "pos.curr"){
      trace.push_back(getPropertyValue("pos.curr"));
    }
    update();
  }
  
  VisTool::TTool VisTool::getTTool() const{
    TTool t = {
      LinearTransform1D(Range32f(getPropertyValue("viewport.minX"),
                                 getPropertyValue("viewport.maxX")), 
                        Range32f(0, W)),
      LinearTransform1D(Range32f(getPropertyValue("viewport.minY"),
                                 getPropertyValue("viewport.maxY")), 
                        Range32f(0, H))
    };
    return t;
  }

  void VisTool::drawCircle2(DrawHandle &draw, const TTool &t, const Point32f &p, const std::string &id){
    Color c = getPropertyValue("colors."+id);

    draw->color(c.x*.5, c.y*.5, c.z*.5, 255);
    draw->fill(c);
    draw->circle(t(p), getPropertyValue("sizes."+id) );
  }

  void VisTool::drawCircle(DrawHandle &draw, const TTool &t, const std::string &id){
    drawCircle2(draw, t, getPropertyValue("pos."+id), id);
  }
  void VisTool::drawLine(DrawHandle &draw, const TTool &t, const Point32f &a, 
                const Point32f &b, const std::string &id){
    draw->color(getPropertyValue("colors.lines."+id).as<Color>());
    draw->line(t(a), t(b));
  }

  void VisTool::drawQuadrangle(DrawHandle &draw, const TTool &t, const Point32f ps[4], 
                      const std::string &id){
    Color c = getPropertyValue("colors.quads."+id).as<Color>();
    draw->color(c.x, c.y, c.z, 100);
    draw->fill(c);
    std::vector<Point32f> tps(4);
    for(int i=0;i<4;++i) tps[i] = t(ps[i]);
    draw->polygon(tps);
  }
  
  Point32f VisTool::cvt(const StraightLine2D::Pos &p){
    return Point32f(p.x,p.y);
  }

  
  void VisTool::update(){
    DrawHandle draw = gui["draw"];
    TTool t = getTTool();
    
    Point32f l = getPropertyValue("pos.left");
    Point32f r = getPropertyValue("pos.right");
    float m = getPropertyValue("margin");
    Point32f d = r - l;
    Point32f dm = Point32f(d.y, -d.x).normalized() * m;
    
    float minX = getPropertyValue("viewport.minX");
    float maxX = getPropertyValue("viewport.maxX");
    float minY = getPropertyValue("viewport.minY");
    float maxY = getPropertyValue("viewport.maxY");

    Point32f pa = l + dm, pb = r + dm, pc = l - dm, pd = r - dm;
    StraightLine2D tLine(pa, (pb - pa).normalized());
    StraightLine2D bLine(pc, (pd - pc).normalized());
    StraightLine2D lBorder(Point32f(minX, minY), Point32f(0, 1));
    StraightLine2D rBorder(Point32f(maxX, minY), Point32f(0, 1));
    Point32f ia = cvt(tLine.intersect(lBorder));
    Point32f ib = cvt(tLine.intersect(rBorder));
    Point32f ic = cvt(bLine.intersect(lBorder));
    Point32f id = cvt(bLine.intersect(rBorder));
    

    {
      Point32f ps[] = { ib, ia, Point32f(minX,minY), Point32f(maxX,minY) };
      drawQuadrangle(draw, t, ps, "top");
    }
    {
      Point32f ps[] = { id, ic, Point32f(minX,maxY), Point32f(maxX,maxY) };
      drawQuadrangle(draw, t, ps, "bottom");
    }

    {
      Point32f ps[] = { ia, ib, id, ic };
      drawQuadrangle(draw, t, ps, "center");
    }
    
    drawLine(draw, t, l, r, "connection");
    drawLine(draw, t, ia, ib, "top");
    drawLine(draw, t, ic, id, "top");
    
    drawCircle(draw, t, "left");
    drawCircle(draw, t, "right");
    drawCircle(draw, t, "curr");
    
    if(getPropertyValue("trace.show")){
      std::string mode = getPropertyValue("trace.mode");
      if( mode == "line" || mode == "both"){
        for(size_t i=1;i<trace.size();++i){
          drawLine(draw, t, trace[i-1], trace[i], "trace");
        }
      }
      if( mode == "points" || mode == "both"){
        for(size_t i=0;i<trace.size()-1;++i){
          drawCircle2(draw, t, trace[i], "trace");
        }
      }
    }
    
    draw->render();
  }
  
  void VisTool::setCurrentPos(const Point32f &p){
    setPropertyValue("pos.curr", p);
  }

  void VisTool::setStartPoint(const Point32f &p){
    setPropertyValue("pos.left",p);
  }

  void VisTool::setEndPoint(const Point32f &p){
    setPropertyValue("pos.right",p);
  }
  
  void VisTool::setTraceVisible(bool on){
    setPropertyValue("trace.show",on);
  }

  void VisTool::clearTrace(){
    setPropertyValue("trace.empty",":-)");
  }
}
