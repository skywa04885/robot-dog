//
// Created by luke on 1-12-22.
//

#ifndef BACKEND_POINT_CLOUD_RENDERER_H
#define BACKEND_POINT_CLOUD_RENDERER_H

#include <boost/cast.hpp>
#include <boost/geometry.hpp>
#include <boost/container/vector.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <gtkmm/drawingarea.h>

class point_cloud_renderer: public Gtk::DrawingArea
{
protected:
  boost::tuple<double, double> m_viewport_origin;
  double m_zoom;
public:
  point_cloud_renderer(void);
public:
  inline boost::tuple<double, double> &get_viewport_origin(void)
  {
    return this->m_viewport_origin;
  }

  inline double &get_zoom(void)
  {
    return this->m_zoom;
  }
public:
  virtual bool on_draw(const Cairo::RefPtr<Cairo::Context> &cairoContext);
};

#endif //BACKEND_POINT_CLOUD_RENDERER_H
