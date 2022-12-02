//
// Created by luke on 1-12-22.
//

#ifndef BACKEND_WINDOW_H
#define BACKEND_WINDOW_H

#include <iostream>

#include <gtkmm/button.h>
#include <gtkmm/window.h>
#include <gtkmm/drawingarea.h>
#include <gtkmm/box.h>
#include <glibmm/main.h>
#include <gtkmm/paned.h>

#include "point_cloud_renderer.h"

class window: public Gtk::Window
{
protected:
  point_cloud_renderer m_current_scan_renderer;
  Gtk::Button m_stop_button;
  Gtk::Paned m_panes;
public:
  window(void);
  virtual ~window(void);

public:
  virtual bool on_scroll_event(GdkEventScroll *scroll_event);

  virtual bool on_key_press_event(GdkEventKey* key_event);

  bool on_refresh();
};

#endif //BACKEND_WINDOW_H
