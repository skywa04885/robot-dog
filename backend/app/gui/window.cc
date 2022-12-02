//
// Created by luke on 1-12-22.
//

#include "window.h"

window::window() :
  m_current_scan_renderer(),
  m_stop_button(),
  m_panes()
{

  this->add(this->m_current_scan_renderer);

  this->show_all();

  Glib::signal_timeout().connect(sigc::mem_fun(*this, &window::on_refresh), 500);
}

window::~window()
{

}

bool window::on_refresh()
{
  this->m_current_scan_renderer.queue_draw();

  return true;
}

bool window::on_scroll_event(GdkEventScroll *scroll_event)
{
  return false;
}

bool window::on_key_press_event(GdkEventKey *key_event)
{
  switch (key_event->keyval)
  {
    case GDK_KEY_z:
    {
      this->m_current_scan_renderer.get_zoom() += 0.05;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
    case GDK_KEY_Z:
    {
      this->m_current_scan_renderer.get_zoom() -= 0.05;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
    case GDK_KEY_Down:
    {
      this->m_current_scan_renderer.get_viewport_origin().get<1>() += 8.0;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
    case GDK_KEY_Up:
    {
      this->m_current_scan_renderer.get_viewport_origin().get<1>() -= 8.0;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
    case GDK_KEY_Left:
    {
      this->m_current_scan_renderer.get_viewport_origin().get<0>() -= 8.0;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
    case GDK_KEY_Right:
    {
      this->m_current_scan_renderer.get_viewport_origin().get<0>() += 8.0;
      this->m_current_scan_renderer.queue_draw();
      break;
    }
  }

  return false;
}
