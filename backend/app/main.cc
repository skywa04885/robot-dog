#include "main.h"
#include <gtkmm/application.h>

#include "slam/point_cloud/registrator.h"
#include "kinematics/leg.h"
#include "slam/process.h"
#include "slam/scanning/lidar.h"
#include "gui/window.h"
#include "events/event_bus.h"
#include "events/event_bus_consumer.h"

int main(int argc, char *argv[])
{
  app::slam::process::get_instance();

  BOOST_LOG_TRIVIAL(info) << "Starting GTK3 application";
  Glib::RefPtr<Gtk::Application> application = Gtk::Application::create(argc, argv, "nl.flukerieff.robot.backend");
  window window{};
  return application->run(window);

  return 0;
}
