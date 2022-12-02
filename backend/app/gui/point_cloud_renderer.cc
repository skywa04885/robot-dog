//
// Created by luke on 1-12-22.
//

#include "point_cloud_renderer.h"
#include "../slam/process.h"

#include <iostream>

point_cloud_renderer::point_cloud_renderer(void):
  m_viewport_origin(0.0, 0.0),
  m_zoom(1.0)
{
  this->set_size_request(500, 500);
}

bool point_cloud_renderer::on_draw(const Cairo::RefPtr<Cairo::Context> &cairoContext)
{
  const auto d_width = boost::numeric_cast<double>(this->get_width());
  const auto d_height = boost::numeric_cast<double>(this->get_height());

  boost::tuple<double, double> point_size(10.0, 10.0);

  boost::tuple<double, double> viewport_size(d_width * this->m_zoom, d_height * this->m_zoom);

  // Draws the background.
  cairoContext->set_source_rgb(0.7, 0.7, 0.7);
  cairoContext->rectangle(0., 0., d_width, d_height);
  cairoContext->fill();

  // Defines the box that will contain all the points in the viewport.
  boost::geometry::model::box<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>
    boundary({
               this->m_viewport_origin.get<0>() - viewport_size.get<0>() / 2.0,
               this->m_viewport_origin.get<1>() - viewport_size.get<1>() / 2.0,
             }, {
               this->m_viewport_origin.get<0>() + viewport_size.get<0>() / 2.0,
               this->m_viewport_origin.get<1>() + viewport_size.get<1>() / 2.0,
             });

  // Gets all the points in the viewport. Because we're dealing with multiple threads
  //  we will use a lock guard to prevent the tree from being modified while reading.
  boost::container::vector<std::pair<boost::geometry::model::point<
    double, 2, boost::geometry::cs::cartesian>, unsigned int>> points;
  {
    app::slam::process &slam_process = app::slam::process::get_instance();
    boost::lock_guard lock(slam_process.get_environment_point_cloud_mutex());
    slam_process.get_environment_point_cloud().query(boost::geometry::index::within(boundary), std::back_inserter(points));
  }

  // Loops over all the points in the viewport and draws them to the screen.
  cairoContext->set_source_rgba(0.1, 0.1, 0.1, 0.5);
  for (const auto &pair: points)
  {
    const auto &point = pair.first;

    const double x = ((point.get<0>() - this->m_viewport_origin.get<0>() + viewport_size.get<0>() / 2.0) / viewport_size.get<0>()) * d_width;
    const double y = ((point.get<1>() - this->m_viewport_origin.get<1>() + viewport_size.get<1>() / 2.0) / viewport_size.get<1>()) * d_height;

    cairoContext->rectangle(
      x - point_size.get<0>(),
      y - point_size.get<1>(),
      point_size.get<0>(),
      point_size.get<1>()
    );
    cairoContext->fill();
  }

  return false;
}