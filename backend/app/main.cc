#include "main.h"

#include "slam/point_cloud/registrator.h"
#include "kinematics/leg.h"
#include "slam/scanning/lidar.h"

int main(int argc, char *argv[])
{
  app::slam::point_cloud::registrator reg;

  boost::container::vector<app::slam::point_cloud::registrator::point> source = {
    {-5., -5.},
    {5., -5.},
    {0., 10.}
  };

  boost::container::vector<app::slam::point_cloud::registrator::value> target = {
    std::make_pair(app::slam::point_cloud::registrator::point {5., 5.}, 0),
    std::make_pair(app::slam::point_cloud::registrator::point{-5, 5.}, 1),
    std::make_pair(app::slam::point_cloud::registrator::point{0., -10.}, 2)
  };

  app::slam::point_cloud::registrator::tree tree;
  tree.insert(target.begin(), target.end());

  reg.execute(source, tree);

  auto a = app::slam::scanning::lidar::builder()
    .set_serial_port("/dev/ttyUSB0")
    .set_serial_baud(115200).build();
  

  return 0;
}
