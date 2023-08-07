
/*
 * Licence: GPLv3 or LGPLv3 at your choice.
 * Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 * Author email: ksatyaki@gmail.com
 */

#include <iostream>
#include <numeric>

#include <QFile>
#include <QXmlStreamReader>

#include <boost/program_options.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

struct Obstacle {
  double x1{0.0};
  double x2{0.0};
  double y1{0.0};
  double y2{0.0};
  Obstacle(double x1, double x2, double y1, double y2)
      : x1(x1), x2(x2), y1(y1), y2(y2) {}

  std::vector<std::pair<double, double>> toCells(const double unit) const {
    int i;               // loop counter
    double ystep, xstep; // the step on y and x axis
    double error;        // the error accumulated during the increment
    double errorprev;    // *vision the previous value of the error variable
    // int y = y1 - 0.5, x = x1 - 0.5;  // the line points
    double y = y1, x = x1; // the line points
    double ddy, ddx; // compulsory variables: the double values of dy and dx
    double dx = x2 - x1;
    double dy = y2 - y1;
    double unit_x, unit_y;
    unit_x = unit;
    unit_y = unit;

    if (dy < 0) {
      ystep = -unit_y;
      dy = -dy;
    } else {
      ystep = unit_y;
    }
    if (dx < 0) {
      xstep = -unit_x;
      dx = -dx;
    } else {
      xstep = unit_x;
    }

    ddy = 2 * dy; // work with double values for full precision
    ddx = 2 * dx;

    std::vector<std::pair<double, double>> obstacle_cells; // TODO - reserve.
    obstacle_cells.emplace_back(std::make_pair(x, y));

    if (ddx >= ddy) {
      // first octant (0 <= slope <= 1)
      // compulsory initialization (even for errorprev, needed when dx==dy)
      errorprev = error = dx; // start in the middle of the square
      for (i = 0; i <= (int)(dx / unit); i++) {
        // do not use the first point (already done)
        x += xstep;
        error += ddy;
        if (error > ddx) {
          // increment y if AFTER the middle ( > )
          y += ystep;
          error -= ddx;
          // three cases (octant == right->right-top for directions
          // below):
          if (error + errorprev < ddx) {
            // bottom square also
            obstacle_cells.emplace_back(std::make_pair(x, y - ystep));
          } else if (error + errorprev > ddx) {
            // left square also
            obstacle_cells.emplace_back(std::make_pair(x - xstep, y));
          } else {
            // corner: bottom and left squares also
            obstacle_cells.emplace_back(std::make_pair(x, y - ystep));
            obstacle_cells.emplace_back(std::make_pair(x - xstep, y));
          }
        }
        obstacle_cells.emplace_back(std::make_pair(x, y));
        errorprev = error;
      }
    } else {
      // the same as above
      errorprev = error = dy;
      for (i = 0; i <= (int)(dy / unit); i++) {
        y += ystep;
        error += ddx;
        if (error > ddy) {
          x += xstep;
          error -= ddy;
          if (error + errorprev < ddy) {
            obstacle_cells.emplace_back(std::make_pair(x - xstep, y));
          } else if (error + errorprev > ddy) {
            obstacle_cells.emplace_back(std::make_pair(x, y - ystep));
          } else {
            obstacle_cells.emplace_back(std::make_pair(x, y - ystep));
            obstacle_cells.emplace_back(std::make_pair(x - xstep, y));
          }
        }
        obstacle_cells.emplace_back(std::make_pair(x, y));
        errorprev = error;
      }
    }
    return obstacle_cells;
  }
};

void saveMapPgmAndYaml(const nav_msgs::msg::OccupancyGrid &map,
                       const std::string &pgmFile,
                       const std::string &yamlFile) {
  printf("Received a %d X %d map @ %.3f m/pix", map.info.width,
                       map.info.height, map.info.resolution);

  printf("Writing map occupancy data to %s", pgmFile.c_str());
  FILE *out = fopen(pgmFile.c_str(), "w");
  if (!out) {
    printf("Couldn't save map file to %s", pgmFile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
          map.info.resolution, map.info.width, map.info.height);
  for (unsigned int y = 0; y < map.info.height; y++) {
    for (unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if (map.data[i] == 0) { // occ [0,0.1)
        fputc(254, out);
      } else if (map.data[i] == +100) { // occ (0.65,1]
        fputc(000, out);
      } else { // occ [0.1,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);

  printf("Writing map occupancy data to %s", yamlFile.c_str());
  FILE *yaml = fopen(yamlFile.c_str(), "w");

  fprintf(yaml,
          "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
          "0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          pgmFile.c_str(), map.info.resolution, map.info.origin.position.x,
          map.info.origin.position.y, 0.0);

  fclose(yaml);

  printf("Done\n");
}

namespace po = boost::program_options;

int main(int argn, char *args[]) {

  std::string iFile, oYaml, oPgm;
  float resolution = 0.1f;
  QXmlStreamReader xml_reader;

  // Declare some program options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce this help message")(
      "input,i", po::value<std::string>(), "pedsim scenario XML")(
      "output,o", po::value<std::string>(), "occupancy map file name")(
      "resolution,r", po::value<float>(), "occupancy map resolution");

  po::variables_map vm;
  po::store(po::parse_command_line(argn, args, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
    std::cout << desc << std::endl;
  if (vm.count("input")) {
    iFile = vm["input"].as<std::string>();
  }
  if (vm.count("output")) {
    oYaml = vm["output"].as<std::string>() + ".yaml";
    oPgm = vm["output"].as<std::string>() + ".pgm";
  }
  if (vm.count("input") == 0 or vm.count("output") == 0) {
    std::cerr << "WRONG USAGE";
    std::cout << desc << std::endl;
  } else {
    if (vm.count("resolution")) {
      resolution = vm["resolution"].as<float>();
    }
    std::cout << "Output occupancy map YAML: " << oYaml << std::endl;
    std::cout << "Output occupancy map PNG: " << oYaml << std::endl;
    std::cout << "Input pedsim XML: " << iFile.c_str() << std::endl;
    std::cout << "Map resolution: " << resolution << " m/cell" << std::endl;
  }

  // Steal code from scenarioreader.cpp
  // open file
  QFile file(QString::fromStdString(iFile));
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    printf("Couldn't open scenario file!");
    return -1;
  }

  // read input
  xml_reader.setDevice(&file);

  std::vector<Obstacle> obs;
  double x_min = std::numeric_limits<double>::max();
  double x_max = -std::numeric_limits<double>::min();
  double y_min = std::numeric_limits<double>::max();
  double y_max = -std::numeric_limits<double>::min();

  while (!xml_reader.atEnd()) {
    xml_reader.readNext();
    if (xml_reader.isStartElement()) {
      const QString elementName = xml_reader.name().toString();
      const QXmlStreamAttributes elementAttributes = xml_reader.attributes();

      if ((elementName == "scenario") || (elementName == "welcome")) {
        // nothing to do
      } else if (elementName == "obstacle") {
        const double x1 = elementAttributes.value("x1").toString().toDouble();
        const double y1 = elementAttributes.value("y1").toString().toDouble();
        const double x2 = elementAttributes.value("x2").toString().toDouble();
        const double y2 = elementAttributes.value("y2").toString().toDouble();

        if (x_min > x1)
          x_min = x1;
        if (x_min > x2)
          x_min = x2;

        if (x_max < x1)
          x_max = x1;
        if (x_max < x2)
          x_max = x2;

        if (y_min > y1)
          y_min = y1;
        if (y_min > y2)
          y_min = y2;

        if (y_max < y1)
          y_max = y1;
        if (y_max < y2)
          y_max = y2;

        // Write to nav_msgs
        obs.emplace_back(Obstacle(x1, x2, y1, y2));
      }
    }
  }

  std::cout << "X is in the range (" << x_min << ", " << x_max << ")\n";
  std::cout << "Y is in the range (" << y_min << ", " << y_max << ")\n";

  std::vector<std::pair<double, double>> all_obs_points;
  for (const Obstacle &ob : obs) {
    auto this_obs_points = ob.toCells(resolution);
    all_obs_points.insert(all_obs_points.begin(), this_obs_points.begin(),
                          this_obs_points.end());
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.info.height = (y_max - y_min) / resolution + 2;
  grid.info.width = (x_max - x_min) / resolution + 2;
  grid.info.origin.position.x = x_min;
  grid.info.origin.position.y = y_min;
  grid.info.resolution = resolution;

  grid.data.resize(grid.info.height * grid.info.width);

  for (const auto &point : all_obs_points) {
    double col = std::round((point.first - x_min) / resolution);
    double row = std::round((point.second - y_min) / resolution);

    grid.data[row * grid.info.width + col] = 100;
  }

  saveMapPgmAndYaml(grid, oPgm, oYaml);
  return 0;
}