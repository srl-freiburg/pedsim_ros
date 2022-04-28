/**
 * Copyright 2016 Social Robotics Lab, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    # Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    # Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    # Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Billy Okal <okal@cs.uni-freiburg.de>
 */

#include <QApplication>
#include <signal.h>

#include <pedsim_simulator/simulator.hpp>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);

  // initialize resources
  auto sm = std::make_shared<Simulator>("pedsim_simulator");

  // use default SIGINT handler so CTRL+C works
  signal(SIGINT, SIG_DFL);

  if (sm->initializeSimulation()) {
    RCLCPP_INFO(sm->get_logger(), "node initialized, now running");
    sm->runSimulation();
  } else {
    RCLCPP_WARN(sm->get_logger(), "Could not initialize simulation, aborting");
    return EXIT_FAILURE;
  }

  return app.exec();
}
