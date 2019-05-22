/*
 * Copyright (C) 2018  Love Mowitz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include <Eigen/Core>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] << "Generates the speed requests for Lynx" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> "
      << " [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 [--verbose]" << std::endl;
    retCode = 1;
  } else {
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    bool const verbose{static_cast<bool>(commandlineArguments.count("verbose"))};

    auto onLocalPath{[&od4, &verbose](cluon::data::Envelope &&envelope)
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 2601) {
          auto msg = cluon::extractMessage<opendlv::logic::action::LocalPath>(std::move(envelope));

          std::string data = msg.data();
          uint32_t length = msg.length();
          Eigen::MatrixXf path(length, 2);
          for (uint32_t i = 0; i < length; i++) {
            float x;
            float y;

            memcpy(&x, data.c_str() + (3 * i + 0) * 4, 4);
            memcpy(&y, data.c_str() + (3 * i + 1) * 4, 4);
            // z not parsed, since not used

            path(i,0) = x;
            path(i,1) = y;
          }

          float angle;
          // TODO: calculate angle based on path...

          opendlv::logic::action::AimPoint aimPoint;
          aimPoint.azimuthAngle(angle);
          od4.send(aimPoint, cluon::time::now(), 2701);
        }
      }};
    od4.dataTrigger(opendlv::logic::action::LocalPath::ID(), onLocalPath);


    // Just sleep as this microservice is data driven
    using namespace std::literals::chrono_literals;
    while(od4.isRunning()) {
      std::this_thread::sleep_for(1s);
    }

  }
  return retCode;
}

