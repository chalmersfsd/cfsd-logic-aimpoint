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
#include <mutex>
#include <cmath>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#define PI_F 3.14159265358979f

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("previewTime")
      || 0 == commandlineArguments.count("lowPassFactor")
      || 0 == commandlineArguments.count("minDistance")) {
    std::cerr << argv[0] << "Generates the speed requests for Lynx" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --previewTime=<Aim point preview Time>"
      << " --lowPassFactor=<Low pass filter factor for angle> --minDistance=<Minimum aim point distance> "
      << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --previewTime=0.5 --lowPassFactor=0.95 --minDistance=2 [--verbose]" << std::endl;
    retCode = 1;
  } else {
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    float lowPassFactor{static_cast<float>(std::stof(commandlineArguments["lowPassFactor"]))};
    float previewTime{static_cast<float>(std::stof(commandlineArguments["previewTime"]))};
    float minDistance{static_cast<float>(std::stof(commandlineArguments["minDistance"]))};
    bool const verbose{static_cast<bool>(commandlineArguments.count("verbose"))};

    std::mutex groundSpeedMutex;
    float groundSpeed{0.0f};
    float headingRequestOld;
    float xAimPoint, yAimPoint;

    auto onGroundSpeedReading{[&groundSpeedMutex, &groundSpeed, verbose](cluon::data::Envelope &&envelope) 
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 112) {
          auto gsr = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(envelope));

          std::lock_guard<std::mutex> lock(groundSpeedMutex);
          groundSpeed = gsr.groundSpeed();

          if (verbose) {
            std::cout << "GroundSpeedReading: " << groundSpeed << std::endl;
          }
        }
      }};
    od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), onGroundSpeedReading);

    auto onLocalPath{[&](cluon::data::Envelope &&envelope)
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 2601) {
          auto msg = cluon::extractMessage<opendlv::logic::action::LocalPath>(std::move(envelope));

          std::string data = msg.data();
          uint32_t length = msg.length();

          // If message is empty, use previous value
          // TODO: Add logic for the case of msg length == 0
          if (msg.length() != 0)
          {
            Eigen::MatrixXf path(length, 2);
            for (uint32_t i = 0; i < length; i++)
            {
              float x;
              float y;

              memcpy(&x, data.c_str() + (3 * i + 0) * 4, 4);
              memcpy(&y, data.c_str() + (3 * i + 1) * 4, 4);
              // z not parsed, since not used

              path(i, 0) = x;
              path(i, 1) = y;
            }

            // Copy groundSpeed
            float groundSpeedCopy;
            {
              std::lock_guard<std::mutex> lock(groundSpeedMutex);
              groundSpeedCopy = groundSpeed;
            }

            // Calculate angle based on path
            float previewDistance = std::abs(groundSpeedCopy) * previewTime;
            if (previewDistance < minDistance) previewDistance = minDistance;

            float pathLength{0.0f};

            
            // Select last point if the preview distance is larger than the path length
            xAimPoint = path(path.rows() - 1, 0);
            yAimPoint = path(path.rows() - 1, 1);

            // Follow path and stop when path is longer than preview distance
            for (int i = 0; i < path.rows() - 1; i++)
            {
              float pointDistance = (path.row(i) - path.row(i + 1)).norm();

              if (pathLength + pointDistance > previewDistance)
              {
                float distanceToGo = previewDistance - pathLength;
                float pointAngle = std::atan2(path(i + 1, 1) - path(i, 1), path(i + 1, 0) - path(i, 0));
                xAimPoint = path(i, 0) + distanceToGo * std::cos(pointAngle);
                yAimPoint = path(i, 1) + distanceToGo * std::sin(pointAngle);
                break;
              }

              pathLength += pointDistance;
            }
          }

          float headingRequest = std::atan2(yAimPoint, xAimPoint);
          headingRequest = headingRequest * lowPassFactor + headingRequestOld * (1.0f - lowPassFactor);


          float aimPointDistance = std::sqrt(std::pow(xAimPoint, 2.0f) + std::pow(yAimPoint, 2.0f));

          if (!std::isnormal(headingRequest) || !std::isnormal(aimPointDistance)) {
            headingRequest = headingRequestOld;
            aimPointDistance = 0.0f;
          }

          opendlv::logic::action::AimPoint aimPoint;
          aimPoint.distance(aimPointDistance);
          aimPoint.azimuthAngle(headingRequest);
          od4.send(aimPoint, cluon::time::now(), 2701);

          headingRequest -= PI_F / 2.0f; // Offset 90 degrees to the local coordinate system
          opendlv::proxy::GroundSteeringRequest gsr;
          gsr.groundSteering(headingRequest * 180.0f / PI_F); // Convert radians to degrees for steering service
          od4.send(gsr, cluon::time::now(), 2801);

          headingRequestOld = headingRequest;

          if (verbose) {
            std::cout << "Aim point distance: " << aimPoint.distance() << "| angle: " << aimPoint.azimuthAngle() << std::endl;
          }
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

