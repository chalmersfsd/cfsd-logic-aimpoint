# cfsd-cognition-velocity-planner
This microservice generates the speed requests for Lynx. By looking at the curvature of the local or global path the microservice calculates the longitudinal speed of the vehicle and outputs a groundSpeedRequest for the longitudinal control microservice.

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-cognition-velocity-control:v0.0.1 .

### Run
See included docker-compose file.

### Dependencies
 - Eigen
