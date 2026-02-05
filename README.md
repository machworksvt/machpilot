# Icarus Pilot
A driver for all interactive electronic components onboard the Icarus UAV. This project is ran through the ROS2 (Robot Operating System) and driven by C++ code written by the avionics subteam for MachWorks.
## Contents
- [Digital Communication Protocol](#digital_communication_protocol)
- [API](#api)

## API
- [Class Sensor](./src/ext_deps/Sensor.md)
- [Class Controller](./src/ext_deps/Controller.md)

## Digital Communication Protocol
Almost every sensor, communicator, or driver electronic on the Icarus uses a digital communication protocol for sending or recieving data. Since this is true, we had to use various libraries for communicating in different protocols, including:
- CAN
- USB
- I²C
- UART

## Docker Environment 
They're two ways to use the docker:
1. Using the Dockerfile 
2. Using the [Github Registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-docker-registry)



### Dockerfile
The following is using the Dockerfile. This takes longer due to building, but if you would like to add any dev tools using the Dockerfile would be more helpful. 

Command to build docker
```sh 
docker-compose build dev #or docker compose ... depending on your version
```

Command to run docker
```sh
docker-compose run --rm dev bash
```

### GitHub Registry 
Using the GitHub Registory includes the prebuilt developement image hosted on Github Container Registry (GHCR):
```bash
ghcr.io/machworksvt/machpilot-dev:latest
```
The recommended workflow is:
* Pull the prebuilt image 
* Run it with Docker Compose

1. Prerequisites 
* Docker Enginer installed 
* GitHub account with access to the machworksvt org

2. Authenticate Docker with GHCR
Docker must be authenticated to pull images from GHCR

2.1 Create a GitHub Personal Access Token (PAT)
Create a token with:
* read:packages (required)
* repo (optional)

2.2 Login to GHCR with Docker
```bash
export CR_PAT=<YOUR_TOKEN>
```
```bash
$ echo $CR_PAT | docker login ghcr.io -u USERNAME --password-stdin
> Login Succeeded
```

3. Pull the Development Image
```bash 
docker pull ghcr.io/machworksvt/machpilot-dev:latest
```
Verify:
```bash 
docker images | grep machpilot 
```

4. Running the Dev Environment 
This repo provides a ghcr.compose.yaml that runs the pulled image wth the correct mounts and settings 

```bash
docker-compose -f ghcr.compose.yml run --rm dev
```

5. Common Issues
* unauthorized when pulling:
    * Not logged into ghcr.io
    * Token missing read::packages
    * using sudo docker without logging in as root

* permission denied /var/run/docker.sock
    * Add your user to the docker group:
    ```bash
    sudo usermod -aG docker $USER
    newgrp docker
    ```