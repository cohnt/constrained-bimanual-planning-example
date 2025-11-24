# Docker Instructions

This guide explains how to use the Docker image for the Constrained Bimanual Planning Example.
The Docker image comes fully configured with Drake, all Python packages, and C++ build tools, so you can run both Python and C++ examples out-of-the-box.

## Downloading the Docker Image

The Docker image is available on [Docker Hub](https://hub.docker.com/repository/docker/cohnt/constrained-bimanual-planning-example/general):
```
docker pull cohnt/constrained-bimanual-planning-example:latest
```

## Launching and Using the Docker Image

You can launch an interactive session with the Docker image, exposing the necessary ports for Jupyter (8888) and Meshcat (7000):
```
docker run -it --rm \
    -p 8888:8888 \
    -p 7000:7000 \
    constrained-bimanual-planning-example:latest \
    /bin/bash
```
*Note:* The `--rm` flag will remove the container when it exits.
If you want to keep files inside the container, omit this flag or copy the files before exiting.

Once inside the container, you can start a Jupyter notebook:
```
jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser --allow-root
```
Open the URL shown in the terminal to interact with the notebook.
You can run both the Python and C++ examples directly in the container.
Once you've launched the notebook, Meshcat visualization will be available in your web browser at `localhost:7000`.

### Saving Files from the Image

The notebook exports a static HTML file to visualize the robot trajectory.
To save this file, do the following before closing the Docker image:
1) List the running containers with `docker ps`, and find the `CONTAINER_ID` of your running container.
2) Copy the trajectory file over to your host computer with `docker cp <CONTAINER_ID>:/opt/proj/notebooks/trajectory.html .`

### Note for Apple Silicon (M1/M2) Users:

This Docker image is built for x86_64 (Intel/AMD) architecture.
On Apple Silicon Macs, Docker Desktop can emulate x86_64 images, so the image usually works, but performance may be slower.
In rare cases, some binaries may not run correctly.

If you encounter any issues running the Docker image on Apple Silicon, please [open an issue](https://github.com/cohnt/constrained-bimanual-planning-example/issues/new/choose).

## Building the Docker Image Yourself

If you want to build the Docker image locally, run
```
docker build -t constrained-bimanual-planning-example:<your-tag> .
```