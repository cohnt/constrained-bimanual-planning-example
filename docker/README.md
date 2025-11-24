# Docker Instructions

This guide explains how to use the Docker image for the Constrained Bimanual Planning Example.
The Docker image comes fully configured with Drake, all Python packages, and C++ build tools, so you can run both Python and C++ examples out-of-the-box.

## Downloading the Docker Image

**TODO**

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

## Building the Docker Image Yourself

If you want to build the Docker image locally, run
```
docker build -t constrained-bimanual-planning-example:<your-tag> .
```