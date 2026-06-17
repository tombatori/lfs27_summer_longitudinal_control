#docker
xhost +local:docker
# db() {
#     mkdir DockerBuild
#     docker build -f Dockerfile -t slamdog DockerBuild
# }

alias dr_lcsp="sudo docker run -it --privileged -e DISPLAY=$DISPLAY -p 8765:8765 -v /tmp/.X11-unix:/tmp/.X11-unix -v ~:/workspace slamdog bash"
da_lcsp() {
    sudo docker exec -it $(sudo docker ps -l -q) bash
}