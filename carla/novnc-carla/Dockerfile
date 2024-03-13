# Use the base image
FROM fredblgr/ubuntu-novnc:20.04

# Set environment variables
ENV DEBIAN_FRONTEND noninteractive

# Update package lists
RUN apt update -y && \
 apt install -y software-properties-common iputils-ping net-tools build-essential && \
 add-apt-repository ppa:deadsnakes/ppa && \
 apt update && \
 apt install -y python3.7 &&\
 wget https://bootstrap.pypa.io/get-pip.py && \
 apt install -y python3.7-distutils python3.7-tk && \
 python3.7 get-pip.py && \
 python3.7 -m pip install pygame numpy opencv-python thonny

RUN \
 echo 'echo export LIBGL_ALWAYS_INDIRECT=0 >> /home/carla/.bashrc' > /etc/startup/libgl.sh && \
 chmod 755  /etc/startup/libgl.sh && \
 echo 'echo tar x -C /workpace -zvf /carla/PythonAPI_Carla_0.9.15.tgz >> /home/carla/.bash_history' >> /etc/startup/history.sh && \
 echo 'echo python3.7 manual_control.py --sync --host 10.6.6.20 >> /home/carla/.bash_history' >> /etc/startup/history.sh && \
 chmod 755  /etc/startup/history.sh

# Expose the port
EXPOSE 80

# Start the container entry point
ENTRYPOINT ["/startup.sh"]
