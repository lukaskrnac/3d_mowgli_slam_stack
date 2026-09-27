FROM koide3/glim_ros2:jazzy

# CycloneDDS, aby GLIM vedel komunikovať s MowgliNext stackom (rmw_cyclonedds_cpp).
# zenity: dialógy offline viewera (Open Additional Map, Save Map, Export) idú
# cez portable-file-dialogs, ktoré bez zenity potichu vrátia prázdnu cestu.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        zenity \
    && rm -rf /var/lib/apt/lists/*

# Supervisor: kontajner beží stále, GLIM (mapovanie / offline viewer / export)
# sa spúšťa na požiadanie cez ROS služby — ovládané z MowgliNext GUI.
COPY supervisor/glim_supervisor.py /opt/glim_supervisor/glim_supervisor.py
