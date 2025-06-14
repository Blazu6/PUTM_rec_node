#!/bin/bash

# Załaduj środowisko ROS2
source /opt/ros/humble/setup.bash

# Domyślny czas (w sekundach)
DEFAULT_DURATION=1800

# Ścieżka do pliku konfiguracyjnego z czasem trwania
CONFIG_FILE="$(dirname "$0")/rosbag_duration.conf"

# Odczytaj czas trwania z pliku, jeśli istnieje i nie jest pusty
if [[ -f "$CONFIG_FILE" ]] && [[ -s "$CONFIG_FILE" ]]; then
    read -r FILE_DURATION < "$CONFIG_FILE"
    if [[ "$FILE_DURATION" =~ ^[0-9]+$ ]]; then
        DURATION=$FILE_DURATION
        echo "Prawidłowa wartość w $CONFIG_FILE. Używam wartości: $DURATION sekund."
    else
        echo "Nieprawidłowa wartość w $CONFIG_FILE. Używam wartości domyślnej: $DEFAULT_DURATION sekund."
        DURATION=$DEFAULT_DURATION
    fi
else
    echo "Plik $CONFIG_FILE nie istnieje lub jest pusty. Używam wartości domyślnej: $DEFAULT_DURATION sekund."
    DURATION=$DEFAULT_DURATION
fi


BAG_DIR="/home/putm/rosbag_logs"

mkdir -p "$BAG_DIR"

exec ros2 bag record -a --max-bag-duration "$DURATION" -o "$BAG_DIR/rosbag_$(date +%Y-%m-%d_%H-%M-%S)"

