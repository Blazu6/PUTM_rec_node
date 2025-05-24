## 1. Utworzenie skryptu nagrywającego rosbag.sh

```bash
mkdir -p ~/ścieżka do pliku
```

## 2. Nadanie uprawnień do wykonania skryptu
```bash
chmod +x ~/sciezka/rosbag.sh
```
## 3. Utworzenie pliku usługi systemd
```bash
sudo nano /etc/systemd/system/rosbag.service
```
## 4. Przeładowanie konfiguracji i uruchomienie usługi
```bash
sudo systemctl daemon-reload
sudo systemctl start rosbag.service
sudo systemctl status rosbag.servic
```

## 5. Włączenie automatycznego uruchamiania usługi
```bash
sudo systemctl enable rosbag.service
```
## 6. Podstawowe komendy zarządzania usługą rosbag
| Komenda                                | Opis                     |
|-----------------------------------     |--------------------------|
| sudo systemctl start rosbag.service    | Uruchom usługę           |
| sudo systemctl stop rosbag.service     | Zatrzymaj usługę         |
| sudo systemctl restart rosbag.service  | Restartuj usługę         |
| sudo systemctl status rosbag.service   | Sprawdź status usługi    |
| sudo systemctl enable rosbag.service   | Włącz auto-start usługi  |
| sudo systemctl disable rosbag.service  | Wyłącz auto-start usługi |

