# Szczegółowe Zestawienie Publisherów i Subskrybentów z Ustawieniami QoS

Poniżej znajduje się szczegółowa lista wszystkich znalezionych publisherów i subskrybentów w projekcie, wraz z ich tematami, typami wiadomości i ustawieniami QoS, posortowana według plików.

Dla wszystkich poniższych wpisów, jeśli nie zaznaczono inaczej, domyślne ustawienia QoS to:
*   **Polityka Historii:** `KEEP_LAST`
*   **Polityka Niezawodności:** `RELIABLE` (wiadomości są gwarantowane do dostarczenia, z ponownymi próbami w razie potrzeby)
*   **Polityka Trwałości:** `VOLATILE` (subskrybenci otrzymują tylko te wiadomości, które zostały opublikowane w czasie ich aktywności)
*   **Polityka Żywotności (Liveliness):** `AUTOMATIC`
*   **Termin (Deadline):** Nieskończony
*   **Czas Życia (Lifespan):** Nieskończony

---
### **Plik: `putm_controller/src/controller.cpp`**
#### Publishery:
*   **`setpoints_publisher`**
    *   **Temat:** `"putm_vcl/setpoints"`
    *   **Typ Wiadomości:** `Setpoints`
    *   **QoS (Głębokość Historii):** `1`
*   **`yaw_rate_ref_publisher`**
    *   **Temat:** `"yaw_ref"`
    *   **Typ Wiadomości:** `YawRef`
    *   **QoS (Głębokość Historii):** `1`

#### Subskrypcje:
*   **`frontbox_driver_input_subscriber`**
    *   **Temat:** `"putm_vcl/frontbox_driver_input"`
    *   **Typ Wiadomości:** `FrontboxDriverInput`
    *   **QoS (Głębokość Historii):** `1`
*   **`amk_front_left_actual_values1_subscriber`**
    *   **Temat:** `"putm_vcl/amk/front/left/actual_values1"`
    *   **Typ Wiadomości:** `AmkActualValues1`
    *   **QoS (Głębokość Historii):** `1`
*   **`amk_front_right_actual_values1_subscriber`**
    *   **Temat:** `"putm_vcl/amk/front/right/actual_values1"`
    *   **Typ Wiadomości:** `AmkActualValues1`
    *   **QoS (Głębokość Historii):** `1`
*   **`amk_rear_left_actual_values1_subscriber`**
    *   **Temat:** `"putm_vcl/amk/rear/left/actual_values1"`
    *   **Typ Wiadomości:** `AmkActualValues1`
    *   **QoS (Głębokość Historii):** `1`
*   **`amk_rear_right_actual_values1_subscriber`**
    *   **Temat:** `"putm_vcl/amk/rear/right/actual_values1"`
    *   **Typ Wiadomości:** `AmkActualValues1`
    *   **QoS (Głębokość Historii):** `1`
*   **`xsens_acceleration_ay_subscriber`**
    *   **Temat:** `"putm_vcl/xsens_acceleration"`
    *   **Typ Wiadomości:** `XsensAcceleration`
    *   **QoS (Głębokość Historii):** `1`
*   **`xsens_acceleration_ax_subscriber`**
    *   **Temat:** `"putm_vcl/xsens_acceleration"`
    *   **Typ Wiadomości:** `XsensAcceleration`
    *   **QoS (Głębokość Historii):** `1`
*   **`xsens_rate_of_turn_subscriber`**
    *   **Temat:** `"putm_vcl/xsens_rate_of_turn"`
    *   **Typ Wiadomości:** `XsensRateOfTurn`
    *   **QoS (Głębokość Historii):** `1`
*   **`vn300_rate_of_turn_subscriber`**
    *   **Temat:** `"vectornav/raw/imu"`
    *   **Typ Wiadomości:** `vectornav_msgs::msg::ImuGroup`
    *   **QoS (Głębokość Historii):** `1`
*   **`bms_hv_main_subscriber`**
    *   **Temat:** `"putm_vcl/bms_hv_main"`
    *   **Typ Wiadomości:** `BmsHvMain`
    *   **QoS (Głębokość Historii):** `1`

---
### **Plik: `putm_lap_timer/src/lap_timer.cpp`**
#### Publishery:
*   **`lap_timer_pub`**
    *   **Temat:** `"/putm_vcl/lap_timer"`
    *   **Typ Wiadomości:** `putm_vcl_interfaces::msg::LapTimer`
    *   **QoS (Głębokość Historii):** `50`
*   **`lap_timer_delta_pub_`**
    *   **Temat:** `"/putm_vcl/lap_timer/delta"`
    *   **Typ Wiadomości:** `std_msgs::msg::UInt16`
    *   **QoS (Głębokość Historii):** `50`
*   **`lap_timer_time_pub_`**
    *   **Temat:** `"/putm_vcl/lap_timer/time"`
    *   **Typ Wiadomości:** `geometry_msgs::msg::Vector3`
    *   **QoS (Głębokość Historii):** `50`

#### Subskrypcje:
*   **`gps_sub_`**
    *   **Temat:** `"/vectornav/gnss"`
    *   **Typ Wiadomości:** `sensor_msgs::msg::NavSatFix`
    *   **QoS (Głębokość Historii):** `50`

---
### **Plik: `putm_vcl/putm_vcl/src/can_nodes/can_rx_node.cpp`**
#### Publishery (wszystkie z QoS o głębokości historii `1`):
*   **`frontbox_driver_input_publisher`** (Typ: `msg::FrontboxDriverInput`, Temat: `frontbox_driver_input`)
*   **`frontbox_data_publisher`** (Typ: `msg::FrontboxData`, Temat: `frontbox_data`)
*   **`bms_hv_main_publisher`** (Typ: `msg::BmsHvMain`, Temat: `bms_hv_main`)
*   **`bms_lv_main_publisher`** (Typ: `msg::BmsLvMain`, Temat: `bms_lv_main`)
*   **`amk_front_left_actual_values1_publisher`** (Typ: `msg::AmkActualValues1`, Temat: `amk/front/left/actual_values1`)
*   **`amk_front_left_actual_values2_publisher`** (Typ: `msg::AmkActualValues2`, Temat: `amk/front/left/actual_values2`)
*   **`amk_front_right_actual_values1_publisher`** (Typ: `msg::AmkActualValues1`, Temat: `amk/front/right/actual_values1`)
*   **`amk_front_right_actual_values2_publisher`** (Typ: `msg::AmkActualValues2`, Temat: `amk/front/right/actual_values2`)
*   **`amk_rear_left_actual_values1_publisher`** (Typ: `msg::AmkActualValues1`, Temat: `amk/rear/left/actual_values1`)
*   **`amk_rear_left_actual_values2_publisher`** (Typ: `msg::AmkActualValues2`, Temat: `amk/rear/left/actual_values2`)
*   **`amk_rear_right_actual_values1_publisher`** (Typ: `msg::AmkActualValues1`, Temat: `amk/rear/right/actual_values1`)
*   **`amk_rear_right_actual_values2_publisher`** (Typ: `msg::AmkActualValues2`, Temat: `amk/rear/right/actual_values2`)
*   **`dashboard_publisher`** (Typ: `msg::Dashboard`, Temat: `dashboard`)
*   **`xsens_acceleration_publisher`** (Typ: `msg::XsensAcceleration`, Temat: `xsens_acceleration`)
*   **`xsens_temp_and_pressure_publisher`** (Typ: `msg::XsensTempAndPressure`, Temat: `xsens_temp`)
*   **`xsens_utc_publisher`** (Typ: `msg::XsensUtc`, Temat: `xsens_utc`)
*   **`xsens_euler_publisher`** (Typ: `msg::XsensEuler`, Temat: `xsens_euler_publisher`)
*   **`xsens_rate_of_turn_publisher`** (Typ: `msg::XsensRateOfTurn`, Temat: `xsens_rate_of_turn`)
*   **`xsens_orientation_publisher`** (Typ: `msg::XsensOrientation`, Temat: `xsens_orientation`)
*   **`xsens_velocity_publisher`** (Typ: `msg::XsensVelocity`, Temat: `xsens_velocity`)
*   **`xsens_inertial_data_publisher`** (Typ: `msg::XsensInertialData`, Temat: `xsens_dv`)
*   **`xsens_position_publisher`** (Typ: `msg::XsensPosition`, Temat: `xsens_position`)

---
### **Plik: `putm_vcl/putm_vcl/src/amk_node/amk_node.cpp`**
#### Publishery (wszystkie z QoS o głębokości historii `1`):
*   **`amk_front_left_setpoints_publisher`** (Typ: `msg::AmkSetpoints`, Temat: `amk/front/left/setpoints`)
*   **`amk_front_right_setpoints_publisher`** (Typ: `msg::AmkSetpoints`, Temat: `amk/front/right/setpoints`)
*   **`amk_rear_left_setpoints_publisher`** (Typ: `msg::AmkSetpoints`, Temat: `amk/rear/left/setpoints`)
*   **`amk_rear_right_setpoints_publisher`** (Typ: `msg::AmkSetpoints`, Temat: `amk/rear/right/setpoints`)

#### Subskrypcje (wszystkie z QoS o głębokości historii `1`):
*   Subskrypcja na temat **`amk/front/left/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/front/right/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/rear/left/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/rear/right/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   **`rtd_subscriber`** (Typ: `msg::Rtd`, Temat: `rtd`)
*   **`setpoints_subscriber`** (Typ: `msg::Setpoints`, Temat: `setpoints`)

---
### **Plik: `putm_vcl/putm_vcl/src/can_nodes/can_tx_node.cpp`**
#### Subskrypcje (wszystkie z QoS o głębokości historii `1`):
*   **`amk_front_left_setpoints_subscriber`** (Typ: `msg::AmkSetpoints`, Temat: `amk/front/left/setpoints`)
*   **`amk_front_right_setpoints_subscriber`** (Typ: `msg::AmkSetpoints`, Temat: `amk/front/right/setpoints`)
*   **`amk_rear_left_setpoints_subscriber`** (Typ: `msg::AmkSetpoints`, Temat: `amk/rear/left/setpoints`)
*   **`amk_rear_right_setpoints_subscriber`** (Typ: `msg::AmkSetpoints`, Temat: `amk/rear/right/setpoints`)
*   **`rtd_subscriber`** (Typ: `msg::Rtd`, Temat: `rtd`)

---
### **Plik: `putm_vcl/putm_vcl/src/rtd_node/rtd_node.cpp`**
#### Publishery:
*   **`rtd_publisher`**
    *   **Temat:** `"rtd"`
    *   **Typ Wiadomości:** `msg::Rtd`
    *   **QoS (Głębokość Historii):** `1`

#### Subskrypcje (wszystkie z QoS o głębokości historii `1`):
*   **`frontbox_driver_input_subscription`** (Typ: `msg::FrontboxDriverInput`, Temat: `frontbox_driver_input`)
*   **`dashboard_subscription`** (Typ: `msg::Dashboard`, Temat: `dashboard`)
*   Subskrypcja na temat **`amk/front/left/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/front/right/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/rear/left/actual_values1`** (Typ: `msg::AmkActualValues1`)
*   Subskrypcja na temat **`amk/rear/right/actual_values1`** (Typ: `msg::AmkActualValues1`)

---
### **Plik: `putm_vcl/putm_vcl/src/rtd_sub_node/rtd_sub_node.cpp`**
#### Subskrypcje:
*   **`subscription_`**
    *   **Temat:** `"rtd"`
    *   **Typ Wiadomości:** `msg::Rtd`
    *   **QoS (Głębokość Historii):** `10`
