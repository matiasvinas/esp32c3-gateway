# Sistema de monitoreo y gestión remota de invernaderos - Gateway
Proyecto realizado dentro del marco del Trabajo Profesional de Ingeniería Eletrónica de la Facultad de Ingeniería de la Universidad de Buenos Aires

## Contenido 
Este repositorio contiene el firmware del dispositivo del gateway del sistema.

## Características Técnicas
- Microcontrolador: ESP32-C3-WROOM-02 de la empresa [Espressif](https://www.espressif.com/)
- Framework: ESP-IDF

## Bluetooh

El dispositivo usa Bluetooth Mesh para la comunicación con el [esp32c3-sensor](https://github.com/matiasvinas/esp32c3-sensor)

## MQTT

El dispositivo actua como MQTT Client interactuando con el MQTT Broker de OpenRemote corriendo de forma local. Se utiliza el puerto 1883

## Notas
- Para la conexión WIFI se utilizó la configuración de ejemplo del esp-idf framwork. Se la puede configurar en el archivo sdkconfig.
- Se creó una "custom partition table" y se modificó el el tamaño de memoria flash de 2MB a 4MB.

|Name|Type|Subtype|Offset|Size|
|----|----|-------|------|----|
|nvs |data|nvs    |0x9000|24K |
|phy_init|data|phy|0xf000|4K |
|factory|app|factory|0x10000|4M|

## Enlaces útiles

[ESP-BLE-MESH - Sensor Server Client Example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/sensor_models/sensor_client/README.md)