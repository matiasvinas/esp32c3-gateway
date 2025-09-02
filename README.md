# Sistema de monitoreo y gestión remota de invernaderos
Proyecto realizado dentro del marco del Trabajo Profesional de Ingeniería Electrónica de la Facultad de Ingeniería de la Universidad de Buenos Aires

## Contenido

Este repositorio contiene la descripción completa del sistema, el firmware del nodo *gateway* y su configuración. 

Para ver la descripción del sistema, referirse a [Sistema de monitoreo y gestión remota de invernaderos](https://github.com/matiasvinas/or-platform).

## Índice

- [Nodo *gateway*](#nodo-gateway)
  - [Características](#características)
  - [Configuración Wi-Fi del nodo *gateway*](#configuración-wi-fi-del-nodo-gateway)
  - [Configuración para la comunicación MQTT sobre SSL](#configuración-para-la-comunicación-mqtt-sobre-ssl-entre-el-nodo-gateway-y-la-plataforma-web)
  - [Configuración para la comunicación BLE Mesh entre el nodo *gateway* y los nodos sensores](#configuración-para-la-comunicación-ble-mesh-entre-el-nodo-gateway-y-los-nodos-sensores)
  - [Conexión del control del actuador a la placa de desarrollo](#conexión-del-control-del-actuador-a-la-placa-de-desarrollo)
- [Puesta en marcha del sistema de sensores](#puesta-en-marcha-del-sistema-de-sensores)
- [Enlaces útiles](#enlaces-útiles)

# Nodo *gateway*

## Características
- Placa de desarrollo: ESP32-C3-WROOM-02 de la empresa [Espressif](https://www.espressif.com/).
- Framework: ESP-IDF v5.2.2.
- Memoria Flash: 4MB.
- *Partition table* implementada:

    |Name|Type|Subtype|Offset|Size|
    |----|----|-------|------|----|
    |nvs |data|nvs    |0x9000|24K |
    |phy_init|data|phy|0xf000|4K |
    |factory|app|factory|0x10000|4M|

## Configuración Wi-Fi del nodo *gateway*

1. Abrir el directorio del proyecto y correr el siguiente comando:
    ```
    idf.py menuconfig
    ```
2. Navegar a la sección de configuración Wi-Fi llamada `Example Connection Configuration`
3. Completar los campos `WiFi SSID` con el nombre de la red y `WiFi Password` con la contraseña correspondiente.
4. Guardar cambios y salir.


## Configuración para la comunicación MQTT sobre SSL entre el nodo *gateway* y la plataforma web

0. Requisitos previos:

    - Plataforma de Open Remote hosteada en dominio.
    - Certificado SSL válido del dominio.
    - Creación del *asset* nodo gateway en la plataforma.
    - Creación de los *assets* de los nodos sensores en la plataforma.

1. Creación del *service user* y uso del *broker* MQTT: 
    1. En la plataforma Open Remote, ir a la sección de *Users* y seleccionar *"Add service user"*.
    2. Ingresar un *username*.
    3. En el campo *realm role*, seleccionar "*admin user*".
    4. Vincular el *service user* al *asset* nodo gateway.

2. Creación del certificado PEM:

    1. Descargar el certificado SSL de la plataforma web.
    2. Obtener y descargar la cadena raíz de certificados de la Autoridad Certificante.
    3. Crear un archivo de extensión ".pem" con toda la cadena de certificados.
    4. Renombrar el archivo a `or_fiuba_tpp.pem` y guardarlo dentro de la carpeta `/main`.

3. Configuración del cliente MQTT:
    1. Obtener el *username* y la contraseña del *service user* creado en anteriormente. 
    2. Configurar la estructura de datos `mqtts_cfg` del archivo `main/main.c`. Para mayor información consultar la documentación oficial [MQTT Broker Open Remote](https://docs.openremote.io/docs/user-guide/manager-apis#mqtt-api-mqtt-broker).

        ```
        const esp_mqtt_client_config_t mqtts_cfg = {
                .broker.address.transport = MQTT_TRANSPORT_OVER_SSL,
                .broker.address.port = 8883,
                .broker.address.path = "{hostname}",
                .broker.address.hostname = "{hostname}",
                .broker.verification.skip_cert_common_name_check = false,
                .broker.verification.certificate = (const char *)or_fiuba_tpp_pem_start,
                .credentials.client_id = "{clientid}",
                .credentials.authentication.password = "{secret}",
                .credentials.username = "{realm}:{username}"
            };
        ```

4. Configuración de los tópicos de los nodos:
    1. Agregar los tópicos de los sensores a la estructura de datos `or_things[]` del archivo `main/main.c`. Para mayor información consultar la documentacion oficial [MQTT Broker Open Remote](https://docs.openremote.io/docs/user-guide/manager-apis#mqtt-api-mqtt-broker).
        ```
        static openremote_thing_t or_things[3] = {
            [0] = {
                .id = 0x01,
                .topic_temp_val = "master/{clientid}/writeattributevalue/temperatura/{sensor1token}",
                .topic_mois_val = "master/{clientid}/writeattributevalue/humedad_suelo/{sensor1token}",
                .topic_battery_val = "master/{clientid}/writeattributevalue/bateria/{sensor1token}",
                .topic_connection = "master/{clientid}/writeattributevalue/nodo_sensor_1_conectado/{gatewaytoken}",
            },
            [1] = {
                .id = 0x02,
                .topic_temp_val = "master/{clientid}/writeattributevalue/temperatura/{sensor2token}",
                .topic_mois_val = "master/{clientid}/writeattributevalue/humedad_suelo/{sensor2token}",
                .topic_battery_val = "master/{clientid}/writeattributevalue/bateria/{sensor2token}",
                .topic_connection = "master/{clientid}/writeattributevalue/nodo_sensor_2_conectado/{gatewaytoken}",
            },
            [2] = {
                .id = 0x03,
                .topic_temp_val = "master/{clientid}/writeattributevalue/temperatura/{sensor3token}",
                .topic_mois_val = "master/{clientid}/writeattributevalue/humedad_suelo/{sensor3token}",
                .topic_battery_val = "master/{clientid}/writeattributevalue/bateria/{sensor3token}",
                .topic_connection = "master/{clientid}/writeattributevalue/nodo_sensor_3_conectado/{gatewaytoken}",	
            }	
        };
        ```
    2. Agregar los tópicos de los actuadores a `topic_frequency_str[]` y `topic_irrigation_str[]` del archivo `main/main.c`.
        ```
        //MQTT Topics handled by Gateway
        const char topic_frequency_str[] = "master/{clientid}/attributevalue/frecuencia/{gatewaytoken}";
        const char topic_irrigation_str[] = "master/{clientid}/attributevalue/riego_activado/{gatewaytoken}";
        ```

## Configuración para la comunicación BLE Mesh entre el nodo *gateway* y los nodos sensores

1. Selección del stack de NimBLE
    1. Abrir el directorio del proyecto y correr el siguiente comando:
        ```
        idf.py menuconfig
        ```
    2. Navegar a `Component Config -> Bluetooth -> Bluetooth -> Host`
    3. En `Host` seleccionar `NimBLE - BLE Only`.
    4. Guardar cambios y salir.

2. Definición de ID de la malla y de los nodos sensores
    1. Definir `SENSOR_ID_MESH_0` y `SENSOR_ID_MESH_1` en el archivo `main/main.c` para identificar a todos los nodos que deben ser aprovisionados por el nodo *gateway*. Este ID debe coincidir con el ID definido para la malla en el nodo sensoer ([esp32c3-sensor](https://github.com/matiasvinas/esp32c3-sensor)).

    2. Definir `SENSOR_ID_NODE_1`, `SENSOR_ID_NODE_2` y `SENSOR_ID_NODE_3` en el archivo `main/main.c` para cada uno de los nodos sensores. Estos valores deben coincidir con los valores definidos en el firmware de los nodos sensores. 

        ```
        /* ID de cada nodo sensor */
        #define SENSOR_ID_NODE_1         0x01
        #define SENSOR_ID_NODE_2         0x02
        #define SENSOR_ID_NODE_3         0x03
        /* ID de la malla */
        #define SENSOR_ID_MESH_0         0x32    
        #define SENSOR_ID_MESH_1         0x10
        ```
## Conexión del control del actuador a la placa de desarrollo

El dispositivo nodo *gateway* controla el actuador a través del pin GPIO 04. El valor lógico del pin puede ser controlado desde la plataforma Open Remote.



## Puesta en marcha del sistema de sensores
1. Encender el nodo *gateway* y verificar que se encuentre vinculado con el *broker* MQTT desde el *service user* creado en la plataforma Open Remote.
2. Encender cada uno de los nodos sensores de forma sencuencial y verificar que se encuentran conectados desde la plataforma web. 

## Enlaces útiles

[esp32c3-sensor](https://github.com/matiasvinas/esp32c3-sensor)


[ESP-BLE-MESH - Sensor Server Client Example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/sensor_models/sensor_client/README.md)


[ESP-BLE-MESH - Provisioner Example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/provisioner/README.md)

[ESP-BLE-MESH - Wi-Fi Coexistence Example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/wifi_coexist/README.md)


[ESP-MQTT SSL Sample Application](https://github.com/espressif/esp-idf/blob/master/examples/protocols/mqtt/ssl/README.md)


[Open Remote - Documentation](https://docs.openremote.io/docs/introduction)


[Open Remote - Broker MQTT](https://docs.openremote.io/docs/user-guide/manager-apis#mqtt-api-mqtt-broker)


