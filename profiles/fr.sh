#!/bin/bash
idf.py \
  -DDEVICE_LOCATION=FR \
  -DDEVICE_WLAN_SSID=fiesta-network \
  -DDEVICE_WLAN_PWD=fiesta-network-123 \
  -DDEVICE_MQTT_URI=mqtt://broker:1883 \
  "$@"
