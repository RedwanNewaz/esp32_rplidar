#pragma once 
#include <Arduino.h>
#include <WiFi.h>

static IPAddress agent_ip(192, 168, 10, 36);
static size_t agent_port = 8888;

static char ssid[] = "AirLabRobots24";
static char psk[]= "unoRobotics";