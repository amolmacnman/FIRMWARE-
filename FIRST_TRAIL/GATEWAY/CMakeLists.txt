cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sw_gateway)

# sensor_proto.h lives in src/ (self-contained repo).
# NOTE: gnss.c/.h and ec200.c/.h are reused from the wagen1 GPS->MQTT project.
# The QMTSUB/QMTRECV additions for receiving downlink commands go into ec200.c
# (see ec200_cmd.c for the exact AT commands).
target_sources(app PRIVATE
	src/main.c
	src/motion.c
	src/nodes.c
	src/node_link.c
	src/ble_sensors.c
	src/telemetry.c
	src/thermal.c
	src/gwalarm.c
	src/ota.c
	src/nodeota.c
	src/ec200_cmd.c
	src/power.c
	src/watchdog.c
	src/charger.c
	src/config.c
	src/sw_secure.c
	src/sht40.c
	src/bma400.c
	src/storage.c
	src/gnss.c
	src/ec200.c
	src/modem_simple.c
)
