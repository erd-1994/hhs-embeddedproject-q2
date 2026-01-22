/* AHT20 and BMP280 topics */
#define MQTT_TOPIC_AHT20_TEMP "sensors/aht20/temperature_C"
#define MQTT_TOPIC_AHT20_HUM "sensors/aht20/humidity_RH"
#define MQTT_TOPIC_BM280_TEMP "sensors/bm280/temperature_C"
#define MQTT_TOPIC_BM280_PRESS "sensors/bm280/pressure_Pa"

/* SPS30 Particulate Matter topics (mass concentration in ug/m3) */
#define MQTT_TOPIC_SPS30_PM1 "sensors/sps30/pm1_ugm3"
#define MQTT_TOPIC_SPS30_PM25 "sensors/sps30/pm25_ugm3"
#define MQTT_TOPIC_SPS30_PM4 "sensors/sps30/pm4_ugm3"
#define MQTT_TOPIC_SPS30_PM10 "sensors/sps30/pm10_ugm3"

/* SPS30 Number Concentration topics (particles per cm3) */
#define MQTT_TOPIC_SPS30_NC05 "sensors/sps30/nc05_percm3"
#define MQTT_TOPIC_SPS30_NC1 "sensors/sps30/nc1_percm3"
#define MQTT_TOPIC_SPS30_NC25 "sensors/sps30/nc25_percm3"
#define MQTT_TOPIC_SPS30_NC4 "sensors/sps30/nc4_percm3"
#define MQTT_TOPIC_SPS30_NC10 "sensors/sps30/nc10_percm3"

/* SPS30 Typical Particle Size topic (in micrometers) */
#define MQTT_TOPIC_SPS30_TPS "sensors/sps30/typical_size_um"

/* Subscription Topics for Updates */
#define TOPIC_UPDATE_TEMP_HIGH "update/threshold_high/temperature"
#define TOPIC_UPDATE_TEMP_LOW "update/threshold_low/temperature"
#define TOPIC_UPDATE_HUM_HIGH "update/threshold_high/humidity"
#define TOPIC_UPDATE_HUM_LOW "update/threshold_low/humidity"
#define TOPIC_UPDATE_PM25_HIGH "update/threshold_high/pm25"

/* ==================== THRESHOLD STATE TOPICS ==================== */
/* These are the topics where the ESP32 will publish the current threshold
 * values */
#define TOPIC_STATE_TEMP_HIGH "state/threshold_high/temperature"
#define TOPIC_STATE_TEMP_LOW "state/threshold_low/temperature"
#define TOPIC_STATE_HUM_HIGH "state/threshold_high/humidity"
#define TOPIC_STATE_HUM_LOW "state/threshold_low/humidity"
#define TOPIC_STATE_PM25_HIGH "state/threshold_high/pm25"
