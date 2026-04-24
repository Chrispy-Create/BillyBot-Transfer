/**
 * BillyBot Pico 2 W Firmware
 *
 * - micro-ROS agent reconnection loop (robust to agent restarts)
 * - GPS: reads NEO-6M on UART1 (GPIO4 TX, GPIO5 RX), publishes sensor_msgs/NavSatFix on /gps/fix
 *
 * UART0 (GPIO0/1) is reserved by pico_uart_transports for the micro-ROS agent link.
 * UART1 (GPIO4/5) is used for the GPS module.
 *
 * NOTE: When the motor hall-sensor differential receiver arrives, add:
 *   - nav_msgs/Odometry publisher on /odom
 *   - geometry_msgs/Twist subscriber on /cmd_vel
 *   Motor pole pairs go in MOTOR_POLE_PAIRS below once known.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico_uart_transports.h"

// ─── Hardware config ────────────────────────────────────────────────────────
#define LED_PIN          25

#define GPS_UART         uart1
#define GPS_BAUD         9600
#define GPS_TX_PIN       4    // not connected to GPS, but must be claimed
#define GPS_RX_PIN       5    // GPS TX → Pico GP5

// ─── Motor constants (fill in when differential receiver arrives) ────────────
// #define MOTOR_POLE_PAIRS  7     // example; measure or check datasheet
// #define GEAR_RATIO        15.0f
// #define WHEEL_DIAMETER_M  0.190f

// ─── Agent state machine ─────────────────────────────────────────────────────
typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} agent_state_t;

// ─── micro-ROS entities ───────────────────────────────────────────────────────
static rcl_publisher_t      gps_publisher;
static rcl_node_t           node;
static rcl_allocator_t      allocator;
static rclc_support_t       support;
static rclc_executor_t      executor;

static sensor_msgs__msg__NavSatFix gps_msg;
static char gps_frame_str[] = "gps_link";

// ─── GPS state ────────────────────────────────────────────────────────────────
#define NMEA_BUF_SIZE 256
static char   nmea_buf[NMEA_BUF_SIZE];
static int    nmea_pos = 0;

static double gps_lat   = 0.0;
static double gps_lon   = 0.0;
static bool   gps_valid = false;

// ─── NMEA helpers ─────────────────────────────────────────────────────────────

/* Convert DDMM.MMMMM + direction to signed decimal degrees */
static double nmea_to_decimal(const char *raw, char dir) {
    double val = atof(raw);
    int    deg = (int)(val / 100);
    double min = val - deg * 100.0;
    double dec = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dec = -dec;
    return dec;
}

/**
 * Parse a $GPRMC or $GNRMC sentence (modifies the string in-place).
 * Returns true if a valid fix was decoded.
 */
static bool parse_rmc(char *s) {
    /* Only handle RMC */
    if (strncmp(s, "$GPRMC", 6) != 0 && strncmp(s, "$GNRMC", 6) != 0)
        return false;

    /* Strip checksum */
    char *star = strchr(s, '*');
    if (star) *star = '\0';

    /* Tokenise on commas */
    char *fields[13] = {0};
    int   n = 0;
    char *p = s;
    while (p && n < 13) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (p) *p++ = '\0';
    }

    if (n < 7) return false;

    /* Field 2: status */
    if (!fields[2] || fields[2][0] != 'A') {
        gps_valid = false;
        return false;
    }

    /* Fields 3/4: lat, fields 5/6: lon */
    if (!fields[3] || !fields[4] || !fields[5] || !fields[6]) return false;
    if (strlen(fields[3]) < 4 || strlen(fields[5]) < 4) return false;

    gps_lat   = nmea_to_decimal(fields[3], fields[4][0]);
    gps_lon   = nmea_to_decimal(fields[5], fields[6][0]);
    gps_valid = true;
    return true;
}

/* Call this in the main loop — reads whatever bytes are waiting */
static void poll_gps(void) {
    while (uart_is_readable(GPS_UART)) {
        char c = (char)uart_getc(GPS_UART);

        if (c == '$') nmea_pos = 0;          /* start of sentence */

        if (nmea_pos < NMEA_BUF_SIZE - 1)
            nmea_buf[nmea_pos++] = c;

        if (c == '\n') {
            nmea_buf[nmea_pos] = '\0';
            char copy[NMEA_BUF_SIZE];
            strncpy(copy, nmea_buf, NMEA_BUF_SIZE);
            parse_rmc(copy);
            nmea_pos = 0;
        }
    }
}

// ─── micro-ROS entity lifecycle ───────────────────────────────────────────────

static bool create_entities(void) {
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&node, "billybot_pico", "", &support) != RCL_RET_OK)
        return false;

    if (rclc_publisher_init_default(
            &gps_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
            "gps/fix") != RCL_RET_OK)
        return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
        return false;

    /* Wire up the static frame_id string */
    gps_msg.header.frame_id.data     = gps_frame_str;
    gps_msg.header.frame_id.size     = strlen(gps_frame_str);
    gps_msg.header.frame_id.capacity = sizeof(gps_frame_str);
    gps_msg.status.service           = 1;  /* SERVICE_GPS */

    return true;
}

static void destroy_entities(void) {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    rcl_publisher_fini(&gps_publisher, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

static void publish_gps(void) {
    gps_msg.header.stamp.sec     = 0;
    gps_msg.header.stamp.nanosec = 0;

    if (gps_valid) {
        gps_msg.status.status          = 0;     /* STATUS_FIX */
        gps_msg.latitude               = gps_lat;
        gps_msg.longitude              = gps_lon;
        gps_msg.altitude               = 0.0;
        /* ~3 m std-dev for a consumer NEO-6M */
        gps_msg.position_covariance[0] = 9.0;
        gps_msg.position_covariance[4] = 9.0;
        gps_msg.position_covariance[8] = 100.0;
        gps_msg.position_covariance_type = 1;  /* COVARIANCE_TYPE_APPROXIMATED */
    } else {
        gps_msg.status.status            = -1;  /* STATUS_NO_FIX */
        gps_msg.latitude                 = 0.0;
        gps_msg.longitude                = 0.0;
        gps_msg.altitude                 = 0.0;
        gps_msg.position_covariance_type = 0;   /* COVARIANCE_TYPE_UNKNOWN */
    }

    rcl_publish(&gps_publisher, &gps_msg, NULL);
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(void) {
    /* micro-ROS UART0 transport (handled by pico_uart_transports) */
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    /* GPS on UART1 */
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);

    /* LED */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    agent_state_t state = WAITING_AGENT;
    uint32_t      last_gps_pub_ms = 0;

    while (true) {
        /* GPS polling runs regardless of ROS connection state */
        poll_gps();

        switch (state) {

        case WAITING_AGENT:
            /* Blink LED slowly while searching */
            gpio_put(LED_PIN, (to_ms_since_boot(get_absolute_time()) / 500) & 1);
            if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
                state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                gpio_put(LED_PIN, 1);   /* solid on = connected */
                state = AGENT_CONNECTED;
            } else {
                destroy_entities();
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED: {
            /* Check agent is still alive before spinning */
            if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
                state = AGENT_DISCONNECTED;
                break;
            }

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

            /* Publish GPS at 1 Hz */
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - last_gps_pub_ms >= 1000) {
                publish_gps();
                last_gps_pub_ms = now_ms;
            }
            break;
        }

        case AGENT_DISCONNECTED:
            gpio_put(LED_PIN, 0);
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }

    return 0;
}
