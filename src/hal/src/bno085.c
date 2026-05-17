#include "bno085.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>

/*
 * BNO085 SHTP (Sensor Hub Transport Protocol) over I2C driver.
 *
 * The BNO085 uses a packet-based protocol over I2C:
 *   - 4-byte SHTP header: [LSB length, MSB length, channel, seq]
 *   - Followed by payload
 *
 * We configure it to output:
 *   - Rotation Vector (report ID 0x05) at requested rate
 *   - Calibrated Gyroscope (report ID 0x02) at requested rate
 */

/* SHTP channels */
#define SHTP_CHAN_COMMAND    0
#define SHTP_CHAN_EXE       1
#define SHTP_CHAN_CONTROL   2
#define SHTP_CHAN_INPUT      3
#define SHTP_CHAN_WAKE_INPUT 4
#define SHTP_CHAN_GYRO_RV   5

/* Report IDs */
#define REPORT_SET_FEATURE    0xFD
#define REPORT_ROTATION_VEC   0x05
#define REPORT_GYRO_CAL       0x02

#define MAX_PACKET_SIZE 128

static int g_fd = -1;
static uint8_t g_seq[6] = {0};  /* Sequence numbers per channel */
static vms_imu_data_t g_latest;

static int i2c_write(const uint8_t *buf, int len)
{
    return (write(g_fd, buf, len) == len) ? 0 : -1;
}

static int i2c_read(uint8_t *buf, int len)
{
    return (read(g_fd, buf, len) == len) ? 0 : -1;
}

static int shtp_send(uint8_t channel, const uint8_t *payload, int payload_len)
{
    uint8_t pkt[MAX_PACKET_SIZE];
    int total = payload_len + 4;

    pkt[0] = total & 0xFF;
    pkt[1] = (total >> 8) & 0xFF;
    pkt[2] = channel;
    pkt[3] = g_seq[channel]++;

    memcpy(&pkt[4], payload, payload_len);
    return i2c_write(pkt, total);
}

static int shtp_recv(uint8_t *buf, int buf_len)
{
    /* Read header first */
    uint8_t hdr[4];
    if (i2c_read(hdr, 4) != 0)
        return -1;

    int pkt_len = (hdr[0] | (hdr[1] << 8)) & 0x7FFF;
    if (pkt_len <= 4 || pkt_len > buf_len + 4)
        return -1;

    /* Read remaining payload */
    int payload_len = pkt_len - 4;
    if (i2c_read(buf, payload_len) != 0)
        return -1;

    return payload_len;
}

static int set_feature_report(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[17];
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = REPORT_SET_FEATURE;
    cmd[1] = report_id;
    /* Report interval in microseconds (little-endian) */
    cmd[5] = (interval_us >>  0) & 0xFF;
    cmd[6] = (interval_us >>  8) & 0xFF;
    cmd[7] = (interval_us >> 16) & 0xFF;
    cmd[8] = (interval_us >> 24) & 0xFF;

    return shtp_send(SHTP_CHAN_CONTROL, cmd, sizeof(cmd));
}

static float q14_to_float(int16_t val)
{
    return (float)val / (float)(1 << 14);
}

static void parse_rotation_vector(const uint8_t *data, int len)
{
    if (len < 14) return;
    /* Bytes 4-11: i,j,k,real as Q14 fixed-point int16 */
    int16_t qi = (int16_t)(data[5]  | (data[6]  << 8));
    int16_t qj = (int16_t)(data[7]  | (data[8]  << 8));
    int16_t qk = (int16_t)(data[9]  | (data[10] << 8));
    int16_t qr = (int16_t)(data[11] | (data[12] << 8));

    g_latest.x = q14_to_float(qi);
    g_latest.y = q14_to_float(qj);
    g_latest.z = q14_to_float(qk);
    g_latest.w = q14_to_float(qr);
    g_latest.accuracy = data[13] & 0x03;
    g_latest.valid = 1;
}

static void parse_gyro(const uint8_t *data, int len)
{
    if (len < 10) return;
    /* Bytes 4-9: x,y,z as Q9 fixed-point int16 (rad/s) */
    int16_t gx = (int16_t)(data[5] | (data[6] << 8));
    int16_t gy = (int16_t)(data[7] | (data[8] << 8));
    int16_t gz = (int16_t)(data[9] | (data[10] << 8));

    g_latest.gyro_x = (float)gx / (float)(1 << 9);
    g_latest.gyro_y = (float)gy / (float)(1 << 9);
    g_latest.gyro_z = (float)gz / (float)(1 << 9);
}

int vms_bno085_init(int i2c_bus, int i2c_addr, int rate_hz)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus);

    g_fd = open(path, O_RDWR);
    if (g_fd < 0) {
        perror("bno085: open i2c");
        return -1;
    }
    if (ioctl(g_fd, I2C_SLAVE, i2c_addr) < 0) {
        perror("bno085: ioctl I2C_SLAVE");
        close(g_fd);
        g_fd = -1;
        return -1;
    }

    memset(&g_latest, 0, sizeof(g_latest));
    memset(g_seq, 0, sizeof(g_seq));

    /* Wait for BNO085 to boot (takes ~100ms) */
    usleep(150000);

    /* Read and discard any pending advertisements */
    uint8_t buf[MAX_PACKET_SIZE];
    for (int i = 0; i < 10; i++) {
        if (shtp_recv(buf, sizeof(buf)) < 0) break;
        usleep(10000);
    }

    /* Configure report rates */
    uint32_t interval_us = 1000000 / rate_hz;

    if (set_feature_report(REPORT_ROTATION_VEC, interval_us) != 0) {
        fprintf(stderr, "bno085: failed to set rotation vector report\n");
        close(g_fd);
        g_fd = -1;
        return -1;
    }
    usleep(10000);

    if (set_feature_report(REPORT_GYRO_CAL, interval_us) != 0) {
        fprintf(stderr, "bno085: failed to set gyro report\n");
        close(g_fd);
        g_fd = -1;
        return -1;
    }

    return 0;
}

int vms_bno085_read(vms_imu_data_t *data)
{
    uint8_t buf[MAX_PACKET_SIZE];
    int len = shtp_recv(buf, sizeof(buf));
    if (len < 5)
        return -1;

    uint8_t report_id = buf[0];
    switch (report_id) {
        case REPORT_ROTATION_VEC:
            parse_rotation_vector(buf, len);
            break;
        case REPORT_GYRO_CAL:
            parse_gyro(buf, len);
            break;
        default:
            break;
    }

    *data = g_latest;
    return g_latest.valid ? 0 : -1;
}

void vms_bno085_close(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}
