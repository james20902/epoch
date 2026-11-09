#include "pico/unique_id.h"
#include "tusb.h"

// TinyUSB's test VID, TODO: replace both before this leaves the bench
#define USB_VID 0xCafe
#define USB_PID 0x4001
#define USB_BCD 0x0200

#define CDC_EP_SIZE 64

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

/*
 * Device
 */

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,

    // CDC uses an interface association descriptor, which the spec requires be
    // advertised as misc class, common subclass, IAD protocol
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,

    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&desc_device; }

/*
 * Configuration
 */

static const uint8_t desc_configuration[] = {
    // config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // interface number, string index, notification ep + size, data ep out + in + size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CDC_EP_SIZE),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; // only the one configuration
    return desc_configuration;
}

/*
 * Strings
 */

static const char *const string_desc[] = {
    [STRID_MANUFACTURER] = "epoch",
    [STRID_PRODUCT] = "epoch flight computer",
    [STRID_SERIAL] = NULL, // filled from the chip's unique id
    [STRID_CDC] = "epoch telemetry",
};

#define DESC_STR_MAX 32

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc_str[DESC_STR_MAX + 1];
    static char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    size_t len;

    if (index == STRID_LANGID) {
        desc_str[1] = 0x0409; // english
        len = 1;
    } else {
        if (index >= sizeof(string_desc) / sizeof(string_desc[0])) {
            return NULL;
        }

        const char *str = string_desc[index];
        if (index == STRID_SERIAL) {
            if (!serial[0]) {
                pico_get_unique_board_id_string(serial, sizeof(serial));
            }
            str = serial;
        }

        // ascii -> utf-16
        for (len = 0; len < DESC_STR_MAX && str[len]; len++) {
            desc_str[1 + len] = str[len];
        }
    }

    // first entry is the byte length (header included) and the descriptor type
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return desc_str;
}
