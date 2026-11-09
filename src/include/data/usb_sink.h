#ifndef USB_SINK_H
#define USB_SINK_H

#include "data/data.h"

/*
 * USB CDC (virtual serial port) data sink. Everything in here, including the
 * USB IRQ, runs on whichever core calls sink_init, so init and task must be
 * called from the same core.
 */

// rhport the device stack runs on, rp2 only has the one
#define USB_SINK_RHPORT 0

extern struct data_sink usb_sink;

#endif
