#include <stdio.h>
#include "cmd.h"

/* The FX3 acks the setup packet for BLADE_USB_CMD_RESET and only then resets
 * the chip (fx3_firmware/src/bladeRF.c), so the status stage of the control
 * transfer races the reset. libusb can report the device as gone on a reset
 * that actually worked.
 *
 * In practice the ack wins and bladerf_device_reset() returns 0. That was
 * measured on firmware 2.6.0 and 2.6.1 on an xA5. The race is still real, so
 * these codes count as success instead of being reported as a failed reset.
 *
 * The libusb-to-libbladeRF mapping is error_conv() in the USB backend:
 * NO_DEVICE and BUSY become NODEV, IO stays IO, and PIPE falls through the
 * default case to UNEXPECTED. A status stage that never completes times out.
 */
static bool reset_raced_the_device(int status)
{
    switch (status) {
        case BLADERF_ERR_NODEV:
        case BLADERF_ERR_IO:
        case BLADERF_ERR_TIMEOUT:
        case BLADERF_ERR_UNEXPECTED:
            return true;

        default:
            return false;
    }
}

int cmd_reset(struct cli_state *state, int argc, char **argv)
{
    int status;

    if (argc != 1) {
        return CLI_RET_NARGS;
    }

    status = bladerf_device_reset(state->dev);

    if (status != 0 && !reset_raced_the_device(status)) {
        state->last_lib_error = status;
        return CLI_RET_LIBBLADERF;
    }

    /* The handle now refers to a device that is rebooting and will come back on
     * a new USB address, so there is nothing left to close cleanly. Dropping
     * the pointer leaks the handle, and that is the deliberate trade:
     * bladerf_close() here can sit in libusb waiting on a device that is
     * already gone, and this command is meant to run from a boot-time
     * provisioning unit where a hang is far worse than a leaked handle in a
     * process that is about to exit. Clearing it also keeps
     * cli_state_destroy() from closing it later.
     */
    state->dev = NULL;

    printf("\n  Device reset. The FX3 reboots from its own SPI flash and "
           "re-enumerates on a\n  new USB address, which takes about a second. "
           "The reset also deconfigures the\n  FPGA, so the board comes back "
           "in the state a power cycle would leave it.\n\n");

    return CLI_RET_OK;
}
