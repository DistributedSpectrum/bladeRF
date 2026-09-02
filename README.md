# bladeRF Source #
This repository contains all the source code required to program and interact with a bladeRF platform, including firmware for the Cypress FX3 USB controller, HDL for the Altera Cyclone IV FPGA, and C code for the host side libraries, drivers, and utilities.
The source is organized as follows:


| Directory         | Description                                                                                       |
| ----------------- |:--------------------------------------------------------------------------------------------------|
| [firmware_common] | Source and header files common between firmware and host software                                 |
| [fx3_firmware]    | Firmware for the Cypress FX3 USB controller                                                       |
| [hdl]             | All HDL code associated with the Cyclone IV FPGA                                                  |
| [host]            | Host-side libraries, drivers, utilities and samples                                               |


## Quick Start ##
1. Clone this repository via: ```git clone https://github.com/Nuand/bladeRF.git```
2. Fetch the latest pre-built bladeRF [FPGA image]. See the README.md in the [hdl] directory for more information.
3. Fetch the latest pre-built bladeRF [firmware image]. See the README.md in the [fx3_firmware] directory for more information.
4. Follow the instructions in the [host] directory to build and install libbladeRF and the bladeRF-cli utility.
5. Attach the bladeRF board to your fastest USB port.
6. You should now be able to see your device in the list output via ```bladeRF-cli -p```
7. You can view additional information about the device via ```bladeRF-cli -e info -e version```.
8. If any warnings indicate that a firmware update is needed, run:```bladeRF-cli -f <firmware_file>```. 
 - If you ever find the device booting into the FX3 bootloader (e.g., if you unplug the device in the middle of a firmware upgrade), see the ```recovery``` command in bladeRF-cli for additional details.
9. See the overview of the [bladeRF-cli] for more information about loading the FPGA and using the command line interface tool

For more information, see the [bladeRF wiki].

## Build Variables ##

Below are global options to choose which parts of the bladeRF project should
be built from the top level.  Please see the [fx3_firmware] and [host]
subdirectories for more specific options.

| Option                            | Description
| --------------------------------- |:--------------------------------------------------------------------------|
| -DENABLE_FX3_BUILD=\<ON/OFF\>     | Enables building the FX3 firmware. Default: OFF                           |                                   |
| -DENABLE_HOST_BUILD=\<ON/OFF\>    | Enables building the host library and utilities overall. Default: ON      |

## DS: Publishing FPGA/FX3 images for a libbladeRF release ##

Whenever a new libbladeRF version tag is cut, publish the matching FPGA and
FX3 images to the `ds-bladerf-images` S3 bucket so anyone installing that
libbladeRF version can fetch known-good images.

1. Determine the version to publish under. Use the libbladeRF version being
   released (`VERSION_INFO_MAJOR`/`MINOR`/`PATCH` in
   `host/libraries/libbladeRF/CMakeLists.txt`), not the FPGA or FX3 version.
2. Build (or locate an already-built, timing-clean) FPGA image for each
   board variant you support (currently xA4 and xA5) via
   `hdl/quartus/build_bladerf.sh`, or reuse an existing `*.rbf` under
   `hdl/quartus/`.
3. Build the FX3 firmware from `fx3_firmware/` (see that directory's
   README for the Cypress FX3 SDK setup):
   ```
   cd fx3_firmware && mkdir -p build && cd build
   cmake -DFX3_INSTALL_PATH=/opt/cypress/fx3_firmware_linux \
         -DCMAKE_TOOLCHAIN_FILE=../cmake/fx3-toolchain.cmake ..
   make
   ```
   This produces `build/output/bladeRF_fw_v<version>.img`.
4. Confirm the FX3 firmware version you're publishing actually satisfies
   `host/libraries/libbladeRF/src/board/bladerf2/compatibility.c`'s
   `fpga_compat` entry for the FPGA version being published (and
   `FPGA_LZMA_FW_MAJOR/MINOR/PATCH` in
   `host/libraries/libbladeRF/src/board/bladerf1/flash.c` if the image
   needs LZMA-compressed autoload to fit a board's SPI flash budget).
   Bump those if the new FPGA/FX3 pairing needs a newer minimum.
5. Upload under a `libbladeRF-v<version>/` prefix, one subfolder per image
   type, plus a `MANIFEST.txt` with sha256 checksums and the source commit.
   The FPGA version goes on the *folder*, not the filename: keep each
   `.rbf` named exactly `hostedxA4.rbf` / `hostedxA5.rbf` (no version
   suffix), matching the fixed names libbladeRF's `file_find()` search
   path looks for in `bladerf2.c` -- that way a file fetched from here can
   be dropped as-is into e.g. `~/.config/Nuand/bladeRF/` for autoload on
   open, with no renaming:
   ```
   aws s3 cp <xA4 rbf>  s3://ds-bladerf-images/libbladeRF-v<version>/fpga/v<fpga_version>/hostedxA4.rbf --profile s3
   aws s3 cp <xA5 rbf>  s3://ds-bladerf-images/libbladeRF-v<version>/fpga/v<fpga_version>/hostedxA5.rbf --profile s3
   aws s3 cp <fw image> s3://ds-bladerf-images/libbladeRF-v<version>/fx3/bladeRF_fw_v<fw_version>.img --profile s3
   aws s3 cp MANIFEST.txt s3://ds-bladerf-images/libbladeRF-v<version>/MANIFEST.txt --profile s3
   ```
   Authenticate first with `aws sso login --profile s3` if the session has
   expired.
6. Before publishing, flash both a real xA4 and xA5 with the new images and
   confirm `bladeRF-cli -e info -e version` reports the expected FPGA/FX3
   versions and `FPGA loaded: yes`, and that RX still produces sane IQ data.

[firmware_common]: ./firmware_common (Host-Firmware common files)
[fx3_firmware]: ./fx3_firmware (FX3 Firmware)
[hdl]: ./hdl (HDL)
[host]: ./host (Host)
[FPGA image]: https://www.nuand.com/fpga.php (Pre-built FPGA images)
[firmware image]: https://www.nuand.com/fx3.php (Pre-built firmware binaries)
[bladeRF-cli]: ./host/utilities/bladeRF-cli (bladeRF Command Line Interface)
[bladeRF wiki]: https://github.com/nuand/bladeRF/wiki (bladeRF wiki)
