# Primary references, not unverified blog summaries

## Sensor hardware

1. MPU-6000/6050 Register Map Rev 4.2, timing, DLPF, FIFO, offsets and aux bypass: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf>.
2. BMP280 Datasheet Rev 1.26, timing, normal mode and factory compensation: <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf>.
3. QST QMC5883P Rev E, chip ID and control/status registers: <https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf>.
4. Betaflight `compass_qmc5883` driver for independent register comparison: <https://github.com/betaflight/betaflight/blob/master/src/main/drivers/compass/compass_qmc5883.c>.
5. Nooploop TOFSense F2 Mini Datasheet and Frame0 parser: <https://ftp.nooploop.com/downloads/tofsense/TOFSense_F2_Mini_Datasheet_V1.0_en.pdf>, <https://github.com/nooploop-dev/nlink_unpack/blob/master/nlink_tofsense_frame0.c>.

## State estimation and flight control

6. Mahony, Hamel & Pflimlin, *Non-linear Complementary Filters on SO(3)* (2008): <https://openresearch-repository.anu.edu.au/items/db91f02d-9a17-4ead-bc64-86ed5bc9c6f4>.
7. Sabatini et al. (2014), inertial/barometric altitude/vertical-velocity fusion: <https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/>.
8. MIT 2.003SC dynamics, MIT 2.14/16.30 feedback, MIT 6.041SC probability, MIT 16.333 aircraft control. Useful educational references, *not* specific board validation.
9. Bluepad32 FAQ on ESP32-S3 BLE-only host and Xbox firmware compatibility: <https://bluepad32.readthedocs.io/en/latest/FAQ/>.

## Reading guideline

Prefer silicon manufacturer documents for register timing/sensitivity, raw captures for what the actual PCB did, peer-reviewed filtering/control papers for methods, and host scripts for reproducibility. Distinguish empirical board behavior from a theoretically permitted sensor mode.
