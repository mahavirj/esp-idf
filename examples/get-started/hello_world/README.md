| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# RAM loadable app

For details, please refer to the documentation at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-app-build-type

**This example is for development purpose and should not be used in production use-cases without thorough review and understanding**
This example burns a randomly generated AES-XTS key in the first available empty eFuse key block. It does not burn a new key if Flash Encryption key is already present in eFuse.
Prints **Status: Success** or **Status: Failure** based on the result of the operations.

Please note that this only burns an AES-XTS key in the eFuse and does not enable flash encryption for the device or set relevant security eFuses.

## Steps to build

```
idf.py build
```

## Steps to execute the RAM loadable firmware 

Execute following command with appropriate serial port value

```
esptool.py -p <PORT> --no-stub --baud 576000 load_ram build/hello_world.bin; python $IDF_PATH/tools/idf_monitor.py --no-reset -p <PORT>
```

