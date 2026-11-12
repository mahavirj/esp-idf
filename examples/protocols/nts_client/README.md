| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- |

# NTS (Network Time Security) Client Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example sets the system time from an authenticated source using Network Time Security ([RFC 8915](https://www.rfc-editor.org/rfc/rfc8915)). Plain SNTP responses are not authenticated, so anyone on the network path can forge them. With NTS every response carries an AEAD tag that only the server can produce.

The default server is `nts.meta.com`. Any RFC 8915 server whose certificate chains to a root in the ESP x509 certificate bundle can be used.

## How it works

1. **Key establishment (NTS-KE).** The client opens a TLS 1.3 connection to the NTS-KE server on TCP port 4460 with ALPN `ntske/1`, and negotiates NTPv4 and an AEAD algorithm. The server returns eight opaque cookies and, optionally, the NTP server to use. Both sides derive the client-to-server and server-to-client keys from the TLS exporter, so no key ever crosses the wire.
2. **Authenticated NTP.** Each NTPv4 request over UDP carries a unique identifier, one cookie and an authenticator. The server recovers the keys from the cookie, authenticates its response and encrypts a replacement cookie into it. A cookie is never used twice.
3. **Recovery.** A lost response costs a cookie, so the next request asks for extra ones. After three consecutive failures, or when the cookies run out, the client runs the key establishment again, waiting 10 seconds and then twice as long each time (up to one hour) until a query succeeds. Some servers, including the default one, do not send an NTS NAK, so silence is treated as "start over". An authenticated kiss-o'-death is handled without a new key establishment: `RATE` doubles the polling interval, `DENY` and `RSTR` stop the client.

| File | Content |
| ---- | ------- |
| `main/nts_client.c` | NTS-KE and NTS-protected NTP exchange |
| `main/aes_gcm_siv.c` | AES-128-GCM-SIV (RFC 8452), AEAD ID 30, preferred |
| `main/aes_siv.c` | AES-SIV-CMAC-256 (RFC 5297), AEAD ID 15, mandatory in RFC 8915 |
| `main/nts_example_main.c` | Runs the AEAD self tests, connects to the network and polls for time |

Both AEAD algorithms are built on the PSA Crypto API, as Mbed TLS does not provide them.

## Configuration

Open the project configuration menu (`idf.py menuconfig`):

* Configure Wi-Fi or Ethernet under "Example Connection Configuration".
* Under "Example Configuration", set the NTS-KE server host name and port, and the polling interval.

The example enables `CONFIG_MBEDTLS_SSL_PROTO_TLS1_3` and `CONFIG_MBEDTLS_SSL_KEYING_MATERIAL_EXPORT` in `sdkconfig.defaults`. NTS does not work without them.

## Build and Flash

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

```
I (107) nts_example: AES-SIV and AES-GCM-SIV self tests passed
I (4107) nts_example: NTS-KE with nts.meta.com:4460
I (5007) nts: NTS-KE done: AEAD 30, 8 cookies of 68 bytes, NTP server time.meta.com:123
I (5317) nts_example: offset +1794466762123761 us, delay 303631 us, stratum 1, cookies 8 (+1)
I (5317) nts_example: Authenticated time: 2026-11-12 06:59:27 UTC
I (69507) nts_example: offset +11171 us, delay 186218 us, stratum 1, cookies 8 (+1)
I (69507) nts_example: Authenticated time: 2026-11-12 07:00:31 UTC
```

## Notes

* **First boot.** The clock is not set when the TLS handshake runs, so the certificate validity period cannot be checked on the first key establishment. This is the default behaviour of ESP-IDF (`CONFIG_MBEDTLS_HAVE_TIME_DATE` is disabled). A product should at least reject times earlier than its build date.
* **Clock discipline.** The example steps the clock by the measured offset on every poll. It does not filter samples or slew the clock.
* **IPv4 first.** The NTP server is resolved over IPv4 first and over IPv6 only if that fails.
