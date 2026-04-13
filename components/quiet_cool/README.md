```yaml
# example configuration:

fan:
  - platform: quiet_cool
    name: QuietCool fan
    output: gpio_d1

output:
  - platform: gpio
    pin: D1
    id: gpio_d1
```

Add an optional text sensor to log every received packet (hex):

```yaml
fan:
  - platform: quiet_cool
    name: QuietCool fan
    output: gpio_d1
    # other config keys omitted for brevity
    rx_packet_sensor:
      name: "QuietCool RX Packet"

# The `rx_packet_sensor` will receive a hex dump of each RX packet
# as a text sensor state which Home Assistant will store in history.
```
