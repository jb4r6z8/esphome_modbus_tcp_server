# Modbus TCP Server for ESPHome

An ESPHome component for an ESP32 that serves as modbus tcp server. Primary use is emulating a **Carlo Gavazzi EM24** three-phase energy meter over **Modbus TCP** (WiFi). The Victron GX device polls the ESP32 as if it were a real EM24 meter.

Energy readings can be sourced in any ESPHome manner. The ESP supplies in real time into the EM24 Modbus register layout.

Setting the registers is done by using lamdas calling specific methods.

Regular exposure of setting registers via method is realized. Therefore it can be used as pure modbus tcp server.

Writting towards the modbus server by a modbus client is not supported yet. Functionality is limited towards byte based registers of type HOLDING & INPUT.

## Features

- Emulates Carlo Gavazzi EM24 — identified by the Victron GX as a native energy meter
- Modbus TCP on configurable port (defaults 502) — connects over WiFi, no extra hardware
- Generic method for writting values in registers, which are provided by modbus
- Support of more than one Modbus TCP Server on different ports
- Support of modbus unit ids, e.g. EM24 on Unit 1 and further one on unit 2
- Full three-phase support: voltage, current, active power, frequency, import/export energy
- Configurable meter role (grid, PV inverter, EV charger) via the Victron GX UI
- OTA firmware updates and Home Assistant integration via ESPHome native API
- ESP IDF framework

## Usage
### External component:
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/jb4r6z8/esphome_modbus_tcp_server
      ref: main
    refresh: 0s
    components:
      - modbus_tcp_server
```

### Initialize component (support of multiple servers by adding further entries in the list):
```yaml
modbus_tcp_server:
  - id: virt_em24_1
    port: 502
```
### Generic setting of registers in lambda
**void MBR_Upd(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t idx, ModbusValueType mbvt, float value)** - set register

### Victron specific methods

**void DvcEM24_Propagate(uid,serial_number)** - Propagte EM24 meter with serial number on specific unit id. Call after boot. e.g. propagating 2 EM24 
```id(virt_em24_1)->DvcEM24_Upd_Energy_Import_Total(1,isnan(x) ? 0 : x );``` 
```id(virt_em24_1)->DvcEM24_Upd_Energy_Import_Total(2,isnan(x) ? 0 : x );```

**void DvcEM24_Upd_Energy_Import_Total(uid,value), DvcEM24_Upd_Energy_Export_Total(uid,phase,value)** - Update meter values on specific unit id. Call in interval or at sensor value change events. e.g. updating mport total values on EM24 as unit id 1 and 2 
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,1,isnan(x) ? 0 : x );``` 
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,2,isnan(x) ? 0 : x );```
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,3,isnan(x) ? 0 : x );```


**void DvcEM24_Upd_Voltage(uid,phase,value), DvcEM24_Upd_Current(uid,phase,value), DvcEM24_Upd_Energy_Import_Phases(uid,phase,value)** - Update meter values on specific unit id and specific phase. The import total is not updated with changes on import by phases. The numbers are intentionally no total - grid meter case with phase saldation. Call in interval or at sensor value change events. e.g. updating voltage values on EM24 as unit id 1 for the three phases 
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,1,isnan(x) ? 0 : x );``` 
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,2,isnan(x) ? 0 : x );```
```id(virt_em24_1)->DvcEM24_Upd_Voltage(1,3,isnan(x) ? 0 : x );```

**void DvcEM24_Upd_Power(uid,phase,value, optional calc_current[default true])** - Update meter values on specific unit id and specific phase similar previous methods. The additional parameter calc_current triggers a recalculation of currents. The total power is always updated with each update on a phase. Call in interval or at sensor value change events. e.g. updating power values on EM24 as unit id 1 for the three phases with different variants of additional parameter calc_current.  
```id(virt_em24_1)->DvcEM24_Upd_Power(1,1,isnan(x) ? 0 : x , false);``` 
```id(virt_em24_1)->DvcEM24_Upd_Power(1,2,isnan(x) ? 0 : x , true);```
```id(virt_em24_1)->DvcEM24_Upd_Power(1,3,isnan(x) ? 0 : x );```


## References:
- https://github.com/mahoekst/em24-emulator
Excellent documentation, base of this implementation
- https://github.com/remcom/victron-grid-meter-esphome
Excellent documentation, base of this implementation

## License

MIT — see [LICENSE](LICENSE).
