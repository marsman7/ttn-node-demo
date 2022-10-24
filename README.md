# LoRaWAN - The Things Network

The Things Network - TTN-Node (Sensor) auf Basis des ESP32 TTGO LoRa32 OLED V1. 

https://github.com/LilyGO/TTGO-LORA32/tree/LilyGO-V1.3-868

In der platformio.ini - Datei kann der "Activation Mode" zwischwn ABP und OTAA gewählt werden. Aus Sicherheitsgründen ist der ABP-Modus nicht zu empfehlen ist. (https://www.thethingsindustries.com/docs/devices/abp-vs-otaa/)
In dieser Demo werden keine Sensoren verwendet. Um den Code so übersichtlich wie möglich zu halten werden fixe Werte an das TTN gesendet.

## Bibliothek

https://github.com/mcci-catena/arduino-lmic

## Daten Komprimieren

LoRaWAN ist darauf ausgelegt möglichst energiesparend um bei Batteriebetrieb lange Laufzeiten zu ermöglichen. Das setzt voraus, dass die Datenpakete so kurz wie möglich sind. Außerdem gibt es auch Beschränkungen die Funk-Frequenzbänder nicht zu lange zu belegen, damit diese auch von anderen Geräten genutzt werden können.

Bei einem Temperatur-Sensor ist es z.B. nicht nötig einen "float"-Wert mit 4 Byte Länge zu senden. Die LMIC-Bibliothek bietet dafür z.B. den Pseudo-Datentyp "sflt16" der nur 2 Byte lang ist. Die Genauigkeit ist für die allermeisten Temperaturmessungen ausreichend. Dieser Datentyp kann aber nur Werte zwischen -1,0 und +1,0 annehem. Deswegen wird die Temperatur vor dem kodieren in sflt16 durch 100 geteilt und nach dem dekodieren wieder mit 100 multipliziert. So können Temperaturen von -100°C bis +100°C übertragen werden.

https://github.com/openwave-co-jp/arduino-lmic-master-for-LG01-JP/blob/master/README.md#encoding-utilities

## Daten Auswerten

https://www.thethingsindustries.com/docs/integrations/payload-formatters/javascript/

````js
function decodeUplink(input) {

    function sflt162f(rawSflt16, factor, precision) {
        rawSflt16 &= 0xFFFF;
        if (rawSflt16 == 0x8000)
            return -0.0;
        var sSign = ((rawSflt16 & 0x8000) !== 0) ? -1 : 1;
        var exp1 = (rawSflt16 >> 11) & 0xF;
        var mant1 = (rawSflt16 & 0x7FF) / 2048.0;
        var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);
        return Number((f_unscaled * factor).toFixed(precision));
    }
  
    function uflt162f(rawUflt16, factor, precision) {
        rawUflt16 &= 0xFFFF;
        var exp1 = (rawUflt16 >> 12) & 0xF;
        var mant1 = (rawUflt16 & 0xFFF) / 4096.0;
        var f_unscaled = mant1 * Math.pow(2, exp1 - 15);
        return Number((f_unscaled * factor).toFixed(precision));
    }  

    return {
        data: {
            temperatur: sflt162f(input.bytes[0] | input.bytes[1] << 8, 100, 1),
            humidity:   uflt162f(input.bytes[2] | input.bytes[3] << 8, 100, 0),
            pressure:   uflt162f(input.bytes[4] | input.bytes[5] << 8, 1100, 0),
            voltage:    uflt162f(input.bytes[6] | input.bytes[7] << 8, 6, 2)
        },
        warnings: [],
        errors: []
    };
}
````

## Troubelshooting

##### multiple definition of hal_init';

In der platformio.ini hinzufügen :

> build_flags = -Dhal_init=LMICHAL_init

oder in LMIC project config

> #define hal_init LMICHAL_init 

https://github.com/mcci-catena/arduino-lmic/issues/714