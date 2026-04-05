# GM MIDI Player — M5Stack Cardputer ADV
### PlatformIO · No PSRAM · SF2 da SD · File browser · 16 canali

---

## Struttura progetto

```
MidiGMPlayer_pio/
├── platformio.ini
├── src/
│   └── main.cpp
├── include/
│   └── FileSelector.h
├── lib/
│   └── TinySoundFont/
│       ├── library.json
│       ├── tsf.h
│       └── tml.h
└── README.md
```

---

## Build

```bash
# Prima build (scarica toolchain ESP32-S3, ~500MB, una tantum)
pio run -e cardputer_adv

# Carica sul dispositivo
# Prima metti in modalità download: Spegni → tieni G0 → accendi → rilascia G0
pio run -e cardputer_adv --target upload

# Monitor seriale
pio device monitor

# Cardputer v1.1 (senza ES8311)
pio run -e cardputer_v11 --target upload
```

---

## Modalità audio

| # | Modalità | Hz | Canali | Hardware |
|---|---|---|---|---|
| 1 | ES8311 integrato ADV | 44100 | Stereo | Jack 3.5mm + speaker (nativo ADV) |
| 2 | I2S DAC esterno | 22050 | Stereo | MAX98357A / PCM5102 |
| 3 | PDM GPIO2 | 16000 | Mono | Speaker interno (nessun extra) |
| 4 | PWM LEDC GPIO2 | 16000 | Mono | Speaker interno (nessun extra) |

### Wiring I2S DAC esterno (solo modalità 2)
```
DAC          Cardputer ADV
─────────────────────────────
BCLK  ──────► GPIO 8
LRCLK ──────► GPIO 7
DIN   ──────► GPIO 6
GND   ──────► GND
VCC   ──────► 3.3V
```

---

## SD Card

Struttura libera — il file browser naviga tutta la SD.

```
/
├── soundfonts/
│   └── gm.sf2       ← max ~200KB senza PSRAM
└── midi/
    └── *.mid
```

**⚠️ Dimensione SF2**: senza PSRAM heap disponibile ~300KB.
Usa Polyphone per creare SF2 < 200KB (8kHz, 8-bit).

---

## Controlli

| Tasto | Azione |
|---|---|
| `SPACE` | Play / Pause |
| `ENTER` o `>` | Traccia successiva |
| `<` | Traccia precedente |
| `R` | Riavvolgi |
| `L` | Toggle loop (`[L]` in header) |
| `+` / `-` | Volume ±3dB |
| `F` | Apri selettore file (SF2 o MIDI) |

### File browser
| Tasto | Azione |
|---|---|
| `W` / `S` | Su / Giù |
| `ENTER` | Apri cartella / seleziona file |
| `BKSP` | Cartella superiore |
| `ESC` | Annulla |
| Lettera | Salta alla prima voce con quella iniziale |

---

## Config automatica (`/midi_player.cfg`)

```ini
sf2=/soundfonts/gm.sf2
midi=/midi/brano.mid
audiomode=1
mididx=0
```

Salvata ad ogni cambio. Al prossimo boot tutti i selettori
partono dall'ultimo path usato.
Per resettare: cancella `/midi_player.cfg` dalla SD.
