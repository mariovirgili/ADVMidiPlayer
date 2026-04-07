# GM MIDI Player - M5Stack Cardputer ADV
### PlatformIO - No PSRAM - SF2 from SD - File browser - 16 channels

---

## Project layout

```text
ADVMidiPlayer/
|-- platformio.ini
|-- src/
|   `-- main.cpp
|-- include/
|   |-- CardputerKeyboard.h
|   `-- FileSelector.h
|-- lib/
|   `-- TinySoundFont/
|       |-- library.json
|       |-- tsf.h
|       `-- tml.h
|-- media/
`-- README.md
```

---

## Build and upload

`platformio.ini` is already configured for:

- `cardputer_adv` as the default environment
- `COM4` for upload and monitor
- `dio` flash mode
- `utf-8` serial monitor encoding

```bash
# First build (downloads the ESP32-S3 toolchain the first time)
pio run -e cardputer_adv

# Upload to the Cardputer ADV
pio run -e cardputer_adv -t upload

# Serial monitor
pio device monitor

# Cardputer v1.1 build/upload (without the ES8311 option)
pio run -e cardputer_v11 -t upload
```

If automatic upload does not start, put the board into download mode manually:
power off -> hold `G0` -> power on -> release `G0`.

---

## Audio modes

### Cardputer ADV

| # | Mode | Hz | Channels | Hardware |
|---|---|---|---|---|
| 1 | ADV built-in ES8311 | 22050 | Mono | 3.5mm jack + internal speaker |
| 2 | External I2S DAC | 22050 | Stereo | MAX98357A / PCM5102 |
| 3 | PDM GPIO2 | 16000 | Mono | Cardputer built-in speaker |
| 4 | PWM LEDC GPIO2 | 16000 | Mono | Cardputer built-in speaker |

### Cardputer v1.1

`cardputer_v11` removes the ES8311 mode and renumbers the remaining options from `1` to `3`.

### External I2S DAC wiring

```text
DAC          Cardputer ADV
--------------------------
BCLK   ----> GPIO 8
LRCLK  ----> GPIO 7
DIN    ----> GPIO 6
GND    ----> GND
VCC    ----> 3.3V
```

Only the external I2S DAC path is stereo. Internal Cardputer output paths are mono.

---

## SD card

The SD layout is flexible. The file browser can navigate the whole card.

```text
/
|-- soundfonts/
|   `-- gm.sf2       <- keep it under about 200 KB without PSRAM
`-- midi/
    `-- *.mid
```

**SF2 size note:** without PSRAM, available heap is limited to roughly 300 KB.
Using Polyphone to create an SF2 below about 200 KB is recommended, for example with 8 kHz / 8-bit samples.

---

## Controls

| Key | Action |
|---|---|
| `SPACE` | Play / Pause |
| `/` or `ENTER` | Next track |
| `,` | Previous track |
| `R` | Restart current track |
| `L` | Toggle loop (`[L]` in the header) |
| `;` | Volume up (+3 dB) |
| `.` | Volume down (-3 dB) |
| `F` | Open the runtime file selector |

Secondary volume aliases `+`, `=` and `-` are also accepted.

### File browser

| Key | Action |
|---|---|
| `;` | Move up |
| `.` | Move down |
| `/` or `ENTER` | Open folder / select file |
| `,` or `BACKSPACE` | Go to parent folder |
| `Fn` + `` ` `` | Cancel |
| `A-Z` | Jump to the first matching entry |

---

## Automatic config (`/Midi/midi_player.cfg`)

```ini
sf2=/soundfonts/gm.sf2
midi=/midi/song.mid
audiomode=1
mididx=0
```

The config is saved automatically after every change. On the next boot, selectors start from the last used path.
If `/Midi` does not exist on the SD card, it is created automatically.

To reset it, delete `/Midi/midi_player.cfg` from the SD card.
