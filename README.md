# GM MIDI Player - M5Stack Cardputer ADV
### PlatformIO firmware for MIDI playback with SF2 soundfonts, SD browser, single/dual display UI, runtime menu, and persistent settings

---

## Overview

This project turns the **M5Stack Cardputer ADV** into a **General MIDI player** that:

- loads **SF2 soundfonts** from the SD card
- plays **`.mid` / `.midi`** files from the SD card
- shows a **16-channel live monitor** during playback
- supports both **single-display** and **dual-display** workflows
- supports **multiple audio outputs**
- keeps the latest **audio mode / soundfont / MIDI selection** across reboots
- includes an **embedded boot splash image**

The firmware is designed for the **Cardputer ADV without PSRAM**, so it focuses on low-memory strategies and stable playback on internal RAM only.

---

## Main firmware features

- **Interactive boot flow**
  - embedded splash screen shown at boot
  - display mode selection: `Single display` or `Dual display`
  - audio output selection menu
  - SF2 selection
  - MIDI selection

- **Runtime menu**
  - available from the player screen with `M`
  - lets you change:
    - audio output
    - soundfont
    - MIDI file

- **Persistent configuration on SD**
  - config path: `/Midi/midi_player.cfg`
  - if `/Midi` does not exist, it is created automatically
  - stores:
    - last audio output
    - last soundfont
    - last MIDI file
    - last MIDI index in the current folder

- **File browser**
  - browses the full SD card
  - remembers the last selected file and reopens with the cursor on it
  - page navigation with `,` and `/`

- **Playback engine**
  - TinySoundFont-based SF2 rendering
  - custom MIDI streaming parser
  - optional **raw MIDI buffering in internal RAM** when enough headroom is available
  - total MIDI duration detection and display
  - automatic next-track advance with optional loop mode

- **UI**
  - 16 channel rows with note/program activity
  - scrolling title marquee for long file names
  - partial redraws to reduce flicker
  - progress bar and elapsed/total time
  - in dual display mode:
    - the **external TFT** shows the live 16-channel monitor
    - the **internal Cardputer display** stays on the MIDI list/browser
    - the internal first row shows track index, `Vol` and battery level
    - if playback stays stopped long enough to return to the boot splash, the external TFT is blanked

- **Idle behavior**
  - if playback stays stopped for more than **5 seconds**, the splash image is shown again
  - the splash stays visible until a key is pressed

---

## Audio outputs

### Cardputer ADV

| # | Mode | Sample rate | Channels | Hardware |
|---|---|---|---|---|
| 1 | ADV built-in ES8311 | 22050 Hz | Mono | 3.5 mm jack + internal speaker |
| 2 | External I2S DAC | 22050 Hz | Stereo | MAX98357A / PCM5102 |
| 3 | PDM GPIO2 | 16000 Hz | Mono | Built-in speaker |
| 4 | PWM LEDC GPIO2 | 16000 Hz | Mono | Built-in speaker |

### Cardputer v1.1

The `cardputer_v11` environment removes the ES8311 option and renumbers the remaining outputs from `1` to `3`.

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

Only the **external I2S DAC** path is stereo. Internal Cardputer outputs are mono.

---

## SD card layout

The browser can navigate the whole SD card, but a structure like this is recommended:

```text
/
|-- Midi/
|   |-- midi_player.cfg
|   |-- Songs/
|   |   `-- *.mid
|   `-- SF2/
|       `-- *.sf2
`-- other folders...
```

The firmware does not require fixed folder names for songs and soundfonts, but it does store its config under `/Midi`.

---

## Memory and playback characteristics

This firmware targets a **no-PSRAM** Cardputer ADV, so memory handling matters.

- **SF2** files are loaded from SD into TinySoundFont.
- **MIDI** files are parsed through a custom streaming path.
- When possible, the raw MIDI file is copied into **internal RAM** for faster access.
- A safety headroom is kept before buffering large MIDI files in RAM.
- Internal ES8311 playback uses a **pre-render / queued-buffer path** to reduce stutter.
- `MAX_VOICES` is currently set to **16**.

Practical recommendations:

- prefer **small or very small soundfonts** for the built-in audio path
- for the built-in ES8311 output, a lightweight default such as **`Super Small Font.sf2`** from **Polyphone.io** is strongly recommended
- use the **external I2S DAC** mode if you want the least compromise
- very large or dense MIDI files can still be heavier than simple GM files

---

## Controls

### Player screen

| Key | Action |
|---|---|
| `SPACE` | Play / Pause |
| `/` | Next track |
| `,` | Previous track |
| `R` | Restart current track |
| `L` | Toggle loop |
| `;` | Volume up (+3 dB) |
| `.` | Volume down (-3 dB) |
| `M` | Open runtime menu |

Secondary volume aliases `+`, `=` and `-` are also accepted.

### Dual display mode

When `Dual display` is selected after boot:

- the **external TFT** becomes the playback monitor
- the **internal display** remains focused on the MIDI file list
- the currently highlighted file on the internal list can be started directly

Controls in dual display mode:

| Key | Action |
|---|---|
| `SPACE` | Play / Pause |
| `;` | Select previous MIDI in the internal list |
| `.` | Select next MIDI in the internal list |
| `,` | Previous page in the internal list |
| `/` | Next page in the internal list |
| `ENTER` | Play highlighted MIDI |
| `-` | Volume down (-3 dB) |
| `=` / `+` | Volume up (+3 dB) |
| `L` | Toggle loop |
| `M` | Open runtime menu |

### Runtime menu

Available from the player screen with `M`:

- `Audio Output`
- `GM Soundfont (.sf2)`
- `MIDI File (.mid)`
- `Cancel`

### File browser

| Key | Action |
|---|---|
| `;` | Move up |
| `.` | Move down |
| `,` | Previous page |
| `/` | Next page |
| `ENTER` | Open folder / select file |
| `BACKSPACE` | Go to parent folder |
| `Fn` + `` ` `` | Cancel |
| `A-Z` | Jump to the first matching entry |

---

## Configuration file

Path:

```text
/Midi/midi_player.cfg
```

Format:

```ini
sf2=/Midi/SF2/Small Soundfont.sf2
midi=/Midi/Songs/song.mid
audiomode=1
mididx=0
```

Notes:

- the file is saved automatically after relevant changes
- the firmware can still read the old legacy config path `/midi_player.cfg`
- if you want to reset everything, delete `/Midi/midi_player.cfg`

---

## Build and upload

`platformio.ini` is configured for:

- `cardputer_adv` as the main environment
- `COM4` for upload and monitor
- `dio` flash mode
- `utf-8` serial monitor encoding
- `460800` upload speed

Standard commands:

```bash
pio run -e cardputer_adv
pio run -e cardputer_adv -t upload
pio device monitor
```

Cardputer v1.1:

```bash
pio run -e cardputer_v11 -t upload
```

### Alternate Windows build config

The repository also includes:

```text
platformio_build_alt.ini
```

This is useful on Windows when `.pio/build` gets locked and you want to build into `.pio/build_alt` instead.

Example:

```bash
pio run -c platformio_build_alt.ini -e cardputer_adv -j 1
```

---

## Embedded assets

The boot splash used by the firmware is embedded from:

```text
media/ScreenTitle_boot.jpg
```

This means the splash does **not** depend on the SD card to be visible at boot.

---

## Project layout

```text
ADVMidiPlayer/
|-- platformio.ini
|-- platformio_build_alt.ini
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
|   |-- ScreenTitle.jpg
|   `-- ScreenTitle_boot.jpg
`-- README.md
```

---

## Current firmware profile

- Board target: **Cardputer ADV / ESP32-S3**
- Flash mode: **DIO**
- CPU: **240 MHz**
- Internal output default: **ES8311 22050 Hz mono**
- External DAC mode: **22050 Hz stereo**
- Synth voices: **16**
- Config persistence: **enabled**
- Runtime menu: **enabled**
- Embedded splash: **enabled**
