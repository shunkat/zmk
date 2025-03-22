# Keychron B1 Pro JIS Custom Firmware

This repository contains a custom ZMK firmware for the Keychron B1 Pro JIS keyboard with a modified keymap based on Karabiner-Elements configuration.

## Key Remappings

The following key remappings have been implemented:

- ESC → Sleep (changed to Scroll Lock in ZMK)
- Function key → Tab
- Backslash → Y
- Comma → N
- H → Print Screen
- International1 → P
- Japanese Kana → Right Shift
- K → H
- L → J
- Left Control → Caps Lock
- O → U
- Open Bracket → O
- P → I
- Period → M
- Q → Disabled
- Quote → L
- Right Shift → Q
- Semicolon → K
- Slash → B
- U → Disabled
- Y → Mouse Button 2 (right click)

## How to Use

### Building with GitHub Actions

1. Fork this repository
2. Navigate to the Actions tab in your forked repository
3. Run the "Build Keychron B1 Pro JIS Custom Firmware" workflow
4. Download the firmware artifact after the build completes

### Flashing the Firmware

1. Put your keyboard into bootloader mode by holding ESC while plugging in the keyboard (or using the reset button if available)
2. The keyboard should appear as a mass storage device
3. Copy the `keychron_b1_jis_custom.uf2` file to the device
4. The keyboard will automatically restart with the new firmware

## Customizing the Keymap

To further customize the keymap:

1. Edit the `config/keychron_b1_jis.keymap` file
2. Commit your changes
3. Run the GitHub Actions workflow to build the new firmware
4. Flash the updated firmware to your keyboard

## Credits

This firmware is based on [ZMK Firmware](https://zmk.dev/) and the original Keychron B1 Pro configuration.