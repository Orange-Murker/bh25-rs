## Persistence Of Vision Firmware For The BornHack 2025 Badge

### Building and flashing

Get your environment set up as described in the [Rust On ESP Book](https://docs.espressif.com/projects/rust/book/).

```
cargo run --release
```

### Using

⚠️ The first effect has all LEDs turned off to somewhat preserve battery on the badges with a broken power switch.

- Use the "SELECT" button on the badge to cycle through effects
- The amount of LEDs lit while the badge is stationary shows the effect number
- Hold the badge stationary for a few seconds so that it can reset its position in space
- Wave the badge from left to right to see the selected effect (works best in the dark)

### Adding your own images

This firmware comes with a few .bmp images in the src folder.

To add your own, see the `display()` function in `main.rs` where you can add your image or effect.
Don't forget to increment the `NUM_EFFECTS` variable when you add effects.

If your image is more than 70 pixels wide you may have to increase the `MAX_WIDTH` variable to use a larger frame buffer.
