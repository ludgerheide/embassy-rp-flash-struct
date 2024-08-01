<!-- Title -->
<p align="center">
  <img width=15% src="https://www.svgrepo.com/show/68860/microchip.svg">
  <h1 align="center">struct-on-flash</h1>
</p>

This is a simple wear leveling library for reading and writing structs to and from flash on the RP2040. See `src/main.rs` for useage information.

In the current configuration, the top megabyte of the 2MB flash is used for this libary and needs to be removed from the program space in your `memory.x` linker script.

## License

This code is licensed under the AGPLv3. See [LICENSE.md](LICENSE.md) for details.