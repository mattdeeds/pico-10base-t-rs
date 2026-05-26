MEMORY {
    /* RP2350 with external flash (Pico 2 has 4 MiB; 2 MiB is a safe default) */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    /* 512K of striped SRAM0-7 (good for performance) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
    /* Direct-mapped SRAM8/9 — useful for predictable-access workspaces */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

/*
 * NOTE: rp235x_riscv.x already defines .start_block, .bi_entries, and .end_block
 * sections in FLASH for the RISC-V build, so we don't redeclare them here.
 * For ARM Cortex-M builds we'd need to add them back (see rp235x-hal-examples/memory.x).
 */
