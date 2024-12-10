const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const gpio = rp2xxx.gpio;
const Pio = rp2xxx.pio.Pio;
const Irq = rp2xxx.pio.Irq;
const StateMachine = rp2xxx.pio.StateMachine;
const dma = rp2xxx.dma;

const ws2812_program = blk: {
    @setEvalBranchQuota(10000);
    const p = rp2xxx.pio.assemble(
        \\.program ws2812
        \\.side_set 1 opt
        \\
        \\    pull block                  ; pull the next data (number of LEDs - 1)
        \\    out y, 32                   ; and put it to "Y"
        \\
        \\.wrap_target
        \\    mov x, y                    ; initialise the LED counter "X" with the value of "Y"
        \\    set pins, 0                 ; Turn off status LED
        \\next_led:
        \\    pull block                  ; pull the next data (RGB value for a LED)
        \\    irq 0                       ; set INT 0 flag
        \\    out null, 8                 ; ignore the trailing 8 bits
        \\
        \\next_bit:
        \\    set pins, 1 side 1          ; Set status LED and output led to 1
        \\
        \\    out pins, 1                 ; the middle 1/3 is equal to actual value we send
        \\    jmp !osre, next_bit side 0  ; set the GPIO to "0", jump if the shift register contains more bits
        \\                                ; shift register empty -> no more data to send for a LED
        \\
        \\    jmp x--, next_led           ; jump if more LEDs to send data to
        \\                                ; here we are done - introduce the 50us reset delay
        \\                                ; wait 120 cycles (50us = 40 bit lengths * 3)
        \\    set x, 14                   ; 120 = 15 * 8 (set "X" to 15-1)
        \\delay_loop:
        \\    set pins, 0
        \\    jmp x-- delay_loop [6]      ; 1 + 7 cycles in each iteration
        \\
        \\    irq clear 0                 ; clear INT 0 flag
        \\.wrap
    , .{}).get_program_by_name("ws2812");
    break :blk p;
};

const pio: Pio = rp2xxx.pio.num(0);
const sm: StateMachine = .sm0;
const ws2812_pin = gpio.num(16);
const led_pin = gpio.num(14);

pub fn main() void {
    pio.gpio_init(ws2812_pin);
    pio.gpio_init(led_pin);
    // led_pin is used as set pin, just for debugging // DELETEME
    pio.sm_set_pindir(sm, @intFromEnum(led_pin), 1, .out); // DELETEME
    // ws2812 is used as out pin AND sideset pin
    pio.sm_set_pindir(sm, @intFromEnum(ws2812_pin), 1, .out);

    // Enable the interrupt that will be set in the PIO program
    const irq = Irq.irq0;
    pio.sm_enable_interrupt(sm, irq, Irq.Source.statemachine);

    const div = @as(f32, @floatFromInt(rp2xxx.clock_config.sys.?.frequency())) / 1e6 * 1.25 / 3;

    // This loads the PIO program, setting the side set, wrap, etc. based on the Program returned from the assembler
    pio.sm_load_and_start_program(sm, ws2812_program, .{
        .clkdiv = rp2xxx.pio.ClkDivOptions.from_float(div),
        .pin_mappings = .{
            // .in doesn't take a count, so it is JUST a start index
            // .in = .{}
            .out = .{
                .base = @intFromEnum(ws2812_pin),
                .count = 1,
            },
            // This needs to be set if we use set or mov
            .set = .{
                .base = @intFromEnum(led_pin),
                .count = 2,
            },
            .side_set = .{
                .base = @intFromEnum(ws2812_pin),
                // Need an extra if we make sideset optional
                .count = 2,
                // .count = 1,
            },
        },
        .shift = .{
            .out_shiftdir = .left,
            // .autopull = true,
            // 0 means 32
            // .pull_threshold = 0,
            .join_tx = true,
        },
    }) catch unreachable;

    pio.sm_set_enabled(sm, true);

    const LED_COUNT = 1;
    const colours = [_]u32{ 0x008CAC, 0x0D3DA5, 0x340EAB, 0x7C0CDE, 0x930D6B, 0x0C0303, 0xB68B01, 0x9EDE00, 0x49DF03, 0x06DF00, 0x00DF09, 0x000000 };
    var led_buffer = [_]u32{0} ** LED_COUNT;
    var first_colour: u32 = 0;

    // Config DMA to copy LED buffer to the PIO statemachine's FIFO
    const DREQ_PIO0_TX0 = 0;
    // This returns null for me!
    // var dma_ch = dma.claim_unused_channel();
    var dma_ch = dma.channel(0);
    const dma_config = dma.Channel.TransferConfig{
        // Transfer 32 bits each time. The program ignores the first 8, and 24
        // bits isn't supported by the DMA
        .transfer_size_bytes = 4,
        .enable = true,
        // Increment the address we read from after each write
        .read_increment = true,
        // But DON'T increment the write address: We are writing to a register
        .write_increment = false,
        // Use the PIO0 TX0 FIFO to request data
        .dreq = @enumFromInt(DREQ_PIO0_TX0),
    };
    const addr = pio.sm_get_tx_fifo(sm);

    const uart_tx_pin = gpio.num(0);
    const uart_rx_pin = gpio.num(1);
    inline for (&.{ uart_tx_pin, uart_rx_pin }) |pin| {
        pin.set_function(.uart);
    }
    const uart = rp2xxx.uart.instance.num(0);
    uart.apply(.{
        .baud_rate = 115200,
        .clock_config = rp2xxx.clock_config,
    });
    rp2xxx.uart.init_logger(uart);

    pio.sm_blocking_write(sm, LED_COUNT - 1);

    while (true) {
        // Rotate colours
        for (0..LED_COUNT) |i| {
            led_buffer[i] = colours[(i + first_colour) % colours.len];
        }
        first_colour = (first_colour + 1) % colours.len;

        // Wait for interrupt from PIO indicating it's done
        while (pio.get_irq_regs(irq).*.status.raw & (1 << 8) != 0) {}
        dma_ch.trigger_transfer(@intFromPtr(addr), @intFromPtr(&led_buffer), LED_COUNT, dma_config);

        rp2xxx.time.sleep_ms(250);
    }
}
