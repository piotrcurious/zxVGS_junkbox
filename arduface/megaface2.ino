#include <SPI.h>
#include <SD.h>

// --- Pin Definitions (Adjust these based on your actual wiring) ---
// Data Bus: PORTA (D0-D7) -> Arduino Mega Pins 22-29
#define DATA_PORT_REGISTER    PORTA // Register to write data to
#define DATA_DDR_REGISTER     DDRA  // Data Direction Register for PORTA
#define DATA_PIN_REGISTER     PINA  // Register to read data from

// Control Signals: Map to specific digital pins on Arduino Mega
// We need to know which AVR port these pins belong to for inline assembly reads.
// Use https://www.arduino.cc/en/Hacking/PinMapping2560 for reference.
// Example mapping for common pins:
// Pin 2 (INT0) -> PD0_bit -> DDC0.
// Pin 4 (Digital) -> PG5_bit -> PING & (1<<PG5).
// Pin 5 (Digital) -> PE3_bit -> PINE & (1<<PE3).
// Pin 6 (Digital) -> PH3_bit -> PINH & (1<<PH3).

#define IORQ_PIN_NUM        2   // INT0. Connect Z80 IORQ to this.
// For RD/WR/M1, we'll define their AVR port and bit positions for fast inline assembly reads.
// IMPORTANT: Adjust these to your actual wiring for optimal performance!
// Example assuming:
//   RD_PIN is Digital Pin 4 (PG5)
//   WR_PIN is Digital Pin 5 (PE3)
//   M1_PIN is Digital Pin 6 (PH3)

#define RD_PIN_PORT_REGISTER   PING  // Register to read RD pin state (assuming PG5)
#define RD_PIN_BIT_MASK        (1 << PG5) // Bit mask for RD pin (assuming PG5)

#define WR_PIN_PORT_REGISTER   PINE  // Register to read WR pin state (assuming PE3)
#define WR_PIN_BIT_MASK        (1 << PE3) // Bit mask for WR pin (assuming PE3)

#define M1_PIN_PORT_REGISTER   PINH  // Register to read M1 pin state (assuming PH3)
#define M1_PIN_BIT_MASK        (1 << PH3) // Bit mask for M1 pin (assuming PH3)

#define RESET_Z80_PIN       7   // Z80 RESET detection

// SD Card Pins (Standard SPI)
#define SD_CS_PIN           53 // SS pin on Mega, often used for SD Card CS
#define SD_MOSI_PIN         51
#define SD_MISO_PIN         50
#define SD_SCK_PIN          52

// --- I/O Port Definitions (Arbitrary, to be decoded by Arduino) ---
// Let's stick with a conceptual base port, say 0xFB.
// The Arduino will verify if the Z80 is accessing an I/O port,
// and then use a simple internal "virtual" port mapping based on Z80's A0-A2
// for specific commands/data.

// For simplicity, we'll react to all IORQ cycles for our base port 0xFB.
// This example doesn't implement full 16-bit address decoding.
// If your Z80 I/O access is always to a fixed port (e.g., OUT (0xFB), A),
// this is simpler. If you need multiple distinct I/O ports for the Arduino,
// you'll need to connect more address lines and decode them.

// --- Global Volatile Variables for Communication ---
volatile byte z80_data_byte;      // Data read from or written to Z80
volatile bool io_request_pending = false; // Flag set by ISR, cleared by loop()
volatile bool write_cycle_detected = false;
volatile bool read_cycle_detected = false;
volatile byte z80_read_address_lsb = 0; // LSB of address during IORQ (A0-A7)
volatile byte z80_current_status = 0x00; // 0x00=READY, 0xFF=BUSY, 0xFE=ERROR

// --- SD Card Buffering ---
#define SD_BLOCK_SIZE       512
byte sd_buffer[SD_BLOCK_SIZE];
volatile unsigned int sd_buffer_ptr = 0;

// --- Command/State Variables ---
enum InterfaceState {
    STATE_IDLE,                 // Ready for new command from Z80
    STATE_RECEIVING_ADDRESS,    // Z80 is sending 24-bit block address bytes
    STATE_PERFORMING_SD_OP,     // Arduino is busy with SD card (e.g., read/write block)
    STATE_TRANSFER_READ,        // Arduino has data ready for Z80 to read (from sd_buffer)
    STATE_TRANSFER_WRITE,       // Arduino is expecting data from Z80 to fill sd_buffer
    STATE_ERROR                 // An error occurred
};
volatile InterfaceState current_state = STATE_IDLE;
volatile byte current_command = 0;
volatile uint32_t current_sd_block_address = 0; // 24-bit address
volatile byte addr_byte_count = 0; // Counter for receiving 24-bit address

// --- Function Prototypes ---
void init_bus_pins();
void isr_iorq_handler();
bool read_sd_block(uint32_t block_address);
bool write_sd_block(uint32_t block_address);
void set_status(byte status);

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Arduino ZX Spectrum SD Interface Booting...");

    init_bus_pins();

    // Initialize SD Card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        set_status(0xFE); // Signal ERROR
        current_state = STATE_ERROR;
        return;
    }
    Serial.println("SD Card initialized.");
    set_status(0x00); // Signal READY
    current_state = STATE_IDLE; // Ready to receive commands
}

// --- Main Loop ---
void loop() {
    // Only process SD card operations or complex command sequences here.
    // The ISR simply captures bus events and sets flags.

    if (io_request_pending) {
        // This flag is set by the IORQ ISR.
        // We're processing it here in the main loop to avoid blocking the ISR.
        cli(); // Disable interrupts during critical section to avoid race conditions with ISR flags
        bool _write_cycle = write_cycle_detected;
        bool _read_cycle = read_cycle_detected;
        byte _z80_data_byte = z80_data_byte;
        io_request_pending = false; // Reset flag
        write_cycle_detected = false; // Reset for next cycle
        read_cycle_detected = false; // Reset for next cycle
        sei(); // Re-enable interrupts

        if (_write_cycle) {
            // Z80 wrote to us
            if (current_state == STATE_IDLE) {
                // First byte of a new command
                current_command = _z80_data_byte;
                Serial.print("Cmd Rcv: "); Serial.println(current_command, HEX);
                set_status(0xFF); // Temporarily busy while processing command

                switch (current_command) {
                    case 0x01: // Initialize SD
                        current_state = STATE_PERFORMING_SD_OP;
                        if (!SD.begin(SD_CS_PIN)) {
                            Serial.println("SD Re-Init Failed!");
                            set_status(0xFE);
                            current_state = STATE_ERROR;
                        } else {
                            Serial.println("SD Re-Init OK.");
                            set_status(0x00);
                            current_state = STATE_IDLE;
                        }
                        break;
                    case 0x03: // Read Block (requires address set)
                        Serial.print("Prepare Read Block @ "); Serial.println(current_sd_block_address, HEX);
                        current_state = STATE_PERFORMING_SD_OP; // Go busy while reading from SD
                        if (read_sd_block(current_sd_block_address)) {
                            sd_buffer_ptr = 0; // Reset pointer for Z80 to read from start
                            current_state = STATE_TRANSFER_READ; // Now ready for Z80 to read data
                            set_status(0x00); // Ready to transmit
                            Serial.println("Block read into buffer. Ready for Z80.");
                        } else {
                            Serial.println("Failed to read block.");
                            set_status(0xFE);
                            current_state = STATE_ERROR;
                        }
                        break;
                    case 0x04: // Write Block (requires address set, then data transfer)
                        Serial.print("Prepare Write Block @ "); Serial.println(current_sd_block_address, HEX);
                        Serial.println("Awaiting 512 bytes from Z80...");
                        sd_buffer_ptr = 0; // Reset pointer for Z80 to fill buffer
                        current_state = STATE_TRANSFER_WRITE; // Ready to receive data
                        set_status(0x00); // Ready to receive data
                        break;
                    case 0x10: // Set Block Address LSB
                    case 0x11: // Set Block Address MID
                    case 0x12: // Set Block Address MSB
                        // These are now commands that signify the *next* byte is the address byte.
                        addr_byte_count = 0; // Prepare to receive address bytes
                        current_state = STATE_RECEIVING_ADDRESS;
                        set_status(0x00); // Ready for address byte
                        break;
                    default:
                        Serial.print("Unknown Command: "); Serial.println(current_command, HEX);
                        set_status(0xFE);
                        current_state = STATE_ERROR;
                        break;
                }
            } else if (current_state == STATE_RECEIVING_ADDRESS) {
                // Z80 is sending an address byte
                if (current_command == 0x10) { // LSB
                    current_sd_block_address = (current_sd_block_address & 0xFFFF00) | _z80_data_byte;
                    Serial.print("Addr LSB: "); Serial.println(_z80_data_byte, HEX);
                } else if (current_command == 0x11) { // MID
                    current_sd_block_address = (current_sd_block_address & 0xFF00FF) | ((uint32_t)_z80_data_byte << 8);
                    Serial.print("Addr MID: "); Serial.println(_z80_data_byte, HEX);
                } else if (current_command == 0x12) { // MSB
                    current_sd_block_address = (current_sd_block_address & 0x00FFFF) | ((uint32_t)_z80_data_byte << 16);
                    Serial.print("Addr MSB: "); Serial.println(_z80_data_byte, HEX);
                }
                current_state = STATE_IDLE; // Address byte received, back to idle for next command
                set_status(0x00); // Ready for next command
            } else if (current_state == STATE_TRANSFER_WRITE) {
                // Z80 is sending data for a block write
                if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                    sd_buffer[sd_buffer_ptr++] = _z80_data_byte;
                }
                if (sd_buffer_ptr >= SD_BLOCK_SIZE) {
                    // Block is full, trigger write to SD
                    Serial.println("Buffer full, performing SD write.");
                    current_state = STATE_PERFORMING_SD_OP;
                    if (write_sd_block(current_sd_block_address)) {
                        Serial.println("Block written to SD.");
                        set_status(0x00); // Ready
                        current_state = STATE_IDLE;
                    } else {
                        Serial.println("Failed to write block to SD.");
                        set_status(0xFE);
                        current_state = STATE_ERROR;
                    }
                    sd_buffer_ptr = 0; // Reset for next transfer
                }
                // No status change here, keep receiving. Status will be updated when block is full.
            } else {
                // Unexpected write during current state
                Serial.print("Unexpected Z80 write in state: "); Serial.println(current_state);
                set_status(0xFE);
                current_state = STATE_ERROR;
            }
        }
    }
    // Any other background tasks can go here
}

// --- Pin Initialization ---
void init_bus_pins() {
    // DATA_DDR_REGISTER (PORTA) - Initially inputs, as Z80 will write command
    DATA_DDR_REGISTER = 0x00; // All pins as inputs (D0-D7)
    // No pull-ups on data lines as Z80 bus is driven.

    // Control Signals: Configure as inputs with internal pull-ups if active low
    pinMode(IORQ_PIN_NUM, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP); // RD_PIN_NUM, not attached to interrupt now. (Digital Pin 4)
    pinMode(5, INPUT_PULLUP); // WR_PIN_NUM (Digital Pin 5)
    pinMode(6, INPUT_PULLUP); // M1_PIN_NUM (Digital Pin 6)
    pinMode(RESET_Z80_PIN, INPUT_PULLUP);

    // Configure External Interrupt for IORQ (Digital Pin 2, INT0)
    attachInterrupt(digitalPinToInterrupt(IORQ_PIN_NUM), isr_iorq_handler, FALLING);

    // Set initial status to READY
    set_status(0x00);
}

// --- Set Interface Status ---
void set_status(byte status) {
    current_status = status;
}

// --- Interrupt Service Routine (ISR) for IORQ ---
// This is the most critical part for timing.
ISR(INT0_vect) { // IORQ_PIN_NUM (Digital Pin 2) - IORQ asserted (falling edge)

    // Read control signals using direct port access for speed.
    // Ensure these align with your `_PIN_PORT_REGISTER` and `_PIN_BIT_MASK` defines.
    byte rd_state;
    byte wr_state;
    byte m1_state;
    byte data_from_z80 = 0;

    // Use inline assembly for reading multiple pins from potentially different ports.
    // Example for reading individual bits (adjust registers/masks as per your specific pin choices)
    // This is more complex than simple port reads if pins are scattered.
    // Assuming RD is on PG5, WR on PE3, M1 on PH3
    __asm__ __volatile__ (
        "in %0, %1\n\t"  // Read PING (RD)
        "in %2, %3\n\t"  // Read PINE (WR)
        "in %4, %5\n\t"  // Read PINH (M1)
        : "=r" (rd_state), "=r" (wr_state), "=r" (m1_state)
        : "I" (_SFR_IO_ADDR(RD_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(WR_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(M1_PIN_PORT_REGISTER))
    );

    // Check actual bit states
    bool is_rd_active = !((rd_state & RD_PIN_BIT_MASK)); // Active LOW
    bool is_wr_active = !((wr_state & WR_PIN_BIT_MASK)); // Active LOW
    bool is_m1_active = !((m1_state & M1_PIN_BIT_MASK)); // Active LOW (optional)

    // Check if it's an I/O request (IORQ is already active due to ISR trigger)
    // and if it's a valid bus cycle (e.g., not M1 or MREQ active at same time, depends on Z80 timing)
    // For simple I/O, IORQ + RD/WR is usually sufficient. M1 can confirm CPU instruction fetch.
    if (is_rd_active && !is_wr_active) { // Z80 is requesting to read (RD is LOW, WR is HIGH)
        // Z80 Read Cycle
        read_cycle_detected = true;
        write_cycle_detected = false;

        // Set DATA_PORT as output
        DATA_DDR_REGISTER = 0xFF;

        // Put data on the bus immediately
        // What data to send? This depends on the *state* of the Arduino and command.
        // It's usually the STATUS byte or a byte from the SD buffer.
        // To avoid race conditions, this data must be prepared in advance by the loop().
        // For status read, we put `current_status` on the bus.
        // For data read, we put `sd_buffer[sd_buffer_ptr]` on the bus.

        if (current_state == STATE_IDLE || current_state == STATE_ERROR || current_state == STATE_PERFORMING_SD_OP) {
            // Z80 is reading status
            __asm__ __volatile__ (
                "out %0, %1\n\t"
                :
                : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (current_status)
            );
        } else if (current_state == STATE_TRANSFER_READ) {
            // Z80 is reading a data block
            if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                __asm__ __volatile__ (
                    "out %0, %1\n\t"
                    :
                    : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (sd_buffer[sd_buffer_ptr])
                );
                // Increment pointer after putting data on bus. Loop will update state after 512.
                sd_buffer_ptr++;
            } else {
                // End of block or overrun
                __asm__ __volatile__ (
                    "out %0, %1\n\t"
                    :
                    : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (0x00) // Send 0x00 for end of block?
                );
                // The main loop should transition state after the last byte.
                // Resetting ptr here is risky if Z80 isn't finished.
                // Best to let the Z80 finish reading and then main loop sets state.
            }
        } else {
            // Unexpected read request
            __asm__ __volatile__ (
                "out %0, %1\n\t"
                :
                : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (0xFF) // Send BUSY/UNKNOWN
            );
        }

        // Wait for RD to go HIGH (Z80 has latched data), then tri-state
        // This busy-wait is critical but consumes time.
        // The Z80 holds RD for some cycles.
        // While (digitalRead(RD_PIN) == LOW); // Slower
        // Faster: Poll the register bit for RD_PIN
        byte temp_rd_state;
        do {
            __asm__ __volatile__ (
                "in %0, %1\n\t" : "=r" (temp_rd_state) : "I" (_SFR_IO_ADDR(RD_PIN_PORT_REGISTER))
            );
        } while (!(temp_rd_state & RD_PIN_BIT_MASK)); // Wait for RD_PIN_BIT_MASK to be HIGH

        DATA_DDR_REGISTER = 0x00; // Back to input (tri-state)

    } else if (!is_rd_active && is_wr_active) { // Z80 is writing (WR is LOW, RD is HIGH)
        // Z80 Write Cycle
        write_cycle_detected = true;
        read_cycle_detected = false;

        // Set DATA_PORT as input
        DATA_DDR_REGISTER = 0x00; // This is the default, but explicitly setting it.

        // Wait for WR to go HIGH (Z80 has finished writing), then read data
        byte temp_wr_state;
        do {
            __asm__ __volatile__ (
                "in %0, %1\n\t" : "=r" (temp_wr_state) : "I" (_SFR_IO_ADDR(WR_PIN_PORT_REGISTER))
            );
        } while (!(temp_wr_state & WR_PIN_BIT_MASK)); // Wait for WR_PIN_BIT_MASK to be HIGH

        // Read data from the bus using inline assembly for speed
        __asm__ __volatile__ (
            "in %0, %1\n\t"
            : "=r" (data_from_z80)
            : "I" (_SFR_IO_ADDR(DATA_PIN_REGISTER))
        );
        z80_data_byte = data_from_z80;

    } else {
        // This could be an M1 cycle with IORQ (Z80 Interrupt acknowledge), or invalid state.
        // For simplicity, we ignore these unless specifically needed.
        // Serial.println("Unhandled IORQ type"); // Don't use Serial in ISR!
    }

    // Set flag for main loop to process
    io_request_pending = true;
}

// --- SD Card Operations (Blocking calls, not in ISR) ---
// These are standard SD.h operations. For true raw block access,
// consider the SdFat library for its `FsVolume::readBlocks()`/`writeBlocks()`.
// For this example, we'll continue with the "ZXDISK.BIN" file approach,
// which is simple but less performant than direct block device access.
bool read_sd_block(uint32_t block_address) {
    File dataFile;
    dataFile = SD.open("ZXDISK.BIN", FILE_READ);
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesRead = dataFile.read(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            if (bytesRead == SD_BLOCK_SIZE) {
                return true;
            }
        }
        dataFile.close();
    }
    return false;
}

bool write_sd_block(uint32_t block_address) {
    // Note: SD.open(FILE_WRITE) truncates. To write to specific blocks,
    // you need to either open in FILE_WRITE and then seek+write for an existing file
    // (which means you need to create a large file first), or use SdFat library.
    // This example will work if ZXDISK.BIN is already created AND large enough.

    File dataFile;
    // For updating a file, FILE_WRITE might not be the best.
    // Try to open it for update or use SdFat.
    // Here's a workaround for SD.h if you want to overwrite within an existing file:
    dataFile = SD.open("ZXDISK.BIN", FILE_READ); // Open for reading, then close.
                                                  // This is a hacky way to prevent truncation
                                                  // if it's already there, then reopen for writing.
                                                  // Not reliable. Use SdFat.
    if(dataFile) dataFile.close(); // Close if exists

    dataFile = SD.open("ZXDISK.BIN", FILE_WRITE); // This will truncate if not careful.
                                                  // Better: use FsFile::open("name", O_RDWR) from SdFat
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesWritten = dataFile.write(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            if (bytesWritten == SD_BLOCK_SIZE) {
                return true;
            }
        }
        dataFile.close();
    }
    return false;
}
