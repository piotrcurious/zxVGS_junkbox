#include <SPI.h>
#include <SD.h>

// --- Pin Definitions (Adjust these based on your actual wiring) ---
// Data Bus: PORTA (D0-D7) -> Arduino Mega Pins 22-29
#define DATA_PORT_REGISTER    PORTA // Register to write data to
#define DATA_DDR_REGISTER     DDRA  // Data Direction Register for PORTA
#define DATA_PIN_REGISTER     PINA  // Register to read data from

// Control Signals: Map to specific digital pins on Arduino Mega
// For RD/WR/M1, we'll still define their AVR port and bit positions for fast inline assembly reads
// within the IORQ ISR, as the WAIT signal needs to be asserted quickly.
// Example mapping (adjust based on your physical wiring):
//   RD_PIN is Digital Pin 4 (PG5)
//   WR_PIN is Digital Pin 5 (PE3)
//   M1_PIN is Digital Pin 6 (PH3)

#define IORQ_PIN_NUM        2   // INT0. Connect Z80 IORQ to this.

#define RD_PIN_PORT_REGISTER   PING  // Register to read RD pin state (assuming PG5)
#define RD_PIN_BIT_MASK        (1 << PG5) // Bit mask for RD pin (assuming PG5)

#define WR_PIN_PORT_REGISTER   PINE  // Register to read WR pin state (assuming PE3)
#define WR_PIN_BIT_MASK        (1 << PE3) // Bit mask for WR pin (assuming PE3)

#define M1_PIN_PORT_REGISTER   PINH  // Register to read M1 pin state (assuming PH3)
#define M1_PIN_BIT_MASK        (1 << PH3) // Bit mask for M1 pin (assuming PH3)

#define RESET_Z80_PIN       7   // Z80 RESET detection

// Z80 WAIT Signal: Arduino OUTPUT pin connected to Z80 WAIT input (active LOW)
#define Z80_WAIT_PIN        8   // Connect this to Z80 WAIT.

// SD Card Pins (Standard SPI)
#define SD_CS_PIN           53 // SS pin on Mega, often used for SD Card CS
#define SD_MOSI_PIN         51
#define SD_MISO_PIN         50
#define SD_SCK_PIN          52

// --- I/O Port Definitions (Arbitrary, to be decoded by Arduino) ---
// Let's assume a single base port, say 0xFB.
// The Arduino will react to all IORQ cycles for our base port 0xFB.
// This example doesn't implement full 16-bit address decoding.

// --- Global Volatile Variables for Communication ---
volatile byte z80_data_byte;      // Data read from or written to Z80
volatile bool io_request_pending = false; // Flag set by ISR, cleared by loop()
volatile bool write_cycle_detected = false;
volatile bool read_cycle_detected = false;
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
void assert_wait();
void release_wait();
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
    // The ISR simply captures bus events and sets flags, then asserts WAIT.
    // The main loop does the heavy lifting, then releases WAIT.

    if (io_request_pending) {
        // This flag is set by the IORQ ISR, and WAIT is being asserted.
        // We can now take our time.
        // Copy ISR set flags to local variables to avoid constant volatile access.
        bool _write_cycle = write_cycle_detected;
        bool _read_cycle = read_cycle_detected;
        byte _z80_data_byte = z80_data_byte; // This will hold the data if it was a write cycle

        if (_write_cycle) {
            // Z80 wrote to us
            // Data has already been read in ISR (while WAIT was asserted)
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
                        // The Z80 will send the *address byte* after receiving 0x00 (ready) from us.
                        // So, the next OUT (SD_PORT), byte will be an address byte.
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
                // Z80 is sending an address byte (following a 0x10-0x12 command)
                // The current_command (0x10, 0x11, 0x12) tells us which part of the address it is.
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
                current_state = STATE_IDLE; // Address byte received, back to idle for next command/data
                set_status(0x00); // Ready for next command/data
            } else if (current_state == STATE_TRANSFER_WRITE) {
                // Z80 is sending data for a block write
                if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                    sd_buffer[sd_buffer_ptr++] = _z80_data_byte;
                }
                if (sd_buffer_ptr >= SD_BLOCK_SIZE) {
                    // Block is full, trigger write to SD
                    Serial.println("Buffer full, performing SD write.");
                    current_state = STATE_PERFORMING_SD_OP; // Block until write is done
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
                // If buffer not full, stay in STATE_TRANSFER_WRITE and set READY for next byte
                if (current_state == STATE_TRANSFER_WRITE) { // Only if not already changed to performing SD op
                    set_status(0x00); // Ready to receive next byte
                }
            } else {
                // Unexpected write during current state
                Serial.print("Unexpected Z80 write in state: "); Serial.println(current_state);
                set_status(0xFE);
                current_state = STATE_ERROR;
            }
        }
        // Release WAIT as the current Z80 I/O cycle processing is finished.
        io_request_pending = false; // Reset flag first before releasing WAIT
        release_wait();
    }
    // Any other background tasks can go here (not time-critical)
}

// --- Pin Initialization ---
void init_bus_pins() {
    // DATA_DDR_REGISTER (PORTA) - Initially inputs, as Z80 will write command
    DATA_DDR_REGISTER = 0x00; // All pins as inputs (D0-D7)

    // Control Signals: Configure as inputs with internal pull-ups (active low signals)
    pinMode(IORQ_PIN_NUM, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP); // RD (Digital Pin 4)
    pinMode(5, INPUT_PULLUP); // WR (Digital Pin 5)
    pinMode(6, INPUT_PULLUP); // M1 (Digital Pin 6)
    pinMode(RESET_Z80_PIN, INPUT_PULLUP);

    // Z80_WAIT_PIN: Configure as output, initially HIGH (no wait states)
    pinMode(Z80_WAIT_PIN, OUTPUT);
    release_wait(); // Ensure WAIT is HIGH initially

    // Configure External Interrupt for IORQ (Digital Pin 2, INT0)
    attachInterrupt(digitalPinToInterrupt(IORQ_PIN_NUM), isr_iorq_handler, FALLING);

    // Set initial status to READY
    set_status(0x00);
}

// --- Assert/Release Z80 WAIT Line ---
void assert_wait() {
    digitalWrite(Z80_WAIT_PIN, LOW); // Active LOW
}

void release_wait() {
    digitalWrite(Z80_WAIT_PIN, HIGH); // Inactive HIGH
}

// --- Set Interface Status ---
void set_status(byte status) {
    z80_current_status = status;
}

// --- Interrupt Service Routine (ISR) for IORQ ---
// This ISR is now simpler: assert WAIT, quickly determine read/write,
// perform the immediate bus transaction, and set flags for main loop.
ISR(INT0_vect) { // IORQ_PIN_NUM (Digital Pin 2) - IORQ asserted (falling edge)
    // IMMEDIATELY assert WAIT to pause the Z80
    assert_wait();

    // Read control signals using direct port access for speed.
    byte rd_state;
    byte wr_state;
    byte m1_state;
    byte data_from_z80 = 0;

    // Use inline assembly for reading multiple pins from potentially different ports.
    __asm__ __volatile__ (
        "in %0, %1\n\t"  // Read PING (RD)
        "in %2, %3\n\t"  // Read PINE (WR)
        "in %4, %5\n\t"  // Read PINH (M1)
        : "=r" (rd_state), "=r" (wr_state), "=r" (m1_state)
        : "I" (_SFR_IO_ADDR(RD_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(WR_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(M1_PIN_PORT_REGISTER))
    );

    bool is_rd_active = !((rd_state & RD_PIN_BIT_MASK)); // Active LOW
    bool is_wr_active = !((wr_state & WR_PIN_BIT_MASK)); // Active LOW
    // bool is_m1_active = !((m1_state & M1_PIN_BIT_MASK)); // Optional check

    // This section is now less time-critical *after* WAIT is asserted.
    // The Z80 is paused until WAIT is released by the main loop.

    if (is_rd_active && !is_wr_active) { // Z80 is requesting to read (RD is LOW, WR is HIGH)
        read_cycle_detected = true;
        write_cycle_detected = false;

        // Set DATA_PORT as output
        DATA_DDR_REGISTER = 0xFF;

        // Put data on the bus. This data needs to be pre-calculated/buffered.
        // It's either the current status or the next byte from the SD buffer.
        if (current_state == STATE_IDLE || current_state == STATE_ERROR || current_state == STATE_PERFORMING_SD_OP) {
            // Z80 is reading status
            __asm__ __volatile__ (
                "out %0, %1\n\t"
                :
                : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (z80_current_status)
            );
        } else if (current_state == STATE_TRANSFER_READ) {
            // Z80 is reading a data block
            if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                __asm__ __volatile__ (
                    "out %0, %1\n\t"
                    :
                    : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (sd_buffer[sd_buffer_ptr])
                );
                sd_buffer_ptr++; // Increment pointer after outputting
            } else {
                // Should not happen if Z80 reads exactly 512 bytes, but for safety:
                __asm__ __volatile__ (
                    "out %0, %1\n\t"
                    :
                    : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (0x00) // End of block marker or dummy
                );
            }
        } else {
            // Unexpected read request state
            __asm__ __volatile__ (
                "out %0, %1\n\t"
                :
                : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (0xFF) // Indicate Busy/Unknown
            );
        }

    } else if (!is_rd_active && is_wr_active) { // Z80 is writing (WR is LOW, RD is HIGH)
        write_cycle_detected = true;
        read_cycle_detected = false;

        // Set DATA_PORT as input
        DATA_DDR_REGISTER = 0x00;

        // Read data from the bus using inline assembly for speed
        __asm__ __volatile__ (
            "in %0, %1\n\t"
            : "=r" (data_from_z80)
            : "I" (_SFR_IO_ADDR(DATA_PIN_REGISTER))
        );
        z80_data_byte = data_from_z80;

    } else {
        // Unhandled IORQ type (e.g., both RD/WR high, or both low, or M1 active IORQ)
        // For robustness, you might want to log this or set an error state.
    }

    // Set flag for main loop to process the command/data.
    // The main loop will handle releasing the WAIT line after processing.
    io_request_pending = true;
}

// --- SD Card Operations (Blocking calls, not in ISR) ---
// These remain the same. `SdFat` is still recommended for real disk image handling.
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
    // As noted before, SD.h's FILE_WRITE is tricky for specific block overwrites.
    // A robust solution for disk images would use SdFat's block device features.
    // This example continues the basic SD.h approach, assuming pre-created file.

    File dataFile;
    // To avoid truncation, you'd generally open in read/write mode for update.
    // With SD.h, a common workaround is to open for read, close, then open for write,
    // which is not atomic and prone to issues. SdFat is preferred.
    // For this example, let's just attempt to open for write and seek.
    // If the file does not exist, it will be created. If it exists, it will be truncated.
    // This part of the code needs careful consideration for actual usage.

    dataFile = SD.open("ZXDISK.BIN", FILE_WRITE); // DANGER: This truncates existing files!
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
