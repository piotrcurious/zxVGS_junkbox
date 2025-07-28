#include <SPI.h>
#include <SD.h>

// --- Pin Definitions (Adjust these based on your actual wiring) ---
// Data Bus: PORTA (D0-D7)
#define DATA_PORT           PORTA
#define DATA_DDR            DDRA
#define DATA_PIN_REGISTER   PINA

// Address Bus (Lower 8 bits for simple decoding): PORTC (A0-A7)
// You'll need to connect higher address lines (A8-A15) to other digital pins
// For simplicity in this example, we'll assume a limited address space is decoded by external logic
// or only a few crucial bits are checked on the Arduino.
// For full address decoding with a limited number of pins, you'd feed specific
// address bits from the Spectrum bus to specific Arduino pins.
// We'll primarily focus on the IORQ/RD/WR signals and assume a single I/O port for now.
#define ADDR_PORT_LSB       PORTC
#define ADDR_PIN_REGISTER   PINC

// Control Signals
#define IORQ_PIN            2   // INT0
#define RD_PIN              3   // INT1
#define WR_PIN              4   // Polled
#define M1_PIN              5   // Polled (Optional, for precise I/O detection)
#define RESET_Z80_PIN       6   // Z80 RESET detection

// 74LS245 Buffer Control (if used - HIGH for A->B (Arduino to Z80), LOW for B->A (Z80 to Arduino))
#define LS245_DIR_PIN       22  // Example: Digital pin PD0
#define LS245_OE_PIN        23  // Example: Digital pin PD1 (Output Enable, Active LOW)

// SD Card Pins (Standard SPI)
#define SD_CS_PIN           53 // SS pin on Mega, often used for SD Card CS
#define SD_MOSI_PIN         51
#define SD_MISO_PIN         50
#define SD_SCK_PIN          52

// --- I/O Port Definitions (Arbitrary, to be decoded by Arduino) ---
// These are examples. Your Z80 code will OUT/IN to these ports.
// Let's assume a single base I/O port, like 0xFB (251 decimal).
// The lower bits will signify command/status/data.
// For example:
// Port 0xFB: Command/Status/Data Register
// Z80 OUT 0xFB, CommandByte  (Arduino interprets CommandByte)
// Z80 IN A, (0xFB)           (Arduino returns Status/Data)

// We'll use a conceptual port address, let's say 0xFB
// The Arduino will look for (IORQ & !RD & !WR) for 0xFB as a command/data port.

// --- Global Variables for Communication ---
volatile byte z80_data_byte;      // Data read from or written to Z80
volatile word z80_address_word;   // Full 16-bit address (if fully decoded)
volatile bool io_request_pending = false;
volatile bool write_cycle = false;
volatile bool read_cycle = false;

// --- SD Card Buffering ---
#define SD_BLOCK_SIZE       512
byte sd_buffer[SD_BLOCK_SIZE];
volatile unsigned int sd_buffer_ptr = 0;
volatile bool sd_block_ready = false; // True when a block is loaded into sd_buffer for reading by Z80

// --- Command/State Variables ---
enum InterfaceState {
    STATE_IDLE,
    STATE_RECEIVING_COMMAND,
    STATE_EXECUTING_COMMAND,
    STATE_TRANSFER_READ,
    STATE_TRANSFER_WRITE,
    STATE_ERROR
};
volatile InterfaceState current_state = STATE_IDLE;
byte current_command = 0;
uint32_t current_sd_block_address = 0; // For block-based operations

// --- Function Prototypes ---
void init_bus_pins();
void configure_ls245_read_z80();
void configure_ls245_write_z80();
void disable_ls245(); // Tri-state the bus
void isr_iorq_handler();
void isr_rd_handler(); // Potentially used to trigger data output
byte read_z80_data_bus_fast();
void write_z80_data_bus_fast(byte data);
void process_command();
bool read_sd_block(uint32_t block_address);
bool write_sd_block(uint32_t block_address);


// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Arduino ZX Spectrum SD Interface Booting...");

    init_bus_pins();

    // Initialize SD Card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        // Potentially signal error via a status LED or Z80 status port
        current_state = STATE_ERROR;
        return;
    }
    Serial.println("SD Card initialized.");
    current_state = STATE_IDLE; // Ready to receive commands
}

// --- Main Loop ---
void loop() {
    // This loop primarily handles higher-level tasks and SD operations
    // that are not time-critical or don't fit in an ISR.

    if (current_state == STATE_EXECUTING_COMMAND) {
        process_command();
    }
    // Other background tasks or error handling can go here
}

// --- Pin Initialization ---
void init_bus_pins() {
    // Data Bus (PORTA) - Initially inputs, as Z80 will write command
    DATA_DDR = 0x00; // All pins as inputs
    DATA_PORT = 0x00; // Disable pull-ups if not needed

    // Address Bus (PORTC for LSB) - Inputs
    // Other address lines (A8-A15) if connected to other digital pins
    DDRC = 0x00; // All pins as inputs (PC0-PC7)

    // Control Signals
    pinMode(IORQ_PIN, INPUT_PULLUP); // Active LOW, so pull-up is good
    pinMode(RD_PIN, INPUT_PULLUP);   // Active LOW
    pinMode(WR_PIN, INPUT_PULLUP);   // Active LOW
    pinMode(M1_PIN, INPUT_PULLUP);   // Active LOW (Optional)
    pinMode(RESET_Z80_PIN, INPUT_PULLUP); // Z80 RESET detection

    // 74LS245 Control (Initially tri-state the bus to avoid contention)
    pinMode(LS245_DIR_PIN, OUTPUT);
    pinMode(LS245_OE_PIN, OUTPUT);
    digitalWrite(LS245_OE_PIN, HIGH); // Disable 74LS245 (High-Z)
    digitalWrite(LS245_DIR_PIN, LOW); // Default to Z80 -> Arduino direction

    // Configure External Interrupts
    // IORQ: Falling edge (when Z80 asserts IORQ)
    attachInterrupt(digitalPinToInterrupt(IORQ_PIN), isr_iorq_handler, FALLING);
    // RD: Falling edge (when Z80 asserts RD) - used for read cycle detection
    attachInterrupt(digitalPinToInterrupt(RD_PIN), isr_rd_handler, FALLING);

    // Ensure global interrupts are enabled (they are by default in Arduino framework)
    // sei();
}

// --- 74LS245 Control Functions ---
void configure_ls245_read_z80() { // Arduino is reading from Z80
    digitalWrite(LS245_DIR_PIN, LOW);   // Z80 -> Arduino (B to A)
    digitalWrite(LS245_OE_PIN, LOW);    // Enable buffer
    DATA_DDR = 0x00; // Ensure Arduino data pins are inputs
}

void configure_ls245_write_z80() { // Arduino is writing to Z80
    DATA_DDR = 0xFF; // Set Arduino data pins as outputs
    digitalWrite(LS245_DIR_PIN, HIGH);  // Arduino -> Z80 (A to B)
    digitalWrite(LS245_OE_PIN, LOW);    // Enable buffer
}

void disable_ls245() { // Tri-state the bus
    DATA_DDR = 0x00; // Set Arduino data pins as inputs
    digitalWrite(LS245_OE_PIN, HIGH);   // Disable buffer
}

// --- Interrupt Service Routines (ISRs) ---
// HIGHLY TIME-CRITICAL
ISR(INT0_vect) { // IORQ_PIN (Digital Pin 2) - IORQ asserted
    // Disable other interrupts temporarily if necessary
    // cli(); // Disable global interrupts

    // Check if WR is asserted (Z80 writing to Arduino)
    if (digitalRead(WR_PIN) == LOW) { // Z80 is writing
        // This is a Z80 write cycle to an I/O port
        // Read the address lines immediately. For simplicity, we'll only check if it's "our" port.
        // In a real scenario, you'd decode more address lines.
        byte lower_addr = PINC & 0x0F; // Read A0-A3 from PORTC. Adjust based on your decoding.
                                      // If you're using a specific port like 0xFB, you'd check
                                      // the relevant address bits here.
                                      // For this example, let's assume we react to *any* IORQ+WR
                                      // and the first byte sent defines the command/state.

        configure_ls245_read_z80(); // Ensure we can read from Z80
        // Use inline assembly for fastest data capture if needed.
        // For reading a byte from PORTA (connected to D0-D7), it's fast enough.
        __asm__ __volatile__ (
            "in %0, %1\n\t"
            : "=r" (z80_data_byte)
            : "I" (_SFR_IO_ADDR(PINA)) // Use PINA for reading PORTA
        );
        disable_ls245(); // Release the bus immediately

        // This is a command or data byte from Z80
        if (current_state == STATE_IDLE || current_state == STATE_RECEIVING_COMMAND) {
            // First byte is usually a command
            current_command = z80_data_byte;
            current_state = STATE_EXECUTING_COMMAND;
            // No, the command is set. Actual execution happens in loop.
            // For now, let's just confirm receipt.
            // Serial.print("Cmd Rcv: "); Serial.println(current_command, HEX);
        } else if (current_state == STATE_TRANSFER_WRITE) {
            // Z80 is sending data for a block write
            if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                sd_buffer[sd_buffer_ptr++] = z80_data_byte;
            }
            if (sd_buffer_ptr >= SD_BLOCK_SIZE) {
                // Block is full, trigger write
                current_state = STATE_EXECUTING_COMMAND; // To trigger write in loop()
            }
        }
    }
    // cli(); sei(); // Re-enable interrupts
}

ISR(INT1_vect) { // RD_PIN (Digital Pin 3) - RD asserted
    // This ISR fires when Z80 asserts RD. We need to put data on the bus.
    // Check if IORQ is also asserted (valid I/O read cycle)
    if (digitalRead(IORQ_PIN) == LOW) {
        // This is a Z80 read cycle from an I/O port
        // Read address bits here too if needed for multi-port addressing

        configure_ls245_write_z80(); // Prepare to write to Z80 bus

        byte data_to_send = 0xFF; // Default to 0xFF (no data / busy)

        if (current_state == STATE_IDLE) {
            data_to_send = 0x00; // Signal READY
        } else if (current_state == STATE_TRANSFER_READ) {
            if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                data_to_send = sd_buffer[sd_buffer_ptr++];
                if (sd_buffer_ptr >= SD_BLOCK_SIZE) {
                    current_state = STATE_IDLE; // All data sent for this block
                    sd_buffer_ptr = 0; // Reset for next transfer
                }
            } else {
                data_to_send = 0x00; // End of block, or error
                current_state = STATE_IDLE;
                sd_buffer_ptr = 0;
            }
        } else if (current_state == STATE_ERROR) {
            data_to_send = 0xFE; // Signal ERROR
        } else {
            data_to_send = 0xFF; // Signal BUSY
        }

        // Write the byte to PORTA (connected to D0-D7) using inline assembly
        __asm__ __volatile__ (
            "out %0, %1\n\t"
            :
            : "I" (_SFR_IO_ADDR(PORTA)), "r" (data_to_send) // Use PORTA for writing
        );
        // Important: You must ensure the Z80 has read the data before disabling.
        // The Z80 holds RD active for a period. The ISR is quick.
        // A slight delay or a tighter loop watching for RD to go HIGH might be needed,
        // but typically the Z80 will latch the data after RD goes LOW and before it goes HIGH.
        // For simplicity, we disable immediately. This might need fine-tuning.
        disable_ls245(); // Release bus immediately after writing.
    }
}

// --- Command Processing (non-ISR, called from loop()) ---
void process_command() {
    Serial.print("Processing Command: "); Serial.println(current_command, HEX);

    switch (current_command) {
        case 0x01: // Command: Initialize SD Card
            Serial.println("Cmd: Init SD Card");
            if (!SD.begin(SD_CS_PIN)) {
                Serial.println("SD Init Failed!");
                current_state = STATE_ERROR;
            } else {
                Serial.println("SD Init OK.");
                current_state = STATE_IDLE;
            }
            break;

        case 0x02: // Command: Set Block Address (Next 3 bytes for 24-bit address)
            // This command needs to be followed by 3 bytes of address.
            // This means the Z80 would send 0x02, then wait for ACK, then send byte1, byte2, byte3
            // This setup makes multi-byte commands tricky with just one IORQ/WR ISR.
            // A better way: Z80 OUT CMD_PORT, CMD_SET_BLOCK_ADDR
            //              Z80 OUT DATA_PORT, LSB_ADDR
            //              Z80 OUT DATA_PORT, MID_ADDR
            //              Z80 OUT DATA_PORT, MSB_ADDR
            // The ISR would have to manage a state machine: after CMD_SET_BLOCK_ADDR,
            // subsequent writes to DATA_PORT are interpreted as address bytes.
            Serial.println("Cmd: Set Block Address - Requires 3 more bytes via data port.");
            // For now, let's simplify and assume the Z80 just sends the address.
            // This is a conceptual example for a multi-byte command.
            // The Z80 will need to send these bytes after the command byte, polling for ready.
            current_state = STATE_RECEIVING_COMMAND; // Arduino awaits address bytes
            // Need a mechanism to store the 3 bytes. Could use z80_data_byte and a counter.
            // Not fully implemented here for brevity and complexity.
            // A dedicated "address" port or more sophisticated state machine is better.
            current_state = STATE_ERROR; // Indicate not implemented in this simple structure
            break;

        case 0x03: { // Command: Read SD Block (Assumes current_sd_block_address is set)
            Serial.print("Cmd: Read SD Block @ "); Serial.println(current_sd_block_address);
            if (read_sd_block(current_sd_block_address)) {
                sd_buffer_ptr = 0; // Reset pointer for Z80 to read from start
                current_state = STATE_TRANSFER_READ; // Now ready for Z80 to read data
                Serial.println("Block read into buffer. Ready for Z80.");
            } else {
                Serial.println("Failed to read block.");
                current_state = STATE_ERROR;
            }
            break;
        }
        case 0x04: { // Command: Write SD Block (Assumes current_sd_block_address set, and buffer filled)
            Serial.print("Cmd: Write SD Block @ "); Serial.println(current_sd_block_address);
            Serial.println("Awaiting 512 bytes from Z80...");
            sd_buffer_ptr = 0; // Reset pointer for Z80 to write to buffer
            current_state = STATE_TRANSFER_WRITE; // Now ready to receive data from Z80
            // The write will trigger when sd_buffer_ptr reaches 512 in the IORQ ISR.
            break;
        }

        // Example: commands for setting the 24-bit address directly.
        // These need to be written to a *different* port, or a very specific sequence.
        // For simplicity with one port, Z80 can send three bytes sequentially after a 'set address' command.
        // This requires an Arduino FSM to track which byte (LSB, MID, MSB) is coming next.
        case 0x10: { // Command: Set Block Address LSB (Z80 writes this to the control port directly after command 0x10)
            current_sd_block_address = (current_sd_block_address & 0xFFFF00) | z80_data_byte;
            Serial.print("Addr LSB: "); Serial.println(z80_data_byte, HEX);
            current_state = STATE_IDLE; // Back to idle, ready for next address byte or another command
            break;
        }
        case 0x11: { // Command: Set Block Address MID
            current_sd_block_address = (current_sd_block_address & 0xFF00FF) | ((uint32_t)z80_data_byte << 8);
            Serial.print("Addr MID: "); Serial.println(z80_data_byte, HEX);
            current_state = STATE_IDLE;
            break;
        }
        case 0x12: { // Command: Set Block Address MSB
            current_sd_block_address = (current_sd_block_address & 0x00FFFF) | ((uint32_t)z80_data_byte << 16);
            Serial.print("Addr MSB: "); Serial.println(z80_data_byte, HEX);
            current_state = STATE_IDLE;
            break;
        }

        default:
            Serial.print("Unknown Command: "); Serial.println(current_command, HEX);
            current_state = STATE_ERROR;
            break;
    }
}

// --- SD Card Operations (Blocking calls, not in ISR) ---
bool read_sd_block(uint32_t block_address) {
    File dataFile;
    // For raw block access, we'd typically use SD.readBlock() if available or
    // directly manipulate the underlying SD class methods.
    // The standard SD library often works with files.
    // To mimic block access: we could open a specific large file (e.g., "disk.img")
    // and seek to the correct offset.

    // Using SD.readBlocks for raw access (if supported by your SD library, typically not standard in Arduino SD.h)
    // For standard SD.h, you'd implement a simple FAT32 block device layer, or just use files.
    // Let's simulate a raw block read using a dummy file for now, or assume a flat disk image.

    // This is simplified: Assuming you pre-created a file "DISK.BIN" for block 0, "DISK1.BIN" for block 1, etc.
    // A better way is to open a single large file and seek.
    // For true raw block access, you might need a more advanced SD library (e.g., SdFat).

    // Let's use a dummy read for now for demonstration, or assume `SdFat` library's `readBlocks`.
    // With standard `SD.h`, you'd need to create a file "data.bin" and treat it as a contiguous disk.
    // This example will use `SD.read()` on a pre-opened file.

    // Using a specific file "ZXDISK.BIN" and seeking.
    // IMPORTANT: Make sure this file exists on your SD card.
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
    return false; // Error in reading or seeking
}

bool write_sd_block(uint32_t block_address) {
    File dataFile;
    // Similar to read, using a single file for writes.
    // Use FILE_WRITE to open for writing. Note: SD.h's FILE_WRITE truncates if file exists.
    // You need to open in update mode for specific block writes without losing data.
    // Or use SdFat library for direct block writes.

    // This is a placeholder using SD.h - direct block writes are tricky with standard SD.h.
    // It's more common to append or write to a new file.
    // For a real disk image, you'd use SdFat's block device features.

    // If writing to a file, ensure it's opened in a mode that allows overwriting specific sections
    // or just appending if it's a log. For disk images, direct block access is preferable.
    dataFile = SD.open("ZXDISK.BIN", FILE_WRITE); // This will truncate!
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
    return false; // Error in writing or seeking
}
