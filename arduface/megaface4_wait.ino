#include <SPI.h>
#include <SD.h>

// --- Pin Definitions (Adjust these based on your actual wiring) ---
// Data Bus: PORTA (D0-D7) -> Arduino Mega Pins 22-29
#define DATA_PORT_REGISTER    PORTA // Register to write data to
#define DATA_DDR_REGISTER     DDRA  // Data Direction Register for PORTA
#define DATA_PIN_REGISTER     PINA  // Register to read data from

// Address Bus (A0-A2 for sub-port decoding): PORTC (PC0-PC2) -> Arduino Mega Pins 37-39
#define ADDR_LSB_PIN_REGISTER PINC   // Register to read A0-A2
#define ADDR_LSB_BIT_MASK     0x07   // Mask for A0, A1, A2

// Control Signals (AVR Port and Bit for fast inline assembly access)
// IMPORTANT: VERIFY THESE AGAINST YOUR MEGA'S PINOUT!
// Example: RD_PIN is Digital Pin 4 (PG5 on PORTG)
#define IORQ_PIN_NUM        2       // INT0. Z80 IORQ.
#define RD_PIN_PORT_REGISTER  PING  // Z80 RD pin's port register
#define RD_PIN_BIT_MASK       (1 << PG5) // Z80 RD pin's bit mask (Digital Pin 4)
#define WR_PIN_PORT_REGISTER  PINE  // Z80 WR pin's port register
#define WR_PIN_BIT_MASK       (1 << PE3) // Z80 WR pin's bit mask (Digital Pin 5)
#define M1_PIN_PORT_REGISTER  PINH  // Z80 M1 pin's port register
#define M1_PIN_BIT_MASK       (1 << PH3) // Z80 M1 pin's bit mask (Digital Pin 6)

// WAIT Signal (Arduino Output to Z80 WAIT input)
#define WAIT_PIN_PORT_REGISTER PORTH // Z80 WAIT pin's port register
#define WAIT_PIN_BIT_MASK      (1 << PH4) // Z80 WAIT pin's bit mask (Digital Pin 7)
#define WAIT_PIN_DDR           DDRH   // DDR for WAIT pin

#define RESET_Z80_PIN       8      // Z80 RESET detection

// SD Card Pins (Standard SPI)
#define SD_CS_PIN           53 // SS pin on Mega, often used for SD Card CS
#define SD_MOSI_PIN         51
#define SD_MISO_PIN         50
#define SD_SCK_PIN          52

// --- I/O Port Definitions (Virtual Ports for the Arduino) ---
// These are sub-addresses of our base I/O port, determined by A0-A2
#define PORT_DATA_RW        0x00 // Z80 OUT (BASE), data / IN A, (BASE)
#define PORT_COMMAND_STATUS 0x01 // Z80 OUT (BASE+1), command / IN A, (BASE+1)
#define PORT_ADDR_LSB       0x02 // Z80 OUT (BASE+2), LSB
#define PORT_ADDR_MID       0x03 // Z80 OUT (BASE+3), MID
#define PORT_ADDR_MSB       0x04 // Z80 OUT (BASE+4), MSB

// Base I/O Port for Z80. We need to decode this in software using higher address lines (e.g., A7, A6, A5 etc.)
// For simplicity, we assume an external logic or a specific A8-A15 combination selects this module,
// and the Arduino only looks at IORQ and A0-A2 for sub-port selection.
// If you want full 16-bit address decoding on Arduino, you'd need more address pins.
// For now, any IORQ triggers the ISR, and A0-A2 determine the port type.
// You would ideally have external logic or other Arduino pins checking higher address lines (e.g., A7, A6, A5)
// to ensure it's *our* base port (e.g., 0xFB).
// Example: if A7=1, A6=1, A5=1, A4=1, A3=0, A2=1, A1=0, A0=1 -> 0xF5 (for example)
// The simplest is just checking A0-A2 for function, and assuming it's *our* device.

// --- Global Volatile Variables for Communication ---
volatile byte z80_current_status = 0x00; // 0x00=READY, 0xFF=BUSY, 0xFE=ERROR
volatile uint32_t current_sd_block_address = 0; // 24-bit address

// --- SD Card Buffering ---
#define SD_BLOCK_SIZE       512
byte sd_buffer[SD_BLOCK_SIZE];
volatile unsigned int sd_buffer_ptr = 0; // Pointer for current read/write block transfer

// --- Command/State Variables for Main Loop ---
enum InterfaceState {
    STATE_IDLE,                 // Ready for new command from Z80
    STATE_PERFORMING_SD_OP,     // Arduino is busy with SD card (e.g., read/write block)
    STATE_ERROR                 // An error occurred
};
volatile InterfaceState current_state = STATE_IDLE;
volatile byte command_received = 0; // Command received from Z80, processed in main loop
volatile bool new_command_flag = false; // Set by ISR when a command is written


// --- Function Prototypes ---
void init_bus_pins();
void set_wait_state(bool active); // true for wait, false for no wait
ISR(INT0_vect); // IORQ interrupt handler
bool read_sd_block_from_storage(uint32_t block_address); // SD card reads
bool write_sd_block_to_storage(uint32_t block_address); // SD card writes
void set_status(byte status);

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Arduino ZX Spectrum SD Interface Booting...");

    init_bus_pins();

    set_wait_state(true); // Hold Z80 in WAIT while initializing SD
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        set_status(0xFE); // Signal ERROR
        current_state = STATE_ERROR;
    } else {
        Serial.println("SD Card initialized.");
        set_status(0x00); // Signal READY
        current_state = STATE_IDLE;
    }
    set_wait_state(false); // Release WAIT after initialization
}

// --- Main Loop ---
void loop() {
    // This loop primarily handles time-consuming SD card operations.
    // The ISR directly serves Z80 requests for data/status or receives command/address bytes.

    if (new_command_flag) {
        // A new command has been received by the ISR. Process it.
        cli(); // Disable interrupts while accessing shared flags/variables
        byte cmd = command_received;
        new_command_flag = false; // Clear flag
        sei(); // Re-enable interrupts

        Serial.print("Processing Command: "); Serial.println(cmd, HEX);
        set_status(0xFF); // Indicate busy while processing command

        switch (cmd) {
            case 0x01: // Command: Initialize SD Card
                Serial.println("Cmd: Init SD Card (from main loop)");
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

            case 0x03: // Command: Read SD Block (Assumes current_sd_block_address is set by Z80 writes)
                Serial.print("Cmd: Read SD Block @ "); Serial.println(current_sd_block_address, HEX);
                current_state = STATE_PERFORMING_SD_OP;
                if (read_sd_block_from_storage(current_sd_block_address)) {
                    Serial.println("Block read into buffer.");
                    set_status(0x00); // Ready for Z80 to read data
                    current_state = STATE_IDLE; // Now ready to serve Z80 read requests
                } else {
                    Serial.println("Failed to read block.");
                    set_status(0xFE);
                    current_state = STATE_ERROR;
                }
                break;

            case 0x04: // Command: Write SD Block (Assumes current_sd_block_address set, and buffer will be filled by Z80)
                Serial.print("Cmd: Write SD Block @ "); Serial.println(current_sd_block_address, HEX);
                Serial.println("Awaiting 512 bytes from Z80 to fill buffer...");
                sd_buffer_ptr = 0; // Reset pointer for Z80 to write to buffer
                set_status(0x00); // Ready for Z80 to send data
                current_state = STATE_IDLE; // Back to idle, ISR will handle data transfer
                break;

            default:
                Serial.print("Unknown Command: "); Serial.println(cmd, HEX);
                set_status(0xFE);
                current_state = STATE_ERROR;
                break;
        }
    }

    // If we were in STATE_TRANSFER_WRITE and the buffer is now full, write it to SD.
    // This needs to be outside the command processing block, as it's a consequence of Z80 data writes.
    if (current_state == STATE_IDLE && sd_buffer_ptr >= SD_BLOCK_SIZE) {
        Serial.println("Buffer full, performing SD write to storage.");
        set_status(0xFF); // Busy during SD write
        current_state = STATE_PERFORMING_SD_OP;
        if (write_sd_block_to_storage(current_sd_block_address)) {
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

    // Reset status to READY if an error occurred and Z80 is not actively requesting.
    if (current_state == STATE_ERROR && digitalRead(IORQ_PIN_NUM) == HIGH) { // If IORQ is inactive
        set_status(0xFE); // Keep signalling error
    } else if (current_state == STATE_IDLE && z80_current_status != 0x00) {
        set_status(0x00); // If idle and not ready, make ready
    }

    // Other background tasks or diagnostics can go here, but keep them minimal.
}


// --- Pin Initialization ---
void init_bus_pins() {
    // Data Bus (PORTA) - Initially inputs
    DATA_DDR_REGISTER = 0x00; // All pins as inputs (D0-D7)

    // Address Bus (PORTC for A0-A2) - Inputs
    DDRC &= ~ADDR_LSB_BIT_MASK; // Clear bits for A0-A2, make them inputs
    PORTC &= ~ADDR_LSB_BIT_MASK; // Disable pull-ups if not needed

    // Control Signals: Configure as inputs with internal pull-ups (active LOW)
    pinMode(IORQ_PIN_NUM, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP); // Z80 RD pin (Digital Pin 4)
    pinMode(5, INPUT_PULLUP); // Z80 WR pin (Digital Pin 5)
    pinMode(6, INPUT_PULLUP); // Z80 M1 pin (Digital Pin 6)
    pinMode(RESET_Z80_PIN, INPUT_PULLUP); // Z80 RESET detection

    // WAIT Signal (Arduino Output to Z80 WAIT input)
    WAIT_PIN_DDR |= WAIT_PIN_BIT_MASK; // Set WAIT pin as output
    set_wait_state(false); // Initially no wait state (HIGH)

    // Configure External Interrupt for IORQ (Digital Pin 2, INT0)
    attachInterrupt(digitalPinToInterrupt(IORQ_PIN_NUM), isr_iorq_handler, FALLING);
}

// --- Set Interface Status ---
void set_status(byte status) {
    z80_current_status = status;
}

// --- WAIT State Control ---
void set_wait_state(bool active) {
    if (active) {
        WAIT_PIN_PORT_REGISTER &= ~WAIT_PIN_BIT_MASK; // Pull WAIT LOW (active)
    } else {
        WAIT_PIN_PORT_REGISTER |= WAIT_PIN_BIT_MASK;  // Pull WAIT HIGH (inactive)
    }
}

// --- Interrupt Service Routine (ISR) for IORQ ---
// This ISR now handles the entire I/O handshake by asserting WAIT.
ISR(INT0_vect) { // IORQ_PIN_NUM (Digital Pin 2) - IORQ asserted (falling edge)

    set_wait_state(true); // Assert WAIT immediately to pause Z80

    // Read control signals and address lines using direct port access for speed.
    byte rd_state;
    byte wr_state;
    byte m1_state;
    byte addr_lsb; // A0-A2

    // Read A0-A2 from PINC directly
    __asm__ __volatile__ (
        "in %0, %1\n\t"  // Read PINC (A0-A2)
        : "=r" (addr_lsb)
        : "I" (_SFR_IO_ADDR(ADDR_LSB_PIN_REGISTER))
    );
    addr_lsb &= ADDR_LSB_BIT_MASK; // Mask to get only A0-A2

    // Read RD, WR, M1 states
    __asm__ __volatile__ (
        "in %0, %1\n\t"  // Read RD_PIN_PORT_REGISTER
        "in %2, %3\n\t"  // Read WR_PIN_PORT_REGISTER
        "in %4, %5\n\t"  // Read M1_PIN_PORT_REGISTER
        : "=r" (rd_state), "=r" (wr_state), "=r" (m1_state)
        : "I" (_SFR_IO_ADDR(RD_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(WR_PIN_PORT_REGISTER)),
          "I" (_SFR_IO_ADDR(M1_PIN_PORT_REGISTER))
    );

    bool is_rd_active = !((rd_state & RD_PIN_BIT_MASK)); // Active LOW
    bool is_wr_active = !((wr_state & WR_PIN_BIT_MASK)); // Active LOW
    bool is_m1_active = !((m1_state & M1_PIN_BIT_MASK)); // Active LOW

    // Valid I/O cycle (IORQ is active, M1 is NOT active)
    if (!is_m1_active && (is_rd_active || is_wr_active)) {

        if (is_wr_active) { // Z80 is writing to Arduino
            // Set DATA_PORT as input
            DATA_DDR_REGISTER = 0x00;

            // Wait for WR to go HIGH (Z80 has finished putting data on bus)
            byte temp_wr_state;
            do {
                __asm__ __volatile__ (
                    "in %0, %1\n\t" : "=r" (temp_wr_state) : "I" (_SFR_IO_ADDR(WR_PIN_PORT_REGISTER))
                );
            } while (!(temp_wr_state & WR_PIN_BIT_MASK)); // Wait for WR_PIN_BIT_MASK to be HIGH

            // Read data from the bus
            byte data_from_z80;
            __asm__ __volatile__ (
                "in %0, %1\n\t"
                : "=r" (data_from_z80)
                : "I" (_SFR_IO_ADDR(DATA_PIN_REGISTER))
            );

            // Process data based on sub-port (A0-A2)
            switch (addr_lsb) {
                case PORT_COMMAND_STATUS: // Write to Command Register
                    command_received = data_from_z80;
                    new_command_flag = true; // Signal main loop to process command
                    break;
                case PORT_DATA_RW: // Write data for block transfer
                    if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                        sd_buffer[sd_buffer_ptr++] = data_from_z80;
                    }
                    break;
                case PORT_ADDR_LSB:
                    current_sd_block_address = (current_sd_block_address & 0xFFFF00) | data_from_z80;
                    break;
                case PORT_ADDR_MID:
                    current_sd_block_address = (current_sd_block_address & 0xFF00FF) | ((uint32_t)data_from_z80 << 8);
                    break;
                case PORT_ADDR_MSB:
                    current_sd_block_address = (current_sd_block_address & 0x00FFFF) | ((uint32_t)data_from_z80 << 16);
                    break;
                default:
                    // Unhandled write port
                    break;
            }

        } else if (is_rd_active) { // Z80 is reading from Arduino
            // Set DATA_PORT as output
            DATA_DDR_REGISTER = 0xFF;

            byte data_to_send = 0xFF; // Default to busy/unknown

            // Provide data based on sub-port (A0-A2)
            switch (addr_lsb) {
                case PORT_COMMAND_STATUS: // Read from Status Register
                    data_to_send = z80_current_status;
                    break;
                case PORT_DATA_RW: // Read data for block transfer
                    if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                        data_to_send = sd_buffer[sd_buffer_ptr];
                        // Increment ptr AFTER data is on bus, Z80 reads it *next* cycle.
                        // For LDIR, Z80 will send 512 separate RD pulses.
                        // The ISR is re-entered for each. So ptr increment happens here.
                        sd_buffer_ptr++;
                        if (sd_buffer_ptr >= SD_BLOCK_SIZE) {
                            // Last byte of block sent. Reset pointer for next block.
                            sd_buffer_ptr = 0;
                            // The main loop will determine next state based on context
                            // (e.g., if Z80 has requested another block read).
                        }
                    } else {
                        // Should not happen if Z80 reads exactly 512 bytes
                        data_to_send = 0x00; // Indicate end of block/no more data
                    }
                    break;
                default:
                    // Unhandled read port
                    data_to_send = 0xFF; // Signal busy or unknown
                    break;
            }

            // Put data on the bus
            __asm__ __volatile__ (
                "out %0, %1\n\t"
                :
                : "I" (_SFR_IO_ADDR(DATA_PORT_REGISTER)), "r" (data_to_send)
            );
            set_wait_state(false); // Release WAIT, allow Z80 to continue
            // Wait for RD to go HIGH (Z80 has latched data)
            byte temp_rd_state;
            do {
                __asm__ __volatile__ (
                    "in %0, %1\n\t" : "=r" (temp_rd_state) : "I" (_SFR_IO_ADDR(RD_PIN_PORT_REGISTER))
                );
            } while (!(temp_rd_state & RD_PIN_BIT_MASK)); // Wait for RD_PIN_BIT_MASK to be HIGH

            // After Z80 has read, return DATA_PORT to input (tri-state)
            DATA_DDR_REGISTER = 0x00;
        }
    }

    
}

// --- SD Card Operations (Blocking calls, not in ISR) ---
// These functions are called from the main loop and are therefore allowed to be slow.
bool read_sd_block_from_storage(uint32_t block_address) {
    File dataFile;
    // Using SdFat for true block access is highly recommended here.
    // For demonstration with standard SD.h, we use a single file and seek.
    // Make sure "ZXDISK.BIN" exists and is large enough on your SD card.
    dataFile = SD.open("ZXDISK.BIN", FILE_READ);
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesRead = dataFile.read(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            return (bytesRead == SD_BLOCK_SIZE);
        }
        dataFile.close();
    }
    return false;
}

bool write_sd_block_to_storage(uint32_t block_address) {
    File dataFile;
    // For writing to specific blocks, you need a file opened in R/W mode
    // (e.g., O_RDWR with SdFat) or a pre-existing large file.
    // SD.h's FILE_WRITE truncates if the file already exists, which is bad for disk images.
    // This is a placeholder; a robust implementation needs proper file mode or SdFat.
    dataFile = SD.open("ZXDISK.BIN", FILE_WRITE); // DANGER: WILL TRUNCATE!
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesWritten = dataFile.write(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            return (bytesWritten == SD_BLOCK_SIZE);
        }
        dataFile.close();
    }
    return false;
}
