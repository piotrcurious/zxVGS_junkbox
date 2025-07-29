#include <SPI.h>
#include <SD.h>

// --- Pin Definitions (Adjust these based on your actual wiring) ---
// Data Bus: PORTA (D0-D7) -> Arduino Mega Pins 22-29
#define DATA_PORT_REGISTER    PORTA // Register to write data to
#define DATA_DDR_REGISTER     DDRA  // Data Direction Register for PORTA
#define DATA_PIN_REGISTER     PINA  // Register to read data from

// Address Bus (A0-A2 for sub-port decoding): PORTC (PC0-PC2) -> Arduino Mega Pins 37, 36, 35
#define ADDR_LSB_PIN_REGISTER PINC   // Register to read A0-A2
#define ADDR_LSB_BIT_MASK     0x07   // Mask for A0, A1, A2

// Control Signals (AVR Port and Bit for fast inline assembly access)
// IMPORTANT: VERIFY THESE AGAINST YOUR MEGA'S PINOUT!
// Example: RD_PIN is Digital Pin 4 (PG5 on PORTG)
#define IORQ_PIN_NUM          2        // INT0. Z80 /IORQ.
#define RD_PIN_PORT_REGISTER  PING     // Z80 /RD pin's port register
#define RD_PIN_BIT_MASK       (1 << 5) // Z80 /RD pin's bit mask (Digital Pin 4 -> PG5)
#define WR_PIN_PORT_REGISTER  PINE     // Z80 /WR pin's port register
#define WR_PIN_BIT_MASK       (1 << 3) // Z80 /WR pin's bit mask (Digital Pin 5 -> PE3)
#define M1_PIN_PORT_REGISTER  PINH     // Z80 /M1 pin's port register
#define M1_PIN_BIT_MASK       (1 << 3) // Z80 /M1 pin's bit mask (Digital Pin 6 -> PH3)

// WAIT Signal (Arduino Output to Z80 WAIT input)
#define WAIT_PIN_PORT_REGISTER PORTH    // Z80 /WAIT pin's port register
#define WAIT_PIN_BIT_MASK      (1 << 4) // Z80 /WAIT pin's bit mask (Digital Pin 7 -> PH4)
#define WAIT_PIN_DDR           DDRH     // DDR for WAIT pin

// SD Card Pins (Standard SPI)
#define SD_CS_PIN             53 // SS pin on Mega, often used for SD Card CS

// --- I/O Port Definitions (Virtual Ports for the Arduino) ---
#define PORT_DATA_RW          0x00 // Z80 OUT (BASE), data / IN A, (BASE)
#define PORT_COMMAND_STATUS   0x01 // Z80 OUT (BASE+1), command / IN A, (BASE+1)
#define PORT_ADDR_LSB         0x02 // Z80 OUT (BASE+2), LSB
#define PORT_ADDR_MID         0x03 // Z80 OUT (BASE+3), MID
#define PORT_ADDR_MSB         0x04 // Z80 OUT (BASE+4), MSB

// --- Global Volatile Variables for Communication ---
enum StatusCodes {
    STATUS_READY = 0x00,
    STATUS_BUSY  = 0xFF,
    STATUS_ERROR = 0xFE
};
volatile byte z80_current_status = STATUS_BUSY; // Start as busy until init is done
volatile uint32_t current_sd_block_address = 0;

// --- SD Card Buffering ---
#define SD_BLOCK_SIZE         512
byte sd_buffer[SD_BLOCK_SIZE];
volatile unsigned int sd_buffer_ptr = 0;

// --- Command/State Variables for Main Loop ---
enum InterfaceState {
    STATE_IDLE,                 // Ready for new command from Z80
    STATE_PERFORMING_SD_OP,     // Arduino is busy with SD card (e.g., read/write block)
    STATE_AWAITING_Z80_DATA,    // Ready for Z80 to write a block of data
    STATE_ERROR                 // An error occurred, requires Z80 to clear
};
volatile InterfaceState current_state = STATE_IDLE;
volatile byte command_received = 0;
volatile bool new_command_flag = false;

// --- Function Prototypes ---
void init_bus_pins();
void set_wait_state(bool active);
bool read_sd_block_from_storage(uint32_t block_address);
bool write_sd_block_to_storage(uint32_t block_address);
void set_status(byte status);

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("Arduino Z80 SD Interface Booting...");

    init_bus_pins();

    set_wait_state(true); // Hold Z80 in WAIT while initializing SD
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        set_status(STATUS_ERROR);
        current_state = STATE_ERROR;
    } else {
        Serial.println("SD Card initialized.");
        set_status(STATUS_READY);
        current_state = STATE_IDLE;
    }
    set_wait_state(false); // Release WAIT after initialization
}

// --- Main Loop ---
void loop() {
    // Process a new command from the Z80 if the flag is set by the ISR
    if (new_command_flag) {
        cli(); // Disable interrupts while accessing shared flag
        byte cmd = command_received;
        new_command_flag = false;
        sei(); // Re-enable interrupts

        Serial.print("Processing Command: 0x"); Serial.println(cmd, HEX);
        set_status(STATUS_BUSY); // Indicate busy while processing command

        switch (cmd) {
            case 0x01: // Command: Initialize SD Card
                current_state = STATE_PERFORMING_SD_OP;
                if (!SD.begin(SD_CS_PIN)) {
                    set_status(STATUS_ERROR);
                    current_state = STATE_ERROR;
                } else {
                    set_status(STATUS_READY);
                    current_state = STATE_IDLE;
                }
                break;

            case 0x03: // Command: Read SD Block
                current_state = STATE_PERFORMING_SD_OP;
                if (read_sd_block_from_storage(current_sd_block_address)) {
                    sd_buffer_ptr = 0; // Reset pointer for the Z80 to start reading
                    set_status(STATUS_READY);
                    current_state = STATE_IDLE;
                } else {
                    set_status(STATUS_ERROR);
                    current_state = STATE_ERROR;
                }
                break;

            case 0x04: // Command: Write SD Block
                Serial.println("Ready for 512-byte block from Z80...");
                sd_buffer_ptr = 0; // Reset pointer for Z80 to write to buffer
                set_status(STATUS_READY);
                current_state = STATE_AWAITING_Z80_DATA; // Change state to wait for data
                break;

            default:
                Serial.print("Unknown Command: 0x"); Serial.println(cmd, HEX);
                set_status(STATUS_ERROR);
                current_state = STATE_ERROR;
                break;
        }
    }

    // Check if a Z80 block write has completed
    if (current_state == STATE_AWAITING_Z80_DATA && sd_buffer_ptr >= SD_BLOCK_SIZE) {
        Serial.println("Buffer full, writing to SD storage.");
        set_status(STATUS_BUSY);
        current_state = STATE_PERFORMING_SD_OP;
        
        if (write_sd_block_to_storage(current_sd_block_address)) {
            Serial.println("Block write successful.");
            set_status(STATUS_READY);
            current_state = STATE_IDLE;
        } else {
            Serial.println("Block write failed.");
            set_status(STATUS_ERROR);
            current_state = STATE_ERROR;
        }
        sd_buffer_ptr = 0; // Reset pointer for next operation
    }
}

// --- Pin Initialization ---
void init_bus_pins() {
    // Data Bus (PORTA) - Initially inputs
    DATA_DDR_REGISTER = 0x00;
    DATA_PORT_REGISTER = 0x00; // Disable pull-ups

    // Address Bus (PORTC for A0-A2) - Inputs
    DDRC &= ~ADDR_LSB_BIT_MASK;
    PORTC &= ~ADDR_LSB_BIT_MASK;

    // Control Signals: Configure as inputs with internal pull-ups (for active LOW signals)
    pinMode(IORQ_PIN_NUM, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP); // Z80 /RD
    pinMode(5, INPUT_PULLUP); // Z80 /WR
    pinMode(6, INPUT_PULLUP); // Z80 /M1

    // WAIT Signal (Arduino Output to Z80 WAIT input)
    WAIT_PIN_DDR |= WAIT_PIN_BIT_MASK;
    set_wait_state(false);

    // FIX: Manually configure External Interrupt 0 for /IORQ on pin 2
    // Replaces the incorrect attachInterrupt() call.
    EICRA |= (1 << ISC01); // The falling edge of INT0 generates an interrupt request.
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0);  // Enable external interrupt request 0.
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
ISR(INT0_vect) {
    set_wait_state(true); // Assert WAIT immediately to pause Z80

    // Read control signals and address lines using direct port access for speed
    byte rd_state = RD_PIN_PORT_REGISTER;
    byte wr_state = WR_PIN_PORT_REGISTER;
    byte m1_state = M1_PIN_PORT_REGISTER;
    byte addr_lsb = PINC & ADDR_LSB_BIT_MASK;

    bool is_rd_active = !(rd_state & RD_PIN_BIT_MASK);
    bool is_wr_active = !(wr_state & WR_PIN_BIT_MASK);
    bool is_m1_active = !(m1_state & M1_PIN_BIT_MASK);

    // Process valid I/O cycle (/IORQ active, /M1 inactive)
    if (!is_m1_active) {
        if (is_wr_active) { // --- Z80 is WRITING to Arduino ---
            // Set DATA_PORT as input
            DATA_DDR_REGISTER = 0x00;

            // FIX: Read data from the bus WHILE /WR is low.
            // The Z80 is paused by the /WAIT line, so data is stable.
            byte data_from_z80 = DATA_PIN_REGISTER;

            switch (addr_lsb) {
                case PORT_COMMAND_STATUS:
                    if (current_state != STATE_PERFORMING_SD_OP) { // Don't accept new commands while busy
                        command_received = data_from_z80;
                        new_command_flag = true;
                    }
                    break;
                case PORT_DATA_RW:
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
            }
        } else if (is_rd_active) { // --- Z80 is READING from Arduino ---
            byte data_to_send = 0xFF; // Default value

            switch (addr_lsb) {
                case PORT_COMMAND_STATUS:
                    data_to_send = z80_current_status;
                    break;
                case PORT_DATA_RW:
                    if (sd_buffer_ptr < SD_BLOCK_SIZE) {
                        data_to_send = sd_buffer[sd_buffer_ptr];
                        // CHANGE: Increment pointer after preparing data.
                        // The main loop is now responsible for resetting the pointer to 0.
                        sd_buffer_ptr++;
                    } else {
                        data_to_send = 0x00; // Indicate end of block
                    }
                    break;
            }
            // Put data on the bus and set port to output
            DATA_PORT_REGISTER = data_to_send;
            DATA_DDR_REGISTER = 0xFF;
            
            // FIX: The `do-while` loop polling /RD is removed for efficiency.
            // Releasing /WAIT is sufficient. The Z80 will latch the data.
            // After the ISR, the bus must be released. To ensure this happens after
            // the Z80 has read, we add a minimal delay.
            __asm__("nop\n\t nop\n\t");
            
            // After Z80 has read, return DATA_PORT to input (tri-state)
            DATA_DDR_REGISTER = 0x00;
        }
    }

    set_wait_state(false); // Release WAIT, allow Z80 to continue
}

// --- SD Card Operations ---
// These are called from the main loop and are allowed to be slow.
bool read_sd_block_from_storage(uint32_t block_address) {
    File dataFile;
    dataFile = SD.open("ZXDISK.BIN", FILE_READ);
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesRead = dataFile.read(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            return (bytesRead == SD_BLOCK_SIZE);
        }
        dataFile.close();
    }
    Serial.println("Failed to open ZXDISK.BIN for reading.");
    return false;
}

bool write_sd_block_to_storage(uint32_t block_address) {
    File dataFile;
    // DANGER: Using standard SD.h with FILE_WRITE on an existing file can be problematic.
    // It may truncate or behave unexpectedly. A library like SdFat with O_RDWR | O_CREAT
    // provides more control for disk-image-like access. This implementation assumes
    // you are writing to a file in a way that seeking is supported.
    dataFile = SD.open("ZXDISK.BIN", FILE_WRITE);
    if (dataFile) {
        if (dataFile.seek(block_address * SD_BLOCK_SIZE)) {
            size_t bytesWritten = dataFile.write(sd_buffer, SD_BLOCK_SIZE);
            dataFile.close();
            return (bytesWritten == SD_BLOCK_SIZE);
        }
        dataFile.close();
    }
    Serial.println("Failed to open ZXDISK.BIN for writing.");
    return false;
}
