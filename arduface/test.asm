; --- ZX Spectrum Z80 Assembly Code Example ---
; Assemble with Pasmo, SjASMPlus, etc.

ORG 0x8000 ; Or any suitable free memory location

SD_PORT EQU 0xFB ; The I/O port the Arduino is connected to

; --- Common Macros / Routines ---
.macro WAIT_FOR_READY
    push af
    .loop:
        in a, (SD_PORT)
        cp 0x00 ; Check for READY (0x00)
        jp nz, .loop ; Loop if not ready
    pop af
.endm

.macro CHECK_FOR_ERROR
    push af
    in a, (SD_PORT)
    cp 0xFE ; Check for ERROR (0xFE)
    jp z, SD_ERROR_HANDLER ; Jump to error handler if error
    pop af
.endm

; --- SD Card Interface Routines ---

; Initialize SD Card
; Call: CALL SD_INIT
SD_INIT:
    ld a, 0x01       ; Command: Initialize SD Card
    out (SD_PORT), a
    WAIT_FOR_READY   ; Wait for Arduino to become ready
    CHECK_FOR_ERROR
    ret

; Set SD Block Address
; Call: LD HL, BlockAddress (24-bit, stored as LSB, MID, MSB in memory)
;       CALL SD_SET_ADDR
SD_SET_ADDR:
    ld a, (hl)       ; Get LSB
    out (SD_PORT), a ; Send LSB as command 0x10 followed by data
    ld a, 0x10       ; Command for LSB
    out (SD_PORT), a
    WAIT_FOR_READY
    inc hl
    ld a, (hl)       ; Get MID
    out (SD_PORT), a ; Send MID as command 0x11 followed by data
    ld a, 0x11       ; Command for MID
    out (SD_PORT), a
    WAIT_FOR_READY
    inc hl
    ld a, (hl)       ; Get MSB
    out (SD_PORT), a ; Send MSB as command 0x12 followed by data
    ld a, 0x12       ; Command for MSB
    out (SD_PORT), a
    WAIT_FOR_READY
    CHECK_FOR_ERROR
    ret

; Read SD Block into Z80 memory
; Call: LD HL, DestinationAddress (where to store 512 bytes)
;       LD DE, BlockNumber (24-bit value, needs to be loaded into a specific format for SET_ADDR)
;       CALL SD_READ_BLOCK
SD_READ_BLOCK:
    ; Assuming HL points to the 24-bit block address (LSB, MID, MSB)
    push hl          ; Save HL
    call SD_SET_ADDR ; First set the block address on the Arduino
    pop hl           ; Restore HL (now points to destination buffer)

    ld a, 0x03       ; Command: Read Block
    out (SD_PORT), a
    WAIT_FOR_READY   ; Wait for Arduino to load the block into its buffer
    CHECK_FOR_ERROR

    ; Now read 512 bytes from the Arduino's data port
    ld bc, (SD_PORT << 8) | 512 ; C = port, B = count
    ldir             ; Transfer 512 bytes from (C) to (HL)
    ret

; Write SD Block from Z80 memory
; Call: LD HL, SourceAddress (512 bytes to write)
;       LD DE, BlockNumber (24-bit value)
;       CALL SD_WRITE_BLOCK
SD_WRITE_BLOCK:
    ; Assuming HL points to the 24-bit block address (LSB, MID, MSB)
    push hl          ; Save HL
    call SD_SET_ADDR ; First set the block address on the Arduino
    pop hl           ; Restore HL (now points to source buffer)

    ld a, 0x04       ; Command: Write Block (Arduino will enter STATE_TRANSFER_WRITE)
    out (SD_PORT), a
    WAIT_FOR_READY   ; Wait for Arduino to acknowledge and be ready to receive data
    CHECK_FOR_ERROR

    ; Now write 512 bytes to the Arduino's data port
    ld bc, (SD_PORT << 8) | 512 ; C = port, B = count
    otir             ; Transfer 512 bytes from (HL) to (C)
    WAIT_FOR_READY   ; Wait for Arduino to complete the write to SD
    CHECK_FOR_ERROR
    ret

; --- Error Handler (Example) ---
SD_ERROR_HANDLER:
    ; Basic error display or loop
    ld hl, ERROR_MSG
    call PRINT_STRING
    halt ; Stop execution
    jp $ ; Infinite loop

ERROR_MSG:
    defb "SD Error!", 0

; --- Utility for Printing String (Example) ---
PRINT_STRING:
    ld a, (hl)
    or a
    ret z
    call 0x1601 ; ROM routine for PRINT_CHAR
    inc hl
    jp PRINT_STRING

; --- Main Program Flow Example ---
MAIN:
    call SD_INIT        ; Initialize the SD card interface

    ; Example: Read Block 0 from SD into 0x8100
    ld hl, 0x8100       ; Destination address
    ld de, 0x000000     ; Block 0 (needs to be prepared as 3 bytes in memory)
    ld (BLOCK_0_ADDR), de ; Store 24-bit block address
    push hl             ; Save destination
    ld hl, BLOCK_0_ADDR ; Point HL to the 24-bit address
    call SD_READ_BLOCK
    pop hl              ; Restore destination (now has the data)

    ; Data from block 0 is now at 0x8100.
    ; You could now, for example, load a screen from here, or a program.
    ; Example: Display 1st char of the block
    ld a, (0x8100)
    call 0x1601 ; PRINT_CHAR

    ; Example: Write Block 1 from 0x8200 to SD
    ld hl, 0x8200       ; Source address (fill this with data)
    ld de, 0x000001     ; Block 1
    ld (BLOCK_1_ADDR), de
    push hl
    ld hl, BLOCK_1_ADDR
    call SD_WRITE_BLOCK
    pop hl

    jp $ ; Loop forever or return

BLOCK_0_ADDR:
    defb 0, 0, 0 ; Placeholder for 24-bit block address

BLOCK_1_ADDR:
    defb 1, 0, 0 ; Placeholder for 24-bit block address

END MAIN
