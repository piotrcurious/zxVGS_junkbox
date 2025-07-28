; --- ZX Spectrum Z80 Assembly Code Example (Adjusted for WAIT states) ---
; Assemble with Pasmo, SjASMPlus, etc.

ORG 0x8000 ; Or any suitable free memory location

; --- Arduino I/O Port Definitions ---
; Base port (arbitrary, depends on higher address line decoding if any)
; For simplicity, let's assume the Arduino responds on base address &HFA (250 decimal)
; and uses A0-A2 for sub-port selection.
; So, Z80 OUT (C), A will have C as the desired port.
; Example mapping:
; Z80 Port = A_high | A_low. A_low will be 0-7, mapping to A0-A2.
; e.g. if Arduino is decoded for &HFA-&HFF, then:
BASE_PORT       EQU 0xFA ; Example base for Arduino interface
DATA_RW_PORT    EQU BASE_PORT | 0x00 ; A2=0, A1=0, A0=0 -> 0xFA
COMMAND_PORT    EQU BASE_PORT | 0x01 ; A2=0, A1=0, A0=1 -> 0xFB
ADDR_LSB_PORT   EQU BASE_PORT | 0x02 ; A2=0, A1=1, A0=0 -> 0xFC
ADDR_MID_PORT   EQU BASE_PORT | 0x03 ; A2=0, A1=1, A0=1 -> 0xFD
ADDR_MSB_PORT   EQU BASE_PORT | 0x04 ; A2=1, A1=0, A0=0 -> 0xFE
STATUS_PORT     EQU COMMAND_PORT     ; Same port as COMMAND, read for status

; --- Commands to Arduino ---
CMD_INIT_SD     EQU 0x01
CMD_READ_BLOCK  EQU 0x03
CMD_WRITE_BLOCK EQU 0x04

; --- Status Codes (read from STATUS_PORT) ---
STATUS_READY    EQU 0x00
STATUS_BUSY     EQU 0xFF
STATUS_ERROR    EQU 0xFE


; --- SD Card Interface Routines ---

; Check for Status (can be used to poll for READY/ERROR, but not strictly needed with WAIT)
; Call: IN A, (STATUS_PORT) ; A will contain status
SD_GET_STATUS:
    in a, (STATUS_PORT)
    ret

; Initialize SD Card
; Call: CALL SD_INIT
SD_INIT:
    ld a, CMD_INIT_SD
    out (COMMAND_PORT), a
    ; Z80 can continue immediately, Arduino will handle the long-running init.
    ; Optionally, poll status later to confirm success:
    ; CALL SD_POLL_UNTIL_READY
    ret

; Poll Status until Arduino is READY (0x00) or ERROR (0xFE)
SD_POLL_UNTIL_READY:
    push af
.loop:
    in a, (STATUS_PORT)
    cp STATUS_READY
    ret z
    cp STATUS_ERROR
    jp z, SD_ERROR_HANDLER
    jp .loop
    pop af
    ret


; Set SD Block Address
; Call: LD HL, BlockAddress (24-bit, stored as LSB, MID, MSB in memory)
;       CALL SD_SET_ADDR
SD_SET_ADDR:
    ld a, (hl)       ; Get LSB
    out (ADDR_LSB_PORT), a ; Arduino WAITs here
    inc hl
    ld a, (hl)       ; Get MID
    out (ADDR_MID_PORT), a ; Arduino WAITs here
    inc hl
    ld a, (hl)       ; Get MSB
    out (ADDR_MSB_PORT), a ; Arduino WAITs here
    ret


; Read SD Block into Z80 memory
; Call: LD HL, DestinationAddress (where to store 512 bytes)
;       LD DE, BlockNumber (24-bit value: LSB in E, MID in D, MSB elsewhere, then push to a 3-byte mem location)
;       CALL SD_READ_BLOCK
SD_READ_BLOCK:
    push hl          ; Save destination HL
    push de          ; Save DE (block number)
    ld (BLOCK_ADDR_TEMP), e
    ld (BLOCK_ADDR_TEMP+1), d
    ; If DE has only 16-bit, you need to load MSB separately if block > 65535
    ; For 24-bit, you need a 3-byte memory location like BLOCK_ADDR_TEMP.
    ld hl, BLOCK_ADDR_TEMP ; Point HL to the 24-bit address (LSB, MID, MSB)
    call SD_SET_ADDR ; Send block address to Arduino
    pop de           ; Restore DE
    pop hl           ; Restore HL (destination)

    ld a, CMD_READ_BLOCK ; Command: Read Block
    out (COMMAND_PORT), a ; Arduino will now read block into its buffer (long operation, Z80 continues or polls)
    call SD_POLL_UNTIL_READY ; Wait for Arduino to signal it's ready with data

    ; Now read 512 bytes from the Arduino's DATA_RW_PORT
    ld bc, (DATA_RW_PORT << 8) | 512 ; C = port, B = count
    ldir             ; Transfer 512 bytes from (C) to (HL). Arduino WAITs for each byte.
    ret


; Write SD Block from Z80 memory
; Call: LD HL, SourceAddress (512 bytes to write)
;       LD DE, BlockNumber (24-bit value)
;       CALL SD_WRITE_BLOCK
SD_WRITE_BLOCK:
    push hl          ; Save source HL
    push de          ; Save DE (block number)
    ld (BLOCK_ADDR_TEMP), e
    ld (BLOCK_ADDR_TEMP+1), d
    ld hl, BLOCK_ADDR_TEMP ; Point HL to the 24-bit address
    call SD_SET_ADDR ; Send block address to Arduino
    pop de
    pop hl           ; Restore HL (source)

    ld a, CMD_WRITE_BLOCK ; Command: Write Block
    out (COMMAND_PORT), a ; Arduino will now prepare to receive data
    call SD_POLL_UNTIL_READY ; Wait for Arduino to signal it's ready to receive data

    ; Now write 512 bytes to the Arduino's DATA_RW_PORT
    ld bc, (DATA_RW_PORT << 8) | 512 ; C = port, B = count
    otir             ; Transfer 512 bytes from (HL) to (C). Arduino WAITs for each byte.
    call SD_POLL_UNTIL_READY ; Wait for Arduino to complete the write to SD (long operation)
    ret


; --- Error Handler (Example) ---
SD_ERROR_HANDLER:
    ld hl, ERROR_MSG
    call PRINT_STRING
    halt
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
    ld (BLOCK_ADDR_TEMP), 0 ; LSB of block 0
    ld (BLOCK_ADDR_TEMP+1), 0 ; MID of block 0
    ld (BLOCK_ADDR_TEMP+2), 0 ; MSB of block 0 (for 24-bit)
    call SD_READ_BLOCK

    ; Data from block 0 is now at 0x8100.
    ; Example: Display 1st char of the block
    ld a, (0x8100)
    call 0x1601 ; PRINT_CHAR

    ; Example: Write Block 1 from 0x8200 to SD
    ld hl, 0x8200       ; Source address (fill this with data, e.g., 512 bytes of 'A')
    ; For testing, fill memory with some pattern
    ld de, 0x8201
    ld bc, 511
    ld (hl), 'A'
    ldir

    ld (BLOCK_ADDR_TEMP), 1 ; LSB of block 1
    ld (BLOCK_ADDR_TEMP+1), 0 ; MID
    ld (BLOCK_ADDR_TEMP+2), 0 ; MSB
    call SD_WRITE_BLOCK

    jp $ ; Loop forever or return

BLOCK_ADDR_TEMP:
    defb 0, 0, 0 ; Temporary storage for 24-bit block address

END MAIN
