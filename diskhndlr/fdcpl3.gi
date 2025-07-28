;Module DSKHNDLR.FDCPL3 (ZX Spectrum +3)
;Last Modified: 01-05-06; (C) Jarek Adamski
;Corrected for GENS dialect compatibility

; Floppy Disk Controller (FDC) handler for ZX Spectrum +3
;   - Can be used independently of the operating system

; Input:        Output:
; ------------------------------------
; A  - Command     | On success:
; B  - Side        | A = 0, Z = 1
; C  - Unit        |
; D  - Cylinder    | On failure:
; E  - Sector      | A > 0, Z = 0
; HL - Address     |
; ------------------------------------
; Preserves: IX, HL', DE', BC'

; Configuration flags
dskhnd          EQU     0       ; Module configuration flags

; FDC Port Definitions (ZX Spectrum +3)
PORT_MOTOR      EQU     $1FFD   ; b3=1: MOTOR ON / b2=1: MOTOR OFF
PORT_FDC_STATUS EQU     $2FFD   ; b7=1: RQM, b6=1: DIO 
PORT_FDC_DATA   EQU     $3FFD   ; FDC Data Register

FILL_BYTE       EQU     $E5     ; Filler byte for formatted sectors

; IX Register Offsets for FDC Configuration Table
FDC_CYLINDER    EQU     0       ; Current cylinder
FDC_STEP_TIME   EQU     1       ; Step rate time
FDC_UNIT_MASK   EQU     2       ; Drive Unit Mask
FDC_GAP3_FORMAT EQU     3       ; Gap3 length for formatting
FDC_GAP3_RW     EQU     4       ; Gap3 length for Read/Write
FDC_SECTOR_LEN  EQU     5       ; Sector length code
FDC_NUM_SECTORS EQU     6       ; Number of sectors per track
FDC_DRIVE_MASK  EQU     7       ; Drive mask
FDC_MOTOR_STATUS EQU    8       ; Motor status

; -----------------------------------------------------------------------------
; Entry point for interrupt handling (passes to next module)
; -----------------------------------------------------------------------------
fphere  JP      fpnext

; -----------------------------------------------------------------------------
; Main entry point for FDC functions
; -----------------------------------------------------------------------------
        JP      fpmain

; -----------------------------------------------------------------------------
; Module Header / Version Information
; -----------------------------------------------------------------------------
        DEFW    $6002           ; Version number
        DEFW    fpnext-fphere   ; Size of interrupt handler
        DEFW    fphere          ; Start address of interrupt handler
        DEFW    fptble-fptbl0   ; Size of configuration table
        DEFW    fptbl0          ; Start address of configuration table
        DEFM    'DSKHNDLR'
        DEFM    'FDCPL3  '

; -----------------------------------------------------------------------------
; Default FDC Configuration Tables
; -----------------------------------------------------------------------------
fptbl0  DEFB    $FF             ; Current Cylinder
        DEFB    $EF             ; Step Rate Time
        DEFB    $00             ; Drive Unit Mask
        DEFB    0               ; Gap3 for formatting
        DEFB    $2A             ; Gap3 for Read/Write
        DEFB    $03             ; Sector Length
        DEFB    $05             ; Number of sectors
        DEFB    $01             ; Drive mask
        DEFB    $80             ; Motor status

fptbl1  DEFB    $FF             ; Current Cylinder
        DEFB    $EF             ; Step Rate Time
        DEFB    $01             ; Drive Unit Mask
        DEFB    0               ; Gap3 for formatting
        DEFB    $2A             ; Gap3 for Read/Write
        DEFB    $03             ; Sector Length
        DEFB    $05             ; Number of sectors
        DEFB    $02             ; Drive mask
        DEFB    $80             ; Motor status
fptble  EQU     $

; -----------------------------------------------------------------------------
; fpmon: Turn motor ON
; -----------------------------------------------------------------------------
fpmon   RES     7, (IX + FDC_MOTOR_STATUS)
        LD      BC, PORT_MOTOR
        LD      A, $0C          ; MOTOR ON and enable RAM paging
        OUT     (C), A
        RET

; -----------------------------------------------------------------------------
; fpmoff: Turn motor OFF
; -----------------------------------------------------------------------------
fpmoff  SET     7, (IX + FDC_MOTOR_STATUS)
        LD      BC, PORT_MOTOR
        LD      A, $04          ; MOTOR OFF and disable RAM paging
        OUT     (C), A
        RET

; -----------------------------------------------------------------------------
; fpwait: Waits for FDC ready and sends command/parameters
; -----------------------------------------------------------------------------
fpwait  DI                      ; Disable interrupts
        PUSH    AF
        CALL    fpmon           ; Ensure motor is on

        ; Check if head needs to be seeked
        LD      A, D            ; Cylinder number
        SRL     A               ; Cylinder / 2
        CP      (IX + FDC_CYLINDER)
        JR      Z, fpw1         ; If same cylinder, no seek needed

        ; Seek command
        LD      A, $0F          ; SEEK command
        CALL    fpcom           ; Send command
        LD      A, (IX + FDC_UNIT_MASK)
        CALL    fpout           ; Send unit byte
        LD      A, D            ; Cylinder number
        SRL     A               ; Cylinder / 2
        CALL    fpout           ; Send target cylinder

fpw0    CALL    fpsis           ; Get FDC Status
        XOR     $20             ; Check for Seek End
        XOR     (IX + FDC_UNIT_MASK)
        AND     $FB             ; Ignore head bit
        JR      NZ, fpw0        ; Loop until seek ends

        LD      (IX + FDC_CYLINDER), B

fpw1    POP     AF              ; Restore FDC command
        CALL    fpcom           ; Send the main FDC command

        ; Send command parameters
        LD      A, D            ; Cylinder number
        ADD     A, A
        ADD     A, A
        AND     $04             ; Get head bit
        OR      (IX + FDC_UNIT_MASK)
        CALL    fpout           ; Send Head and Unit

        LD      A, D            ; Cylinder number
        SRL     A               ; Cylinder / 2
        CALL    fpout           ; Send cylinder

        LD      A, D            ; Original cylinder
        AND     $01             ; Get head bit
        CALL    fpout           ; Send head number

        LD      A, E            ; Sector number
        CALL    fpout

        LD      A, (IX + FDC_SECTOR_LEN)
        CALL    fpout

        LD      A, E            ; Last sector
        CALL    fpout

        LD      A, (IX + FDC_GAP3_RW)
        CALL    fpout

        LD      A, $FF          ; Data Length
        CALL    fpout

        ; Wait for RQM
        LD      DE, $0000       ; Timeout counter
        LD      BC, PORT_FDC_STATUS
fpw2    IN      A, (C)
        RET     M               ; Return if RQM set
        DEC     DE
        LD      A, E
        OR      D
        JR      NZ, fpw2
        RET

; -----------------------------------------------------------------------------
; fpstat: Final phase status checking
; -----------------------------------------------------------------------------
fpstat  CALL    fpin            ; Read ST0
        AND     $88             ; Check for errors
        LD      E, A

        CALL    fpin            ; Read ST1
        AND     $37             ; Check for ST1 errors
        OR      E
        LD      E, A

        CALL    fpin            ; Read ST2
        AND     $33             ; Check for ST2 errors
        OR      E
        LD      E, A

        CALL    fpinb           ; Read TRK
        CALL    fpinb           ; Read HED
        CALL    fpinb           ; Read SEC
        CALL    fpinb           ; Read SIZ

        LD      A, E            ; Move status to A
        AND     A               ; Set Z flag
        RET     Z               ; Return if no errors

fperr   CALL    fphome          ; Attempt recalibrate on error
        OR      $7F             ; Set error code
        RET

; -----------------------------------------------------------------------------
; fphome: Seeks to Track 0 (Recalibrate)
; -----------------------------------------------------------------------------
fphome  LD      A, $07          ; RECALIBRATE command
        CALL    fpcom
        LD      A, (IX + FDC_UNIT_MASK)
        CALL    fpout

        LD      DE, $4000       ; Timeout
        LD      (IX + FDC_CYLINDER), E

fphom1  LD      A, $08          ; SENSE INTERRUPT STATUS
        CALL    fpcom
        CALL    fpin            ; Read ST0
        CALL    fpin            ; Read PCN
        
        ; Check if at track 0
        LD      A, $04          ; SENSE DRIVE STATUS
        CALL    fpcom
        LD      A, (IX + FDC_UNIT_MASK)
        CALL    fpout
        CALL    fpin            ; Read ST3
        CPL
        AND     $10             ; Check TR0 bit
        RET     Z               ; Return if at track 0

        DEC     DE
        LD      A, E
        OR      D
        JR      NZ, fphom1

        OR      $7F             ; Timeout error
        RET

; -----------------------------------------------------------------------------
; fpsis: SENSE INTERRUPT STATUS command
; -----------------------------------------------------------------------------
fpsis   LD      A, $08          ; SENSE INTERRUPT STATUS
        CALL    fpcom
        CALL    fpin            ; Read ST0 into A
        JP      fpinb           ; Read PCN into B and return

; -----------------------------------------------------------------------------
; fpcom: Sends command byte to FDC
; -----------------------------------------------------------------------------
fpcom   LD      BC, PORT_FDC_STATUS
fpcom1  IN      A, (C)          ; Read status
        JP      P, fpcom1       ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      NZ, fpc1        ; Jump if DIO set
        LD      BC, PORT_FDC_DATA
        OUT     (C), A          ; Send command
        RET

fpc1    LD      BC, PORT_FDC_DATA
        IN      A, (C)          ; Read garbage byte
        JP      fpcom           ; Retry

; -----------------------------------------------------------------------------
; fpout: Sends data byte to FDC
; -----------------------------------------------------------------------------
fpout   PUSH    AF              ; Save data byte
        LD      BC, PORT_FDC_STATUS
fpo1    IN      A, (C)          ; Read status
        JP      P, fpo1         ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      Z, fpo1         ; Wait for DIO=1
        LD      BC, PORT_FDC_DATA
        POP     AF              ; Restore data byte
        OUT     (C), A          ; Send data
        RET

; -----------------------------------------------------------------------------
; fpin: Reads data byte from FDC into A
; -----------------------------------------------------------------------------
fpin    LD      BC, PORT_FDC_STATUS
fpin1   IN      A, (C)          ; Read status
        JP      P, fpin1        ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      NZ, fpin1       ; Wait for DIO=0
        LD      BC, PORT_FDC_DATA
        IN      A, (C)          ; Read data
        RET

; -----------------------------------------------------------------------------
; fpinb: Reads data byte from FDC into B
; -----------------------------------------------------------------------------
fpinb   LD      BC, PORT_FDC_STATUS
fpinb1  IN      A, (C)          ; Read status
        JP      P, fpinb1       ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      NZ, fpinb1      ; Wait for DIO=0
        LD      BC, PORT_FDC_DATA
        IN      B, (C)          ; Read data into B
        RET

; -----------------------------------------------------------------------------
; fpmain: Main entry point for FDC functions
; -----------------------------------------------------------------------------
fpmain  PUSH    BC              ; Save registers
        PUSH    DE
        PUSH    HL

        AND     $0F             ; Isolate command
        JR      NZ, fpset       ; If not 0, jump to fpset

; -----------------------------------------------------------------------------
; fpinit: Function 0 - Initialize FDC
; -----------------------------------------------------------------------------
fpinit  ; Check for FDC presence
        LD      HL, $0010       ; Counter for detection
        LD      BC, PORT_FDC_STATUS
fpich1  IN      A, (C)
        INC     A               ; Check if port returns $FF
        JR      NZ, fpije       ; If not $FF, FDC present
        DEC     HL
        LD      A, L
        OR      H
        JR      NZ, fpich1      ; Loop with counter

        LD      E, $00          ; No FDC found
        JR      fpiex

fpije   CALL    fpset2          ; Set FDC parameters
        CALL    fpmon           ; Turn motor on

        ; Detect drives
        LD      DE, $0000       ; D=drive count, E=drive mask
        LD      IX, fptbl0      ; Start with drive 0
        CALL    fphome          ; Try to home drive 0
        JR      NZ, fpichk1     ; If error, skip drive 0
        SET     0, E            ; Set bit 0 for drive 0
        INC     D               ; Increment drive count

fpichk1 LD      IX, fptbl1      ; Try drive 1
        CALL    fphome
        JR      NZ, fpiex       ; If error, done
        SET     1, E            ; Set bit 1 for drive 1
        INC     D               ; Increment drive count

fpiex   LD      B, D            ; Drive count in B
        POP     HL              ; Restore registers
        POP     DE
        POP     BC
        XOR     A               ; A=0 for success
        RET

; -----------------------------------------------------------------------------
; fpset: Handle configuration commands
; -----------------------------------------------------------------------------
fpset   ; Set IX based on unit
        LD      IX, fptbl0      ; Default to drive 0
        BIT     0, C            ; Check unit bit
        JR      Z, fpset1       ; If unit 0, use fptbl0
        LD      IX, fptbl1      ; Use fptbl1 for unit 1

fpset1  DEC     A               ; Decrement command
        JR      NZ, fpread      ; If not 1, continue to read

        ; Function 1 - Set parameters
        LD      (IX + FDC_GAP3_FORMAT), H
        LD      (IX + FDC_GAP3_RW), L
        LD      (IX + FDC_SECTOR_LEN), D
        LD      (IX + FDC_NUM_SECTORS), E
        
        LD      A, B            ; Step time
        NEG
        ADD     A, A
        ADD     A, A
        ADD     A, A
        OR      $0F
        LD      (IX + FDC_STEP_TIME), A

fpset2  LD      A, $03          ; SPECIFY command
        CALL    fpcom
        LD      A, (IX + FDC_STEP_TIME)
        CALL    fpout
        LD      A, $0E          ; Head Load Time, DMA disabled
        CALL    fpout
        POP     HL              ; Restore registers
        POP     DE
        POP     BC
        XOR     A               ; Success
        RET

; -----------------------------------------------------------------------------
; fpread: Handle Read Sector (Command 2)
; -----------------------------------------------------------------------------
fpread  PUSH    AF              ; Save command
        LD      A, D            ; Prepare cylinder/side
        ADD     A, A
        OR      B               ; Add side bit
        LD      D, A
        POP     AF

        DEC     A               ; Decrement command
        JR      NZ, fpwrit      ; If not 2, continue to write

        LD      A, $66          ; READ DATA command
        CALL    fpwait

        ; Data transfer loop for read
        CALL    fpget_sector_size ; Get sector size in BC
        LD      BC, PORT_FDC_DATA

fpg1    PUSH    BC
        LD      BC, PORT_FDC_STATUS
        IN      A, (C)          ; Check status
        JP      P, fpg1         ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      NZ, fpg1        ; Wait for DIO=0
        POP     BC
        IN      A, (C)          ; Read data byte
        LD      (HL), A         ; Store in buffer
        INC     HL              ; Next buffer position
        DEC     DE              ; Decrement byte counter
        LD      A, D
        OR      E
        JR      NZ, fpg1        ; Continue if more bytes

        JP      fpstat          ; Check final status

; -----------------------------------------------------------------------------
; fpwrit: Handle Write Sector (Command 3)
; -----------------------------------------------------------------------------
fpwrit  DEC     A               ; Decrement command
        JR      NZ, fpchck      ; If not 3, continue to verify

        LD      A, $C5          ; WRITE DATA command
        CALL    fpwait

        ; Data transfer loop for write
        CALL    fpget_sector_size ; Get sector size in DE
        LD      BC, PORT_FDC_DATA

fpp1    PUSH    BC
        LD      BC, PORT_FDC_STATUS
        IN      A, (C)          ; Check status
        JP      P, fpp1         ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      Z, fpp1         ; Wait for DIO=1
        POP     BC
        LD      A, (HL)         ; Get data from buffer
        OUT     (C), A          ; Write to FDC
        INC     HL              ; Next buffer position
        DEC     DE              ; Decrement byte counter
        LD      A, D
        OR      E
        JR      NZ, fpp1        ; Continue if more bytes

        JP      fpstat          ; Check final status

; -----------------------------------------------------------------------------
; fpchck: Handle Verify Sector (Command 4)
; -----------------------------------------------------------------------------
fpchck  DEC     A               ; Decrement command
        JR      NZ, fpform      ; If not 4, continue to format

        LD      A, $66          ; READ DATA for verification
        CALL    fpwait

        ; Verification loop
        CALL    fpget_sector_size ; Get sector size in DE
        LD      BC, PORT_FDC_DATA

fpv1    PUSH    BC
        LD      BC, PORT_FDC_STATUS
        IN      A, (C)          ; Check status
        JP      P, fpv1         ; Wait for RQM
        BIT     6, A            ; Check DIO
        JR      NZ, fpv1        ; Wait for DIO=0
        POP     BC
        IN      A, (C)          ; Read data byte
        CP      (HL)            ; Compare with buffer
        JP      NZ, fpv_error   ; Jump if mismatch
        INC     HL              ; Next buffer position
        DEC     DE              ; Decrement byte counter
        LD      A, D
        OR      E
        JR      NZ, fpv1        ; Continue if more bytes

        JP      fpstat          ; Check final status

fpv_error:
        ; Drain remaining data on verify error
fpv2    PUSH    BC
        LD      BC, PORT_FDC_STATUS
        IN      A, (C)
        JP      P, fpv_done     ; No more data
        BIT     6, A
        JR      NZ, fpv_done    ; DIO changed
        POP     BC
        IN      A, (C)          ; Read and discard
        DEC     DE
        LD      A, D
        OR      E
        JR      NZ, fpv2
fpv_done:
        POP     BC
        CALL    fpstat
        OR      $3F             ; Verification error
        POP     HL              ; Restore registers
        POP     DE
        POP     BC
        RET

; -----------------------------------------------------------------------------
; fpform: Handle Format Track (Command 5)
; -----------------------------------------------------------------------------
fpform  DEC     A
        JR      NZ, fpnic       ; Unimplemented function

        IF      dskhnd & 20     ; Check if formatting disabled
        OR      $FF
        POP     HL
        POP     DE
        POP     BC
        RET
        ENDIF

        LD      A, $4D          ; FORMAT TRACK command
        CALL    fpcom

        LD      A, $40          ; MFM mode
        CALL    fpout

        LD      A, (IX + FDC_SECTOR_LEN)
        CALL    fpout

        LD      A, (IX + FDC_NUM_SECTORS)
        CALL    fpout

        LD      A, (IX + FDC_GAP3_FORMAT)
        CALL    fpout

        LD      A, FILL_BYTE
        CALL    fpout

        ; Send ID field data for each sector
        LD      A, (IX + FDC_NUM_SECTORS)
        LD      B, A            ; Sector count
        LD      E, 1            ; Starting sector

fpf1    LD      A, D            ; Cylinder
        SRL     A
        CALL    fpout           ; Send C

        LD      A, D            ; Head from cylinder LSB
        AND     $01
        CALL    fpout           ; Send H

        LD      A, E            ; Sector number
        CALL    fpout           ; Send R

        LD      A, (IX + FDC_SECTOR_LEN)
        CALL    fpout           ; Send N

        INC     E               ; Next sector
        DJNZ    fpf1            ; Loop for all sectors

        JP      fpstat

fpnic   OR      $FF             ; Unimplemented function
        POP     HL              ; Restore registers
        POP     DE
        POP     BC
        RET

; -----------------------------------------------------------------------------
; fpget_sector_size: Calculate sector size from length code
; Input: (IX+FDC_SECTOR_LEN) = sector length code
; Output: DE = sector size in bytes
; -----------------------------------------------------------------------------
fpget_sector_size:
        LD      A, (IX + FDC_SECTOR_LEN)
        LD      DE, 128         ; Base size (code 0)
        AND     A
        RET     Z               ; Return 128 for code 0
        
        LD      B, A            ; Code in B
fpgs1   SLA     E               ; Multiply DE by 2
        RL      D
        DJNZ    fpgs1           ; Repeat for each code level
        RET

fpnext  EQU     $               ; End of module
