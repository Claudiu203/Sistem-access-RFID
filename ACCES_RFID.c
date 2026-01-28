#include "MKL25Z4.h"
#include <stdint.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* ================= SYSTICK ================= */
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) { msTicks++; }

static void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {}
}

/* ================= RC522 PINS ================= */
#define RC522_CS_PIN     4   // PTC4
#define RC522_RST_PIN    0   // PTC0

/* ================= LED (active LOW) ================= */
#define LED_RED_PIN     18   // PTB18
#define LED_GREEN_PIN   19   // PTB19

/* ================= RC522 REGISTERS ================= */
#define CommandReg      0x01
#define ComIrqReg       0x04
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxASKReg        0x15
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D

/* ================= RC522 COMMANDS ================= */
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_RESETPHASE  0x0F
#define PICC_REQIDL     0x26

/* ================= KEYPAD 4x4 =================
   ROWS: PTD5, PTD0, PTD2, PTD3
   COLS: PTD7, PTA13, PTE0, PTE1
*/
#define KP_ROW0 5   // PTD5
#define KP_ROW1 0   // PTD0
#define KP_ROW2 2   // PTD2
#define KP_ROW3 3   // PTD3

#define KP_COL0 7   // PTD7  (evita PTD1 => LED albastru)
#define KP_COL1 13  // PTA13
#define KP_COL2 0   // PTE0
#define KP_COL3 1   // PTE1

static const char keymap[4][4] = {
    {'1','2','3','F'},
    {'4','5','6','E'},
    {'7','8','9','D'},
    {'A','0','B','C'}   // C = Enter
};

#define ENTER_KEY 'C'
#define RESET_KEY 'A'
static const char PIN_CODE[5] = "1346";

/* ================= OLED SSD1306 (I2C1) ================= */
#define OLED_I2C_ADDR   0x3C   // schimba in 0x3D daca nu merge
#define SSD1306_PAGES   4      // 32px => 4 pagini

/* ================= BUTTON ================= */
#define BUTTON_PIN  3   // PTC3

/* ================= ELECTROMAGNET ================= */
#define MAGNET_PIN   2

/* ================= LED timer (verde 3 sec) ================= */
volatile uint32_t led_green_until = 0;

static void LED_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[LED_RED_PIN]   = PORT_PCR_MUX(1);
    PORTB->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    PTB->PDDR |= (1u << LED_RED_PIN) | (1u << LED_GREEN_PIN);
    PTB->PSOR |= (1u << LED_RED_PIN) | (1u << LED_GREEN_PIN); // off
}

static void LED_Red(void) {
    PTB->PCOR = (1u << LED_RED_PIN);
    PTB->PSOR = (1u << LED_GREEN_PIN);
}

static void LED_Green(void) {
    PTB->PCOR = (1u << LED_GREEN_PIN);
    PTB->PSOR = (1u << LED_RED_PIN);
}

static inline void LED_Green_3s(void) {
    LED_Green();
    led_green_until = msTicks + 3000;
}

static inline void LED_Update(void) {
    if ((int32_t)(msTicks - led_green_until) >= 0) LED_Red();
}

/* ================= ELECTROMAGNET ================= */
static void Magnet_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[MAGNET_PIN] = PORT_PCR_MUX(1);  // GPIO
    PTC->PDDR |= (1u << MAGNET_PIN);           // output

    // implicit OFF
    PTC->PCOR = (1u << MAGNET_PIN);
}

static inline void Magnet_On(void)
{
	PTC->PCOR = (1u << MAGNET_PIN);
}

static inline void Magnet_Off(void)
{
    PTC->PSOR = (1u << MAGNET_PIN);
}
static inline void Magnet_Update(void)
{
    // dacă încă suntem în fereastra de verde => ON
    if ((int32_t)(msTicks - led_green_until) < 0) {
        Magnet_On();
    } else {
        Magnet_Off();
    }
}


/* ================================================= */
/* ================= SPI0 (RC522) ================== */
/* ================================================= */
static void SPI0_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;

    PORTC->PCR[5] = PORT_PCR_MUX(2); // SCK
    PORTC->PCR[6] = PORT_PCR_MUX(2); // MOSI
    PORTC->PCR[7] = PORT_PCR_MUX(2); // MISO

    SPI0->C1 = SPI_C1_MSTR_MASK;
    SPI0->C2 = 0;
    SPI0->BR = SPI_BR_SPPR(3) | SPI_BR_SPR(8); // ~500kHz
    SPI0->C1 |= SPI_C1_SPE_MASK;
}

static uint8_t SPI0_Transfer(uint8_t data) {
    while (!(SPI0->S & SPI_S_SPTEF_MASK)) {}
    SPI0->D = data;
    while (!(SPI0->S & SPI_S_SPRF_MASK)) {}
    return SPI0->D;
}

/* ================= RC522 GPIO ================= */
static inline void RC522_CS_LOW(void)  { PTC->PCOR = (1u << RC522_CS_PIN); }
static inline void RC522_CS_HIGH(void) { PTC->PSOR = (1u << RC522_CS_PIN); }

static void RC522_GPIO_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[RC522_CS_PIN] = PORT_PCR_MUX(1);
    PTC->PDDR |= (1u << RC522_CS_PIN);
    RC522_CS_HIGH();

    PORTC->PCR[RC522_RST_PIN] = PORT_PCR_MUX(1);
    PTC->PDDR |= (1u << RC522_RST_PIN);
    PTC->PSOR |= (1u << RC522_RST_PIN);
}

static void RC522_Reset(void) {
    PTC->PCOR |= (1u << RC522_RST_PIN);
    for (volatile int i = 0; i < 10000; i++) {}
    PTC->PSOR |= (1u << RC522_RST_PIN);
}

/* ================= RC522 REG I/O ================= */
static void RC522_WriteReg(uint8_t reg, uint8_t value) {
    RC522_CS_LOW();
    SPI0_Transfer((reg << 1) & 0x7E);
    SPI0_Transfer(value);
    RC522_CS_HIGH();
}

static uint8_t RC522_ReadReg(uint8_t reg) {
    uint8_t val;
    RC522_CS_LOW();
    SPI0_Transfer(((reg << 1) & 0x7E) | 0x80);
    val = SPI0_Transfer(0x00);
    RC522_CS_HIGH();
    return val;
}

/* ================= RC522 INIT ================= */
static void RC522_Init(void) {
    RC522_Reset();
    RC522_WriteReg(CommandReg, PCD_RESETPHASE);
    for (volatile int i = 0; i < 100000; i++) {}

    RC522_WriteReg(TModeReg, 0x8D);
    RC522_WriteReg(TPrescalerReg, 0x3E);
    RC522_WriteReg(TReloadRegL, 30);
    RC522_WriteReg(TReloadRegH, 0);

    RC522_WriteReg(TxASKReg, 0x40);
    RC522_WriteReg(ModeReg, 0x3D);

    RC522_WriteReg(TxControlReg, RC522_ReadReg(TxControlReg) | 0x03);
}

/* ================= CARD REQUEST ================= */
static uint8_t RC522_Request(uint8_t *atqa) {
    uint8_t irq;
    uint8_t fifo;

    RC522_WriteReg(BitFramingReg, 0x07);
    RC522_WriteReg(CommandReg, PCD_IDLE);
    RC522_WriteReg(FIFOLevelReg, 0x80);

    RC522_WriteReg(FIFODataReg, PICC_REQIDL);
    RC522_WriteReg(CommandReg, PCD_TRANSCEIVE);
    RC522_WriteReg(BitFramingReg, 0x87);

    for (int i = 0; i < 3000; i++) {
        irq = RC522_ReadReg(ComIrqReg);
        if (irq & 0x20) break;
        if (irq & 0x01) return 0;
    }

    if (RC522_ReadReg(ErrorReg) & 0x1B) return 0;

    fifo = RC522_ReadReg(FIFOLevelReg);
    if (fifo != 2) return 0;

    atqa[0] = RC522_ReadReg(FIFODataReg);
    atqa[1] = RC522_ReadReg(FIFODataReg);
    return 1;
}

/* ================================================= */
/* ================= KEYPAD ========================= */
/* ================================================= */
static inline void rows_all_high(void) {
    PTD->PSOR = (1u << KP_ROW0) | (1u << KP_ROW1) | (1u << KP_ROW2) | (1u << KP_ROW3);
}
static inline void row_low(uint32_t rpin) { PTD->PCOR = (1u << rpin); }

static inline uint32_t col_read_by_index(int col) {
    switch (col) {
        case 0: return (PTD->PDIR & (1u << KP_COL0)) ? 1u : 0u;
        case 1: return (PTA->PDIR & (1u << KP_COL1)) ? 1u : 0u;
        case 2: return (PTE->PDIR & (1u << KP_COL2)) ? 1u : 0u;
        case 3: return (PTE->PDIR & (1u << KP_COL3)) ? 1u : 0u;
        default: return 1u;
    }
}

static void Keypad_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK;

    PORTD->PCR[KP_ROW0] = PORT_PCR_MUX(1);
    PORTD->PCR[KP_ROW1] = PORT_PCR_MUX(1);
    PORTD->PCR[KP_ROW2] = PORT_PCR_MUX(1);
    PORTD->PCR[KP_ROW3] = PORT_PCR_MUX(1);

    PORTD->PCR[KP_COL0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[KP_COL1] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[KP_COL2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTE->PCR[KP_COL3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PTD->PDDR |= (1u << KP_ROW0) | (1u << KP_ROW1) | (1u << KP_ROW2) | (1u << KP_ROW3);
    PTD->PDDR &= ~(1u << KP_COL0);
    PTA->PDDR &= ~(1u << KP_COL1);
    PTE->PDDR &= ~((1u << KP_COL2) | (1u << KP_COL3));

    rows_all_high();
}



static char Keypad_GetKey(void) {
    const uint32_t rows[4] = { KP_ROW0, KP_ROW1, KP_ROW2, KP_ROW3 };

    for (int r = 0; r < 4; r++) {
        rows_all_high();
        row_low(rows[r]);

        for (volatile int k = 0; k < 200; k++) {}

        for (int c = 0; c < 4; c++) {
            if (col_read_by_index(c) == 0u) {
                delay_ms(15);
                if (col_read_by_index(c) == 0u) {
                    char ch = keymap[r][c];
                    while (col_read_by_index(c) == 0u) {}
                    delay_ms(10);
                    return ch;
                }
            }
        }
    }
    return 0;
}



//============= BUTTON ======================
static void Button_Init(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[BUTTON_PIN] =
        PORT_PCR_MUX(1) |   // GPIO
        PORT_PCR_PE_MASK |  // pull enable
        PORT_PCR_PS_MASK;   // pull-up

    PTC->PDDR &= ~(1u << BUTTON_PIN); // input
}


static inline int Button_Pressed(void)
{
    return ((PTC->PDIR & (1u << BUTTON_PIN)) == 0); // LOW = apăsat
}


/* ================================================= */
/* ================= I2C1 (OLED) ==================== */
/* ================================================= */
static void I2C1_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;

    // PTC10 = I2C1_SCL (ALT2), PTC11 = I2C1_SDA (ALT2)
    // FARA ODE mask (ca sa nu ai eroarea de macro lipsa)
    PORTC->PCR[10] = PORT_PCR_MUX(2) | (1u << 5); // ODE bit = 5
    PORTC->PCR[11] = PORT_PCR_MUX(2) | (1u << 5);

    I2C1->F  = I2C_F_MULT(0) | I2C_F_ICR(0x14); // ~100kHz
    I2C1->C1 = I2C_C1_IICEN_MASK;
}

static void I2C1_WaitBusFree(void)
{
    uint32_t t0 = msTicks;
    while (I2C1->S & I2C_S_BUSY_MASK) {
        if ((msTicks - t0) > 20) break; // nu bloca infinit
    }
}

static void I2C1_Start(void)
{
    I2C1_WaitBusFree();
    I2C1->C1 |= I2C_C1_TX_MASK;  // TX mode
    I2C1->C1 |= I2C_C1_MST_MASK; // master mode
}

static void I2C1_Stop(void)
{
    I2C1->C1 &= ~I2C_C1_MST_MASK; // generate STOP
    I2C1->C1 &= ~I2C_C1_TX_MASK;  // back to RX (ok)
    // mic delay ca sa lase bus-ul sa se stabilizeze
    for (volatile int i=0; i<200; i++) {}
}

static int I2C1_WriteByte(uint8_t b)
{
    // clear IICIF BEFORE sending
    I2C1->S |= I2C_S_IICIF_MASK;

    I2C1->D = b;

    uint32_t t0 = msTicks;
    while (!(I2C1->S & I2C_S_IICIF_MASK)) {
        if ((msTicks - t0) > 10) return 0; // timeout
    }

    // clear flag
    I2C1->S |= I2C_S_IICIF_MASK;

    // NACK?
    if (I2C1->S & I2C_S_RXAK_MASK) return 0;

    // arbitration lost?
    if (I2C1->S & I2C_S_ARBL_MASK) {
        I2C1->S |= I2C_S_ARBL_MASK;
        return 0;
    }

    return 1;
}

static int I2C1_WriteBytes(uint8_t addr7, const uint8_t *data, uint32_t len)
{
    I2C1_Start();

    if (!I2C1_WriteByte((addr7 << 1) | 0)) { I2C1_Stop(); return 0; }

    for (uint32_t i = 0; i < len; i++) {
        if (!I2C1_WriteByte(data[i])) { I2C1_Stop(); return 0; }
    }

    I2C1_Stop();
    return 1;
}

static void I2C_Scan(void)
{
    PRINTF("I2C scan...\r\n");
    for (uint8_t a = 1; a < 0x7F; a++) {
        I2C1_Start();
        if (I2C1_WriteByte((a << 1) | 0)) {
            PRINTF("ACK at 0x%02X\r\n", a);
        }
        I2C1_Stop();
        delay_ms(2);
    }
    PRINTF("Scan done.\r\n");
}

/* ================= SSD1306 ================= */
static void SSD1306_Cmd(uint8_t cmd)
{
    uint8_t pkt[2] = {0x00, cmd};
    if (!I2C1_WriteBytes(OLED_I2C_ADDR, pkt, 2)) {
        PRINTF("OLED I2C FAIL (addr=0x%02X) cmd=0x%02X\r\n", OLED_I2C_ADDR, cmd);
        // ca să nu se spam-eze infinit:
        delay_ms(300);
    }
}

static void SSD1306_Data(const uint8_t *data, uint32_t len) {
    uint8_t buf[17];
    buf[0] = 0x40;

    while (len) {
        uint32_t chunk = (len > 16) ? 16 : len;
        for (uint32_t i = 0; i < chunk; i++) buf[1+i] = data[i];
        I2C1_WriteBytes(OLED_I2C_ADDR, buf, chunk + 1);
        data += chunk;
        len  -= chunk;
    }
}

static void SSD1306_SetCursor(uint8_t x, uint8_t page) {
    SSD1306_Cmd(0xB0 | (page & 0x03));
    SSD1306_Cmd(0x00 | (x & 0x0F));
    SSD1306_Cmd(0x10 | ((x >> 4) & 0x0F));
}

static void SSD1306_Clear(void) {
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        SSD1306_SetCursor(0, page);
        for (uint8_t col = 0; col < 128; col++) {
            uint8_t z = 0x00;
            SSD1306_Data(&z, 1);
        }
    }
    SSD1306_SetCursor(0, 0);
}

static void SSD1306_Init(void) {
    delay_ms(50);

    SSD1306_Cmd(0xAE);
    SSD1306_Cmd(0xD5); SSD1306_Cmd(0x80);
    SSD1306_Cmd(0xA8); SSD1306_Cmd(0x1F); // 32
    SSD1306_Cmd(0xD3); SSD1306_Cmd(0x00);
    SSD1306_Cmd(0x40);
    SSD1306_Cmd(0x8D); SSD1306_Cmd(0x14);
    SSD1306_Cmd(0x20); SSD1306_Cmd(0x00);
    SSD1306_Cmd(0xA1);
    SSD1306_Cmd(0xC8);
    SSD1306_Cmd(0xDA); SSD1306_Cmd(0x02);
    SSD1306_Cmd(0x81); SSD1306_Cmd(0x7F);
    SSD1306_Cmd(0xD9); SSD1306_Cmd(0xF1);
    SSD1306_Cmd(0xDB); SSD1306_Cmd(0x40);
    SSD1306_Cmd(0xA4);
    SSD1306_Cmd(0xA6);
    SSD1306_Cmd(0xAF);

    SSD1306_Clear();
    SSD1306_SetCursor(0,0);
}

/* ===== font minimal pentru '*', cifre, A..F, spatiu, ':' ===== */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F

};
static const uint8_t GLYPH_P[5] = {0x7F,0x09,0x09,0x09,0x06}; // P
static const uint8_t GLYPH_I[5] = {0x00,0x41,0x7F,0x41,0x00}; // I
static const uint8_t GLYPH_N[5] = {0x7F,0x02,0x04,0x08,0x7F}; // N
static const uint8_t GLYPH_O[5] = {0x3E,0x41,0x41,0x41,0x3E}; // O
static const uint8_t GLYPH_K[5] = {0x7F,0x08,0x14,0x22,0x41}; // K
static const uint8_t GLYPH_R[5] = {0x7F,0x09,0x19,0x29,0x46}; // R
static const uint8_t GLYPH_W[5] = {0x7F,0x20,0x18,0x20,0x7F}; // W
static const uint8_t GLYPH_G[5] = {0x3E,0x41,0x49,0x49,0x3A}; // G
static const uint8_t GLYPH_Y[5] = {0x07,0x08,0x70,0x08,0x07}; // Y

static const uint8_t* glyph(char c)
{
    if (c == ' ') return font5x7[0];
    if (c == ':') return font5x7[1];
    if (c == '*') return font5x7[2];

    if (c >= '0' && c <= '9') return font5x7[3 + (c - '0')];
    if (c >= 'A' && c <= 'F') return font5x7[13 + (c - 'A')];

    // litere extra pentru mesaje
    if (c == 'P') return GLYPH_P;
    if (c == 'I') return GLYPH_I;
    if (c == 'N') return GLYPH_N;
    if (c == 'O') return GLYPH_O;
    if (c == 'K') return GLYPH_K;
    if (c == 'R') return GLYPH_R;
    if (c == 'W') return GLYPH_W;
    if (c == 'G') return GLYPH_G;
    if (c == 'Y') return GLYPH_Y;

    return font5x7[0];
}

static void SSD1306_WriteChar(char c) {
    uint8_t out[6];
    const uint8_t *g = glyph(c);
    out[0]=g[0]; out[1]=g[1]; out[2]=g[2]; out[3]=g[3]; out[4]=g[4];
    out[5]=0x00;
    SSD1306_Data(out, 6);
}

static void SSD1306_WriteChar2x(char c) {
    const uint8_t *g = glyph(c);

    for (int col = 0; col < 5; col++) {
        uint8_t line = g[col];

        uint8_t upper = 0;
        uint8_t lower = 0;

        for (int bit = 0; bit < 7; bit++) {
            if (line & (1 << bit)) {
                upper |= (3 << (bit * 2));
                lower |= (3 << (bit * 2));
            }
        }

        uint8_t buf[3];
        buf[0] = upper;
        buf[1] = upper;
        buf[2] = 0x00;
        SSD1306_Data(buf, 3);

        buf[0] = lower;
        buf[1] = lower;
        buf[2] = 0x00;
        SSD1306_Data(buf, 3);
    }
}

static void SSD1306_WriteString(const char *s) {
    while (*s) SSD1306_WriteChar(*s++);
}

static void SSD1306_WriteString2x(const char *s) {
    while (*s) {
        SSD1306_WriteChar2x(*s++);
    }
}

/* ================= OLED helper ================= */

volatile uint32_t oled_msg_until = 0;
volatile uint8_t  oled_showing_msg = 0;
static void OLED_ShowPIN(uint8_t count) {
    SSD1306_Clear();
    SSD1306_SetCursor(0,0);
    SSD1306_WriteString("PIN: ");
    for (uint8_t i = 0; i < count; i++) SSD1306_WriteChar('*');
}
static void OLED_ClearLine(uint8_t page)
{
    SSD1306_SetCursor(0, page);
    uint8_t zeros[16];
    for (int i = 0; i < 16; i++) zeros[i] = 0x00;

    // 128 coloane => 8 blocuri a câte 16 bytes
    for (int blk = 0; blk < 8; blk++) {
        SSD1306_Data(zeros, 16);
    }
}

static void OLED_ShowPIN_Buffer(const char *buf)
{
    OLED_ClearLine(0);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("PIN: ");
    SSD1306_WriteString(buf);
}

static void OLED_ShowMessage(const char *msg) {
    SSD1306_Clear();
    SSD1306_SetCursor(0,1);   // rândul 2 (mai centrat)
    SSD1306_WriteString(msg);
}
static void OLED_ShowMessageTimed(const char *msg, uint32_t ms)
{
    // scrie mesaj pe linia de jos (page 2), fara sa stergi PIN-ul
    OLED_ClearLine(2);
    SSD1306_SetCursor(0, 2);
    SSD1306_WriteString(msg);

    oled_showing_msg = 1;
    oled_msg_until = msTicks + ms;
}
static void OLED_Update(void)
{
    if (!oled_showing_msg) return;

    if ((int32_t)(msTicks - oled_msg_until) >= 0) {
        oled_showing_msg = 0;
        OLED_ClearLine(2);     // sterge mesajul de pe linia de jos
    }
}


static void OLED_WriteLine(uint8_t page, const char *text)
{
    OLED_ClearLine(page);
    SSD1306_SetCursor(0, page);
    SSD1306_WriteString(text);
}

/* ================================================= */
/* ================= PIN LOGIC ====================== */
/* ================================================= */
static inline int is_digit(char ch) { return (ch >= '0' && ch <= '9'); }

static void PIN_ProcessKey(char ch, char *buf, uint8_t *idx)
{
    if (ch == 0) return;

    if (is_digit(ch)) {
        if (*idx < 4) {
            buf[*idx] = ch;
            (*idx)++;
            buf[*idx] = '\0';

            OLED_ShowPIN_Buffer(buf);     // linia 0
            OLED_ClearLine(2);            // șterge mesajul vechi
            PRINTF("key=%c idx=%u buf=%s\r\n", ch, (unsigned)*idx, buf);
        }
        return;
    }

    if (ch == RESET_KEY) {
        *idx = 0;
        for (int i = 0; i < 5; i++) buf[i] = 0;

        OLED_ShowPIN_Buffer("");         // golește PIN pe ecran
        OLED_ClearLine(2);               // golește mesaj
        PRINTF("RESET\r\n");
        return;
    }

    if (ch == ENTER_KEY) {
        int ok = 0;

        if (*idx == 4) {
            ok = 1;
            for (int i = 0; i < 4; i++) {
                if (buf[i] != PIN_CODE[i]) { ok = 0; break; }
            }
        }

        if (ok) {
        	OLED_ShowMessageTimed("PIN OK", 1500);
            LED_Green_3s();
            PRINTF("PIN OK\r\n");
        } else {
        	OLED_ShowMessageTimed("PIN WRONG", 1500);
            PRINTF("PIN WRONG\r\n");
        }

        // aici chiar "ștergem pin-ul" după enter
        *idx = 0;
        for (int i = 0; i < 5; i++) buf[i] = 0;
        OLED_ShowPIN_Buffer("");         // golește pinul pe ecran

        return;
    }
}

/* ================================================= */
/* ================= MAIN ========================== */
/* ================================================= */
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    SysTick_Config(SystemCoreClock / 1000);

    SPI0_Init();
    RC522_GPIO_Init();
    RC522_Init();
    LED_Init();
    Keypad_Init();
    Button_Init();
    Magnet_Init();

    LED_Init();
    LED_Red();
    delay_ms(200);
    LED_Green();
    delay_ms(200);
    LED_Red();
    I2C1_Init();
    I2C_Scan();
    LED_Green();
    delay_ms(200);
    LED_Red();
    SSD1306_Init();
    OLED_ShowPIN_Buffer("1234");
    OLED_WriteLine(2, "PIN OK");
    LED_Green();
    delay_ms(200);
    LED_Red();
    OLED_ShowPIN(0);

    PRINTF("Ready\r\n");

    uint8_t atqa[2];
    char pinbuf[5] = {0};   // IMPORTANT: 4 cifre + '\0'
    uint8_t pinidx = 0;

    LED_Red();

    while (1) {
        char k = Keypad_GetKey();
        PIN_ProcessKey(k, pinbuf, &pinidx);

        if (RC522_Request(atqa)) {
            LED_Green_3s(); // card -> verde 3 sec
        }
        LED_Update();
        Magnet_Update();
        OLED_Update();

        if (Button_Pressed()) {
            LED_Green_3s();
            while (Button_Pressed()) {} // așteaptă eliberarea (debounce simplu)
        }

    }
}
