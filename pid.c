#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

// USART
#define USART1_BAUD (unsigned int *) (0x0828)
#define USART1_CTRLB (unsigned char *)(0x0826)
#define USART1_STATUS (unsigned char *)(0x0824)
#define USART1_TXDATAL (unsigned char *)(0x0822)
#define USART1_RXDATAL (unsigned char *)(0x0820)
#define USART_DREIF_bm (0x20)
#define USART_RXCIF_bm (0x80)

#define F_CPU 16000000 // 16MHz
#define BAUD_RATE 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

// motor pullup
#define PORTD_PIN6CTRL (*(volatile unsigned char*) (0x0440 + 0x16))
#define PORTD_PIN6CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)

// TCB0 정의
#define TCB0 (*volatile unsigned char*) (0x0A80)

#define TCB0_CTRLA (*(volatile unsigned char*) (0x0A80 + 0x00))
#define TCB_CLKSEL_CLKDIV2_gc ((unsigned char) 0b00000010)
#define TCB_ENABLE_bm (unsigned char) 0b00000001

#define TCB0_CTRLB (*(volatile unsigned char*) (0x0A80 + 0x01))
#define TCB_CCMPEN_bm (unsigned char) 0b00010000
#define TCB_CNTMODE_PWM8_gc (unsigned char) 0b00000111

#define TCB0_CCMPL (*(volatile unsigned char*) (0x0A80 + 0x0C))
#define TCB0_CCMPH (*(volatile unsigned char*) (0x0A80 + 0x0D))

// PORTMUX 정의
#define PORTMUX (*(volatile unsigned char*) 0x05E0)
#define PORTMUX_TCBROUTEA (*(volatile unsigned char*) (0x05E0 + 0x05))
#define PORTMUX_TCB0_ALT1_bm (unsigned char) 0b00000001 // PF5

// TWI 정의
#define PORTMUX_TWISPIROUTEA (*(volatile unsigned char*) (0x05E0 + 0x03))
#define PORTMUX_TWI0_ALT2_bm ((unsigned char) 0b00100000)

#define TWI0 (*(volatile unsigned char*) 0x08A0)
#define TWI0_MCTRLA (*(volatile unsigned char*) (0x08A0 + 0x03))
#define TWI_ENABLE_bm ((unsigned char) 0b00000001)

#define TWI0_MCTRLB (*(volatile unsigned char*) (0x08A0 + 0x04))
#define TWI_ACKACT_NACK_gc ((unsigned char) 0b00000100)
#define TWI_MCMD_RECVTRANS_gc ((unsigned char) 0b00000010)
#define TWI_MCMD_STOP_gc ((unsigned char) 0b00000011)

#define TWI0_MSTATUS (*(volatile unsigned char*) (0x08A0 + 0x05))
#define TWI_RIF_bm ((unsigned char) 0b10000000)
#define TWI_WIF_bm ((unsigned char) 0b01000000)
#define TWI_RXACK_bm ((unsigned char) 0b00010000)
#define TWI_ARBLOST_bm ((unsigned char) 0b00001000)
#define TWI_BUSERR_bm ((unsigned char) 0b00000100)
#define TWI_BUSSTATE_IDLE_gc ((unsigned char) 0b00000001)

#define TWI0_MBAUD (*(volatile unsigned char*) (0x08A0 + 0x06))
#define TWI0_MADDR (*(volatile unsigned char*) (0x08A0 + 0x07))
#define TWI0_MDATA (*(volatile unsigned char*) (0x08A0 + 0x08))

#define TWI_READ true
#define TWI_WRITE false

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

#define PI 3.141592
#define ALPHA 0.75
#define DT 0.2534

void USART1_init(void);
void USART1_Transmit(unsigned char data);
unsigned char USART1_Receive(void);
void USART1_Transmit_String(unsigned char data[]);
void USART1_Transmit_Float(float num);

void twi_init();
bool twi_start(unsigned char device, bool read);
void twi_stop();
bool twi_read(unsigned char* data, bool last);
bool twi_write(unsigned char data);

void mpu6050_init();
void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz);

void tcb_init();
void set_direction(bool direction);
void motor_on();

// PID constants
float Kp = 18; 
float Ki = 0;
float Kd = 60;
float previous_error = 0;
float integral = 0;

short mpu6050_offsets[6];
float angle_gx = 0, angle_gy = 0, angle_x = 0, angle_y = 0, angle_z = 0;
float target_angle = 0; // 목표 각도

void setup() {
    USART1_init();
    mpu6050_init();
    tcb_init();
}

void loop() {
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // Calculate angles from accelerometer data
    float ax = raw_ax - mpu6050_offsets[0];
    float ay = raw_ay - mpu6050_offsets[1];
    float az = raw_az - mpu6050_offsets[2];

    float angle_ax = atan2(ay, sqrt(ax * ax + az * az)) * (180 / PI);
    float angle_ay = atan2(ax, sqrt(ay * ay + az * az)) * (180 / PI);

    float gx = ((float) (raw_gx - mpu6050_offsets[3])) / 131;
    float gy = ((float) (raw_gy - mpu6050_offsets[4])) / 131;
    float gz = ((float) (raw_gz - mpu6050_offsets[5])) / 131;

    // Apply complementary filter
    angle_x = ALPHA * (angle_x + gx * DT) + (1 - ALPHA) * angle_ax;
    angle_y = ALPHA * (angle_y + gy * DT) + (1 - ALPHA) * angle_ay;
    angle_z += gz * DT; // Integrate gyro data for Z angle

    // PID controller
    float error = target_angle - angle_y; // 목표각도와 현재각도의 차이로 오차 계산
    integral += error * DT;
    float derivative = (error - previous_error) / DT;
    float output = Kp * error + Ki * integral + Kd * derivative;

    previous_error = error;

    // 모터 제어
    if (output > 0) {
      set_direction(true); // CCW
      TCB0_CCMPH = (unsigned char) fmin(output, 100); // 출력 제한
    } else {
      set_direction(false); // CW
      TCB0_CCMPH = (unsigned char) fmin(-output, 100); // 출력 제한
    }

    // 각도 출력 (디버깅을 위해)
    USART1_Transmit_String("angle_y: ");
    USART1_Transmit_Float(angle_y);
    USART1_Transmit_String("\n");
}


void USART1_init(void) {
    PORTC_DIR |= 0x1 << 0;
    PORTC_DIR &= ~(0x1 << 1);

    *USART1_BAUD = (uint16_t)USART0_BAUD_RATE(BAUD_RATE);
    *USART1_CTRLB |= (1 << 3) | (1 << 4); // tx_en, rx_en
}

void USART1_Transmit(unsigned char data) {
    while (!((*USART1_STATUS) & USART_DREIF_bm)); // Data Empty
    *USART1_TXDATAL = data;
}

void USART1_Transmit_String(unsigned char data[]) {
    int i = 0;
    while (data[i] != '\0') {
        USART1_Transmit(data[i]);
        i++;
    }
}

void USART1_Transmit_Float(float num) {
    char buf[10]; // 문자열 담을 buf 선언
    dtostrf(num, 6, 2, buf); // float를 문자열로 변환, 너비 6으로 설정(각도 3자리 소수점2자리), 소수점 아래 2자리까지 표현
    USART1_Transmit_String(buf);
}

unsigned char USART1_Receive(void) {
    while (!((*USART1_STATUS) & USART_RXCIF_bm)); // Receive Complete
    return *USART1_RXDATAL;
}

void twi_init() {
    PORTMUX_TWISPIROUTEA = PORTMUX_TWI0_ALT2_bm;
    unsigned int frequency = 400000;
    unsigned short t_rise = 300;
    unsigned int baud = (F_CPU / frequency - F_CPU / 1000 / 1000 * t_rise / 1000 - 10) / 2;
    TWI0_MBAUD = (unsigned char) baud;

    TWI0_MCTRLA = TWI_ENABLE_bm;
    TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

bool twi_start(unsigned char address, bool read) {
    TWI0_MADDR = address << 1 | (read ? 1 : 0);
    if (read)
        while (!(TWI0_MSTATUS & TWI_RIF_bm));
    else
        while (!(TWI0_MSTATUS & TWI_WIF_bm));

    if (TWI0_MSTATUS & TWI_ARBLOST_bm) {
        while (!(TWI0_MSTATUS & TWI_BUSSTATE_IDLE_gc));
        return false;
    }
    if (TWI0_MSTATUS & TWI_RXACK_bm) {
        TWI0_MCTRLB |= TWI_MCMD_STOP_gc;
        while (!(TWI0_MSTATUS & TWI_BUSSTATE_IDLE_gc));
        return false;
    }
    return true;
}

void twi_stop() {
    TWI0_MCTRLB |= TWI_MCMD_STOP_gc;
    while (!(TWI0_MSTATUS & TWI_BUSSTATE_IDLE_gc));
}

bool twi_read(unsigned char* data, bool last) {
    while (!(TWI0_MSTATUS & TWI_RIF_bm));
    *data = TWI0_MDATA;
    if (last) {
        TWI0_MCTRLB = TWI_ACKACT_NACK_gc;
    } else {
        TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
    }
    return true;
}

bool twi_write(unsigned char data) {
    TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
    TWI0_MDATA = data;
    while (!(TWI0_MSTATUS & TWI_WIF_bm));
    if (TWI0_MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
        return false;
    }
    return !(TWI0_MSTATUS & TWI_RXACK_bm);
}

void mpu6050_init() {
    twi_init();

    twi_start(MPU6050, TWI_WRITE);
    twi_write(MPU6050_PWR_MGMT_1);
    twi_write(0);
    twi_stop();

    short sum[6] = { 0 };
    for (int i = 0; i < 10; i += 1) {
        short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        sum[0] += raw_ax;
        sum[1] += raw_ay;
        sum[2] += raw_az;
        sum[3] += raw_gx;
        sum[4] += raw_gy;
        sum[5] += raw_gz;
    }

    for (int i = 0; i < 6; i += 1) {
        mpu6050_offsets[i] = sum[i] / 10;
    }
}

void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz) {
    unsigned char buf[14];
    for (int i = 0; i < 14; i += 1) {
        twi_start(MPU6050, TWI_WRITE);
        twi_write(MPU6050_ACCEL_XOUT_H + i);
        twi_stop();
        twi_start(MPU6050, TWI_READ);
        twi_read(&buf[i], true);
        twi_stop();
    }
    *raw_ax = (buf[0] << 8) | buf[1];
    *raw_ay = (buf[2] << 8) | buf[3];
    *raw_az = (buf[4] << 8) | buf[5];
    *raw_gx = (buf[8] << 8) | buf[9];
    *raw_gy = (buf[10] << 8) | buf[11];
    *raw_gz = (buf[12] << 8) | buf[13];
}

void tcb_init() {
    PORTMUX_TCBROUTEA = PORTMUX_TCB0_ALT1_bm;
    PORTF_DIR = (0x1 << 4);
    PORTD_DIR |= (1 << 5);

    TCB0_CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;
    TCB0_CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
    TCB0_CCMPH = 0x64; // Duty cycle 99%
    TCB0_CCMPL = 0x3E8; // Period
}

void set_direction(bool direction) {
    if (direction) {
        PORTD_OUT |= (0x1 << 5); // CW
    } else {
        PORTD_OUT &= ~(0x1 << 5); // CCW
    }
}
