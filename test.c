/* main.c
 * Designed for an attiny13(a), uses ??? bytes of program space, 
 * fuses set using -U lfuse:w:0x6a:m -U hfuse:w:0xff:m powered from 5V
 * Распиновка
                 +-\/-+
 RES (D5/A0)PB5 1|    |8 VCC
     (D3/A3)PB3 2|    |7 PB2(D2/A1)
     (D4/A2)PB4 3|    |6 PB1(D1/PWM)
            GND 4|    |5 PB0(D0/PWM)
                 +----+
    0 63
   64 127
  128 191
  192 255
  
    0 255
  256 511
  512 767
  768 1023
  
  int8_t   1 байт  0...255
  uint16_t 2 байта 0...65535
  */

#ifndef __AVR_ATtiny13A__
  #define __AVR_ATtiny13A__
#endif
//#define F_CPU 9600000UL  // 9.6 MHz
#define F_CPU 1200000UL  // 1.2 MHz
#define LDR     PB2
#define RED     PB0
#define YELLOW  PB1
#define GREEN   PB4
#define BLUE    PB3

/*#ifndef sbi // Set Bit in I/O Register
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef cbi // Clear Bit in I/O Register
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) 
#endif*/

#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>
//#include <avr/sleep.h>
#include <inttypes.h>


uint8_t transformer(uint16_t x)
{
  /* Автомат для проверки результата преобразования 10битного АЦП разбиение 
    числа 1024 на 4 части и в зависимости от промежутка вывод значения 
    от 1 до 4, что позволяет сделать индикацию выводя результат на один 
    из 4 разноцветных светодиодов. Значения можно поменять, чтобы найти 
    точное значение датчика*/
  if (x >= 0 && x < 256)
  {
    return RED;
  }
  else if (x >= 256 && x < 512)
  {
    return YELLOW;
  }
  else if (x >= 512 && x < 768)
  {
    return GREEN;
  }
  else if (x >= 768 && x < 1024)
  {
    return BLUE;
  }
  else
  {
    return 0;
  }
}



void initPins(void)
{
  /* Инициация входов и выходов пинов, и их значений по умолчанию */
  PORTB = 0x00; // Сброс значений на портах в нули
  // Подключаем светодиоды как выход
  DDRB |= (1 << RED) | (1 << YELLOW) | (1 << GREEN) | (1 << BLUE);
  // Фоторозистор как вход, зачем пока не понятно ибо и так 0 по дефолту
  DDRB &= ~(1 << LDR); // pinMode(PHOTORES, INPUT);
}

void initAdc(void)
{
  /* This function initialises the ADC
    ADC Prescaler Notes:
    --------------------
    ADC Prescaler needs to be set so that the ADC input frequency 
    is between 50 - 200kHz.
    Clock   Available prescaler values
    ----------------------------------
    1.2 MHz   8 (150kHz), 16 (75kHz)
    2-600000|4-300000|8-150000|16-75000|32-37500|64-18750|128-9375 
    
    В качестве аналогового опорного сигнала используется Vcc~=5v */
  
  // Настройка мультиплексора АЦП на канал ADC1 (PB2) 7 пин
  ADMUX |= (1 << MUX0);
  ADCSRA |= (1 << ADEN) // Включение АЦП
          | (1 << ADPS1)|(1 << ADPS0); // Устанавливаем предделитель на cpu/8
  // Запрещаем цифровой вход на ноге аналогового входа
  DIDR0 |= (1 << LDR);
}


int main(void)
{ // -> main
  initPins(); // Инициализация пинов
  initAdc(); // Инициализация АЦП
  /* Вызов с помощью функции asm ассемблерной команды nop с ключевым словом 
    volatile (скрыть от оптимизатора) с целью задержки для синхронизации*/
  asm volatile("nop");
  
  while(1)
  { // -> loop
    ADCSRA |= (1 << ADSC); // ADC start the conversion
    while ((ADCSRA & (1 << ADIF)) == 0); // Ждем флага окончания преобразования
    /* Получаем данные результа АЦП из нижнего регистра, потом склеиваем со 
     старшим начиная с 8 бита и получаем 10битное значение 0-1023 LDR*/
    uint16_t adc = (ADCL | (ADCH << 8));
    
    PORTB = (1 << transformer(adc));
    _delay_ms(100);
    
  } // <- loop
} // <- main