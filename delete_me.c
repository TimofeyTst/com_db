#include <avr/interrupt.h>
#include <avr/io.h>  // Библиотека для работы с портами ввода/вывода
#include <avr/sleep.h>
#include <inttypes.h>
#include <util/delay.h>

// USART
#define BAUD 9600
#include <util/setbaud.h>

#define NUM_SENSORS 4
#define DELAY_MS 500             // можно менять
#define MAX_SIRENA_REPEAT_CNT 3  // можно менять
#define CONTROL_PIN PD3
#define UNDER_SECURE_LED_PIN PB5
#define MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN PB6
#define SIREN_PIN PB7

uint8_t inputPins[NUM_SENSORS] = {PA0, PA1, PA2, PA3};
uint8_t outputPins[NUM_SENSORS] = {PB0, PB1, PB2, PB3};
uint8_t prevStates[NUM_SENSORS] = {0, 0, 0, 0};
uint8_t is_under_secure = 0;
uint8_t move_was_detected = 0;
int sirena_repeat_cnt = MAX_SIRENA_REPEAT_CNT;


void USART_init(void) {
    /* Set baud rate */
    UBRRH = UBRR_VALUE >> 8;
    UBRRL = UBRR_VALUE;
    /* Enable receiver and transmitter */
    UCSRB = (1 << RXEN) | (1 << TXEN);
    /* Set frame format: 8data, 1 stop bit */
    UCSRC = (1 << URSEL) | (1 << USBS) | (3 << UCSZ0);
}


void USART_Transmit(unsigned char data) {
    /* Wait for empty transmit buffer */
    while (!(UCSRA & (1 << UDRE)));
    /* Put data into buffer, sends the data */
    UDR = data;
}

// Передача строки через USART
void usart_print(const char* str) {
    while (*str) {
        USART_Transmit(*str);
        str++;
    }
}

void usart_print_secure_flag(uint8_t is_under_secure, const char* str) {
    if (is_under_secure) {
        usart_print("[UNDER SECURE] ");
    };
    usart_print(str);
}

uint8_t sensor_detect_move(uint8_t index) {
    return PINA & (1 << inputPins[index]);
}

// Проверка состояния кнопки
uint8_t button_is_pressed() {
    return !(PIND & (1 << CONTROL_PIN));  // Кнопка нажата, если значение на пине 0
}

void init_interrupt_button(void) {
    DDRD &= ~(1 << CONTROL_PIN);
    PORTD |= (1 << CONTROL_PIN);

    GICR |= (1 << INT1);    // разрешить прерывание INT1
    MCUCR |= (1 << ISC10);  // прерывание INT1 при любом изменении состояния пина

    sei();  // разрешение прерываний глобально
}

void clear_prev_states(void) {
    uint8_t i;
    for (i = 0; i < NUM_SENSORS; i++) {
        prevStates[i] = 0;
    }
}

ISR(INT1_vect) {
    usart_print("\r\n[INTERRUPTED]\r\n");
    // Здесь код обработчика прерывания по изменению состояния кнопки
    if (button_is_pressed()) {
        is_under_secure = !is_under_secure;
        move_was_detected = 0;
        PORTB &= ~(1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN);  // Выключаем мигалку
        DDRB &= ~(1 << SIREN_PIN);
        PORTB &= ~(1 << SIREN_PIN);  // Выключаем сирену
        sirena_repeat_cnt = MAX_SIRENA_REPEAT_CNT;  // сирену в макс число
        clear_prev_states();
        if (is_under_secure) {
            PORTB |= (1 << UNDER_SECURE_LED_PIN);
            usart_print("[WARNING]: SECURE MODE ENABLED\r\n");
        } else {
            PORTB &= ~(1 << UNDER_SECURE_LED_PIN);
            usart_print("[WARNING]: SECURE MODE DISABLED\r\n");
        }
        _delay_ms(500);  // Антидребезговая задержка
    }
}

int main(void) {
    // Инициализация ввода на порту A (датчик движения)
    DDRA = 0x00;  // Настроить все биты порта A как входы (ввод)
    PORTA = 0x00;  // Включить подтягивающие резисторы на порту C

    // Инициализация вывода на порту B (светодиоды)
    DDRB = 0xFF;  // Настроить все биты порта B как выходы (вывод)
    PORTB = 0x00;  // Установить начальное состояние порта B (все LED выключены)

    DDRB |= (1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN);
    DDRB &= ~(1 << SIREN_PIN);  // выкл сирены

    init_interrupt_button();
    clear_prev_states();

    USART_init();
    _delay_ms(DELAY_MS);

    char buffer[50];
    while (1) {
        uint8_t i;
            for (i = 0; i < NUM_SENSORS; i++) {
            // Проверка состояния датчика движения
            uint8_t is_sensor_detects_move = sensor_detect_move(i);

            if (is_sensor_detects_move) {
                // Если с датчика приходит 1 (обнаружено движение), включаем
                // соответствующий светодиод
                PORTB |= (1 << outputPins[i]);

                // Проверка нужно ли включить сирену
                if (prevStates[i]) {
                    // Если в прошлый раз движение тоже было
                    if (is_under_secure) {
                        sirena_repeat_cnt = 0;
                        move_was_detected = 1;  // Включаем мигалку
                        PORTB |= (1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN);  // Включаем мигалку
                        DDRB |= (1 << SIREN_PIN);
                        PORTB |= (1 << SIREN_PIN);  // Включаем сирену
                        sprintf(buffer, "[ERROR]: Siren! MSensor [%u] detects move >= 2 times!\r\n", i);
                    } else {
                        sprintf(buffer, "[INFO]: MSensor [%u] detects move >= 2tms, not secure\r\n", i);
                    }
                } else {
                    sprintf(buffer, "[INFO]: MSensor [%u] detects move 1 time, check repeat\r\n", i);
                }
            } else {
                // Если с датчика приходит 0 (движение не обнаружено), выключаем
                // светодиод
                PORTB &= ~(1 << outputPins[i]);
                if (prevStates[i]) {
                    sprintf(buffer, "[INFO]: MSensor [%u] not repeat last detected move, skip\r\n", i);
                } else {
                    sprintf(buffer, "[INFO]: MSensor [%u] empty\r\n", i);
                }
            }

            // Запоминаем прошлое состояние
            prevStates[i] = is_sensor_detects_move;

            // Вывод
            usart_print_secure_flag(is_under_secure, buffer);
        }

        if (sirena_repeat_cnt >= MAX_SIRENA_REPEAT_CNT) {
            // Выключить сирену
            DDRB &= ~(1 << SIREN_PIN);
            PORTB &= ~(1 << SIREN_PIN);
        } else {
            ++sirena_repeat_cnt;
        }

        _delay_ms(DELAY_MS);  // Блокировать короткую задержку для отклика на изменения
    }

    return 0;  // Эта часть кода никогда не выполнится
}
