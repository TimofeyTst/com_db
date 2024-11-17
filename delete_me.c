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
#define SECURE_MODE_BTN_PIN PD3
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

    _delay_ms(DELAY_MS); 
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


void init_interrupt_button(void) {
    DDRD &= ~(1 << SECURE_MODE_BTN_PIN);
    PORTD |= (1 << SECURE_MODE_BTN_PIN);

    GICR |= (1 << INT1);    // разрешить прерывание INT1
    MCUCR |= (1 << ISC10);  // прерывание INT1 при любом изменении состояния пина

    sei();  // разрешение прерываний глобально
}


turn_off_under_secure_led(void) {
    move_was_detected = 0;
    PORTB &= ~(1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN);  // Выключаем мигалку
}

turn_on_under_secure_led(void) {
    move_was_detected = 1;  // Включаем мигалку
    PORTB |= (1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN);  // Включаем мигалку
}


turn_off_siren(void) {
    sirena_repeat_cnt = MAX_SIRENA_REPEAT_CNT;  // сирену в макс число
    DDRB &= ~(1 << SIREN_PIN);  // выкл сирены
    PORTB &= ~(1 << SIREN_PIN); // Выключаем сирену
}

turn_on_siren(void) {
    sirena_repeat_cnt = 0;
    DDRB |= (1 << SIREN_PIN);  // Включаем сирену
    PORTB |= (1 << SIREN_PIN);  // Включаем сирену
}

check_siren_need_turn_off(void) {
    if (sirena_repeat_cnt >= MAX_SIRENA_REPEAT_CNT) {
        turn_off_siren();
    } else {
        ++sirena_repeat_cnt;
    }
}


void clear_prev_states(void) {
    uint8_t i;
    for (i = 0; i < NUM_SENSORS; i++) {
        prevStates[i] = 0;
    }
}

uint8_t is_secure_mode_btn_pressed() {
    return !(PIND & (1 << SECURE_MODE_BTN_PIN));  // Кнопка нажата, если значение на пине 0
}

ISR(INT1_vect) {
    // Обработчик прерывания
    usart_print("\r\n[INTERRUPTED]\r\n");
    if (is_secure_mode_btn_pressed()) {
        is_under_secure = !is_under_secure;

        turn_off_under_secure_led();
        turn_off_siren();
        clear_prev_states();

        if (is_under_secure) {
            PORTB |= (1 << UNDER_SECURE_LED_PIN); // включаем светодиод
            usart_print("[WARNING]: SECURE MODE ENABLED\r\n");
        } else {
            PORTB &= ~(1 << UNDER_SECURE_LED_PIN); // выключаем светодиод
            usart_print("[WARNING]: SECURE MODE DISABLED\r\n");
        }

        _delay_ms(500);  // Антидребезговая задержка
    }
}

init_pins(void) {
    // Инициализация ввода на порту A (датчик движения)
    DDRA = 0x00;  // Настроить все биты порта A как входы (ввод)
    PORTA = 0x00;  // Включить подтягивающие резисторы на порту C

    // Инициализация вывода на порту B (светодиоды)
    DDRB = 0xFF;  // Настроить все биты порта B как выходы (вывод)
    PORTB = 0x00;  // Установить начальное состояние порта B (все LED выключены)
    DDRB |= (1 << MOVE_UNDER_SECURE_WAS_DETECTED_LED_PIN); // включаем выход мигалки

    turn_off_siren();
    clear_prev_states();

    init_interrupt_button();
    USART_init();
}


turn_on_led_PB(uint8_t i) {
    PORTB |= (1 << outputPins[i]);
}

turn_off_led_PB(uint8_t i) {
    PORTB &= ~(1 << outputPins[i]);
}


uint8_t sensor_detect_move(uint8_t index) {
    return PINA & (1 << inputPins[index]);
}

void process_sensor_detects_move(uint8_t sensor_id, char *buffer) {
    turn_on_led_PB(sensor_id);

    if (prevStates[sensor_id]) {
        if (is_under_secure) {
            turn_on_under_secure_led();
            turn_on_siren();
            sprintf(buffer, "[ERROR]: Siren! MSensor [%u] detects move >= 2 times!\r\n", sensor_id);
        } else {
            sprintf(buffer, "[INFO]: MSensor [%u] detects move >= 2tms, not secure\r\n", sensor_id);
        }
    } else {
        sprintf(buffer, "[INFO]: MSensor [%u] detects move 1 time, check repeat\r\n", sensor_id);
    }
}

void process_sensor_empty(uint8_t sensor_id, char *buffer) {
    turn_off_led_PB(sensor_id);

    if (prevStates[sensor_id]) {
        sprintf(buffer, "[INFO]: MSensor [%u] not repeat last detected move, skip\r\n", sensor_id);
    } else {
        sprintf(buffer, "[INFO]: MSensor [%u] empty\r\n", sensor_id);
    }
}

int main(void) {
    init_pins();

    char buffer[50];
    while (1) {
        uint8_t i;
        for (i = 0; i < NUM_SENSORS; i++) {
            uint8_t is_sensor_detects_move = sensor_detect_move(i);

            if (is_sensor_detects_move) {
                process_sensor_detects_move(i, buffer); // С датчика приходит 1 (обнаружено движение), включаем
            } else {
                process_sensor_empty(i, buffer); // С датчика приходит 0 (движение не обнаружено)
            }

            prevStates[i] = is_sensor_detects_move; // Запоминаем прошлое состояние
            usart_print_secure_flag(is_under_secure, buffer);
        }

        check_siren_need_turn_off();
        _delay_ms(DELAY_MS);
    }

    return 0;
}
