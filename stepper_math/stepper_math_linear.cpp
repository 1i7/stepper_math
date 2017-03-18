/**
 * stepper_math.cpp 
 *
 * Полезные математические функции для формирования траекторий
 * перемещения рабочего инструмента станка с ЧПУ на шаговых моторах
 * под управлением библиотеки stepper_h.
 *
 * LGPLv3, 2014-2017
 *
 * @author Антон Моисеев 1i7.livejournal.com
 */

#include "stepper.h"
#include "stepper_math.h"

#include <math.h>

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL 
#include "Arduino.h"
#endif // DEBUG_SERIAL


/******************************************************************/
/* Путешествие по линии */

/**
 * Подготовить линейное перемещение из текущей позиции в заданную точку
 * с заданной скоростью для одной координаты.
 * 
 * Координата точки относительна текущей позиции.
 *
 * Скорость задаём целым числом как <единица измерения координаты мотора> в секунду.
 * 
 * Допустим, шаг мотора (distance_per_step) - 7500 нанометров,
 * минимальная задержка между шагами (pulse_delay) - 1000микросекунд (1 миллисекунда).
 * 
 * Максимальное значение скорости получаем
 * 7500 нанометров в миллисекунду = 7500000 нанометров в секунду (параметр spd).
 * 
 * Минимальная скорость (целочисленное значение spd) - 1 нанометр в секунду -
 * это 1 миллиметр в 1000000 секунд (=277 часов ~11 дней).
 * 
 * Для микрометров получаем минимальную целочисленную скорость:
 * 1 микрометр в секунду = 1 миллиметр в 1000 секунд (~16 минут)
 * 
 * @param sm - мотор на выбранной координате
 * @param dl - сдвиг по указанной оси
 * @param spd - скорость перемещения, <единица измерения координаты мотора> в секунду
 *     0 для максимальной скорости
 * @return 0 - задание подготовлено, 
 *   > 0 - задание не подготовлено, код ошибки:
 *     STEPPER_MATH_ERR_TOO_FAST=1 - скорость превышает допустимую для выбранных моторов
 */
int prepare_line(stepper *sm, long dl, unsigned long spd) {
    #ifdef DEBUG_SERIAL
        Serial.print("prepare line:");
        Serial.print(sm->name);
        Serial.print("1=");
        Serial.print(sm->current_pos/1000000, DEC);
        Serial.print("um, ");
        Serial.print(sm->name);
        Serial.print("2=");
        Serial.print(dl, DEC);
        Serial.print("um, speed=");
        Serial.print(spd, DEC);
        Serial.println("um/s");
    #endif // DEBUG_SERIAL
    
    long steps;
    long mod_steps;
    unsigned long step_delay;
    
    steps = dl / sm->distance_per_step;
    mod_steps = steps >= 0 ? steps : -steps;
    
    #ifdef DEBUG_SERIAL
        Serial.print("steps=");
        Serial.print(steps, DEC);
        Serial.println();
    #endif // DEBUG_SERIAL
    
    // максимальная скорость, расстояние в секунду:
    // длина шага делить на минимальную задержку между шагами.
    // sm->pulse_delay - мкс, нужно конвертировать в секунды.
    // Очевидный способ - умножить делимое расстояние на 1млн,
    // а потом делить на микросекунды:
    // unsigned long max_spd = sm->distance_per_step * 1000000 / sm->pulse_delay;
    // проблема в том, что если значение distance_per_step больше, чем ~4300,
    // при умножении на 1000000 промежуточный результат выйдет за верхнуюю границу
    // 32битного unsigned long=2^32=4294967296
    // 
    // С другой стороны, если значение pulse_delay=1000микросекунд (обычно это так и будет),
    // а значение distance_per_step содержит часть, меньше 1000 (например, 7500), то
    // если сначала поделить distance_per_step на pulse_delay, а потом умножить
    // результат на 1млн:
    // unsigned long max_spd = (sm->distance_per_step / sm->pulse_delay) * 1000000;
    // мы потеряем часть distance_per_step на операции промежуточного округления
    // (не смертельно, т.к. значение будет в любом случае меньше, чем реальная
    // максимальная скорость, но все равно не очень приятно, т.к. потом
    // округленные знаки заполнятся миллионными нулями).
    // 
    // Компромиссный хитрый вариант - сначала умножить расстояние на 1000,
    // потом поделить на время а потом результат умножить еще раз на 1000.
    // Например: distance_per_step = 7500нм, pulse_delay=1000млс
    // 1) 7500*1000=7500000 (умещаемся в unsigned long), 
    // 2) 7500000/1000=7500 (не потеряли значимые разряды справа),
    // 3) 7500*1000=7500000 - финальный результат (опять уместились в unsigned long, 
    // при этом не потеряли исходных значимых разрядов)
    unsigned long max_spd = (sm->distance_per_step * 1000 / sm->pulse_delay) * 1000;
    
    if(spd == 0) {
        // движение с максимальной скоростью
        spd = max_spd;
    } else if(spd > max_spd) {
        // заданная скорость больше максимальной
        return STEPPER_MATH_ERR_TOO_FAST;
    }
    // время на прохождение диагонали - длина делить на скорость, микросекунды
    long dt = ((double)dl / (double)spd) * 1000000;
    
    // задержка между 2мя шагами, микросекунды
    step_delay = dt / mod_steps;
    
    #ifdef DEBUG_SERIAL
        Serial.print("spd=");
        Serial.print(spd, DEC);
        Serial.print("len/sec, dt=");
        Serial.print(dt, DEC);
        Serial.print("us, step_delay=");
        Serial.print(step_delay, DEC);
        Serial.println("us");
    #endif // DEBUG_SERIAL
    
    prepare_steps(sm, steps, step_delay);
    
    return 0;
}

/**
 * Подготовить линейное перемещение из текущей позиции в заданную точку
 * с заданной скоростью для одной координаты.
 * 
 * Координата точки абсолютна.
 *
 * @param sm - мотор на выбранной координате
 * @param cvalue - координата целевой точки
 * @param spd - скорость перемещения, <единица измерения координаты мотора> в секунду
 *     0 для максимальной скорости
 * @return 0 - задание подготовлено, 
 *   > 0 - задание не подготовлено, код ошибки:
 *     STEPPER_MATH_ERR_TOO_FAST=1 - скорость превышает допустимую для выбранных моторов
 */
int prepare_line_abs(stepper *sm, long cvalue, unsigned long spd) {
    // сдвиг по оси
    long dl = cvalue - sm->current_pos;
    return prepare_line(sm, dl, spd);
}
/**
 * Подготовить линейное перемещение из текущей позиции в заданную точку с заданной скоростью
 * для двух координат.
 * 
 * Координаты точки относительны текущей позиции.
 *
 * @param sm1 - мотор на координате 1
 * @param sm2 - мотор на координате 2
 * @param dl1 - сдвиг по оси 1
 * @param dl2 - сдвиг по оси 2
 * @param spd - скорость перемещения, <единица измерения координаты мотора> в секунду
 *     0 для максимальной скорости
 * @return 0 - задание подготовлено, 
 *   > 0 - задание не подготовлено, код ошибки:
 *     STEPPER_MATH_ERR_TOO_FAST=1 - скорость превышает допустимую для выбранных моторов
 */
int prepare_line_2d(stepper *sm1, stepper *sm2, long dl1, long dl2, unsigned long spd) {
    #ifdef DEBUG_SERIAL
        Serial.print("prepare line:");
        Serial.print(" ");
        Serial.print(sm1->name);
        Serial.print("1=");
        Serial.print(sm1->current_pos / 1000000, DEC);
        Serial.print("um, ");
        Serial.print(sm1->name);
        Serial.print("2=");
        Serial.print(dl1, DEC);
        Serial.print("um; ");
        Serial.print(sm2->name);
        Serial.print("1=");
        Serial.print(sm2->current_pos / 1000000, DEC);
        Serial.print("um, ");
        Serial.print(sm2->name);
        Serial.print("2=");
        Serial.print(dl2, DEC);
        Serial.print("um; speed=");
        Serial.print(spd, DEC);
        Serial.println("um/s");
    #endif // DEBUG_SERIAL
    
    long steps_sm1;
    long steps_sm2;
    long mod_steps_sm1;
    long mod_steps_sm2;
    unsigned long step_delay_sm1;
    unsigned long step_delay_sm2;
    
    // количество шагов по осям (знак задаёт направление)
    steps_sm1 = dl1 / sm1->distance_per_step;
    steps_sm2 = dl2 / sm2->distance_per_step;
    
    // количество шагов по осям (модуль)
    mod_steps_sm1 = steps_sm1 >= 0 ? steps_sm1 : -steps_sm1;
    mod_steps_sm2 = steps_sm2 >= 0 ? steps_sm2 : -steps_sm2;
    
    #ifdef DEBUG_SERIAL
        Serial.print("steps_x=");
        Serial.print(steps_sm1, DEC);
        Serial.print(", steps_y=");
        Serial.print(steps_sm2, DEC);
        Serial.println();
    #endif // DEBUG_SERIAL
    
    // максимальная скорость
    unsigned long max_spd_sm1 = (sm1->distance_per_step * 1000 / sm1->pulse_delay) * 1000;
    unsigned long max_spd_sm2 = (sm2->distance_per_step * 1000 / sm2->pulse_delay) * 1000;
    // максимальная скорость - меньшая из максимальных
    unsigned long max_spd = max_spd_sm1 < max_spd_sm2 ? max_spd_sm1 : max_spd_sm2;
    if(spd == 0) {
        // движение с максимальной скоростью
        spd = max_spd;
    } else if(spd > max_spd) {
        // заданная скорость больше максимальной
        return STEPPER_MATH_ERR_TOO_FAST;
    }
    
    // длина гипотенузы
    long dl = sqrt((double)dl1*(double)dl1 + (double)dl2*(double)dl2);
    
    // время на прохождение диагонали - длина делить на скорость, микросекунды
    long dt = ((double)dl / (double)spd) * 1000000;
    
    #ifdef DEBUG_SERIAL
        Serial.print("dl=");
        Serial.print(dl, DEC);
        Serial.print(", spd=");
        Serial.print(spd, DEC);
        Serial.println();
    #endif // DEBUG_SERIAL
    
    // задержка между 2мя шагами, микросекунды
    step_delay_sm1 = dt / mod_steps_sm1;
    step_delay_sm2 = dt / mod_steps_sm2;
    
    #ifdef DEBUG_SERIAL
        Serial.print("step_delay_x=");
        Serial.print(step_delay_sm1, DEC);
        Serial.print(", step_delay_y=");
        Serial.print(step_delay_sm2, DEC);
        Serial.println();
    #endif // DEBUG_SERIAL
    
    prepare_steps(sm1, steps_sm1, step_delay_sm1);
    prepare_steps(sm2, steps_sm2, step_delay_sm2);
    
    return 0;
}

/**
 * Подготовить линейное перемещение из текущей позиции в заданную точку с заданной скоростью
 * для двух координат.
 * 
 * Координаты точки абсолютны.
 *
 * @param sm1 - мотор на координате 1
 * @param sm2 - мотор на координате 2
 * @param cvalue1 - координата точки по оси 1
 * @param cvalue2 - координата точки по оси 2
 * @param spd spd - скорость перемещения, <единица измерения координаты мотора> в секунду
 *     0 для максимальной скорости
 * @return 0 - задание подготовлено, 
 *   > 0 - задание не подготовлено, код ошибки:
 *     STEPPER_MATH_ERR_TOO_FAST=1 - скорость превышает допустимую для выбранных моторов
 */
int prepare_line_2d_abs(stepper *sm1, stepper *sm2, long cvalue1, long cvalue2, unsigned long spd) {

    // сдвиг по оси
    long dl1 = cvalue1 - sm1->current_pos;
    long dl2 = cvalue2 - sm2->current_pos;
    
    return prepare_line_2d(sm1, sm2, dl1, dl2, spd);
}


