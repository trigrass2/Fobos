/*
 * Universal_controller_PC_protocol.h
 *
 *  Created on: 19th march 2018
 *      Author: Zilkov
 *      Protocol version: 10
 */

#ifndef UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
#define UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
#include <stdint.h>
/*
 * Структура протокола обмена пакетами по TCP/IP между ПК и контроллером в Фобос:
 * 	===TX (от ПК к контроллеру):
 * 	CMD 	(команда, uint8_t)
 * 	N		(количество байт передачи данных uint8_t. N - переменное значение от 0 до 255, если N = 0, то набор данных отсутствует)
 * 	DATA[N]	(массив данных, выставленный для передачи элементов типа uint8_t)
 * 	===RX (от контроллера к ПК):
 * 	CMD 	(команда, uint8_t)
 * 	N		(количество байт передачи данных uint8_t. N - переменное значение от 0 до 255, если N = 0, то набор данных отсутствует)
 * 	DATA[N]	(массив данных, выставленный для передачи элементов типа uint8_t)
 */

// !!!!! Здесь расписать подробнее про WatchDog !!!!!
// посылать любую команду не реже чем 250 мс!

/*
 * 	Команда Echo:
 * 	===TX
 * 	CMD = 0		N = x	DATA0..DATA[x - 1]		// Произвольный набор данных. Где, x - переменное значение от 0 до 255, если x = 0, то набор данных отсутствует
 * 	===RX
 * 	CMD = 0		N = x	DATA0..DATA[x - 1]		// в ответ всегда приходит, то что было в запросе
 *
 *  Примечание: команда Echo (CMD = 0) в ответе не содержит байт статуса! Все остальные в обязательном порядке содержат байт статуса и принимают значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Изменение IP адреса:
 * 	===TX
 * 	CMD = 1		N = 4	DATA0..DATA3 = {XX, XX, XX, XX}		// IP адрес. По умолчанию: {0xC0, 0xA8, 0x64, 0x01} = "192.168.100.1"
 * 	===RX
 * 	CMD = 1		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Изменение маски подсети:
 * 	===TX
 * 	CMD = 2		N = 4	DATA0..DATA3 = {XX, XX, XX, XX}		// Маска подсети. По умолчанию: {0xFF, 0xFF, 0xFF, 0x00} = "255.255.255.0"
 * 	===RX
 * 	CMD = 2		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Изменение номер порта:
 * 	===TX
 * 	CMD = 3		N = 2	DATA0..DATA1 = {high_byte, low_byte}		// (uint16_t) Номер порта. По умолчанию: {0x3A, 0x98} = 15000
 * 	===RX
 * 	CMD = 3		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Переключение на статический/динамический (DHCP клиент):
 * 	===TX
 * 	CMD = 4		N = 1	DATA0 = 1 - static, 2 - dynamic (DHCP клиент)		// Режим сети. По умолчанию: 2 - с DHCP
 * 	===RX
 * 	CMD = 4		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Изменение Timeout:
 * 	===TX
 * 	CMD = 5		N = 2	DATA0..DATA0 = {high_byte, low_byte}		// (uint16_t)timeout в ms. !!!!!!!!!!!!!!!!!! Расписать что за Timeout !!!!!!!!!!!!!!!!.  По умолчанию: {0x00, 0x64} = 100
 * 	===RX
 * 	CMD = 5		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Reset устройства Ethernet:
 * 	===TX
 * 	CMD = 6		N = 0
 * 	===RX
 * 	CMD = 6		N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Получение MAC адреса платы:
 *	===TX
 *	CMD = 7		N = 0
 *	===RX
 *	CMD = 7		N = 7	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *						DATA1..DATA6 = {XX, XX, XX, XX, XX, XX}		// MAC адрес платы. Уникальный для каждой платы
 *												(по умолчанию {0x00,0x08,0xDC,0x0F,0x00,0x01})
 *
 * 	IP адрес платы:
 *	===TX
 *	CMD = 254	N = 0
 *	===RX
 *	CMD = 254	N = 5	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1..DATA4 = {XX, XX, XX, XX}		// IP адрес платы
 *
 * 	Адрес маски подсети платы:
 *	===TX
 *	CMD = 253	N = 0
 *	===RX
 *	CMD = 253	N = 5	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *						DATA1..DATA4 = {XX, XX, XX, XX}		// Mask подсети
 *
 * 	Порт платы:
 *	===TX
 *	CMD = 252	N = 0
 *	===RX
 *	CMD = 252	N = 3	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1..DATA2 = {high_byte, low_byte}		// Номер порта
 *
 * 	Режим сети (статический или динамический):
 *	===TX
 *	CMD = 251	N = 0
 *	===RX
 *	CMD = 251	N = 3	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1 = 1 - static, 2 - dynamic (DHCP клиент)		// Режим сети.
 *
 * 	Значение Timeout
 *	===TX
 *	CMD = 250	N = 0
 *	===RX
 *	CMD = 250	N = 3 	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1..DATA2 = {high_byte, low_byte}		// (uint16_t)timeout (ms). Например: {0x00, 0x64} = 100
 *
 * 	Запрос состояния всех датчиков (Концевики, в каком положении С-рама):
 * 	===TX
 * 	CMD = 10	N = 0
 * 	===RX
 *	CMD = 10	N = 4	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - Bit's number (см. АДН175.03.50.000 Э4):
 * 						0 - состояние "датчик поворота рабочий нижний" 		(S3)
 * 		     	                	1 - состояние "датчик поворота рабочий верхний"		(S4)
 *      	    	            		2 - не исп.(по умолч. 0)
 *          	    	        		3 - не исп.(по умолч. 0)
 *              	    	    		4 - не исп.(по умолч. 0)
 *                  	    			5 - не исп.(по умолч. 0)
 *                      			6 - состояние "датчик поворота нижний аварийный"	(S1)
 *                      			7 - состояние "датчик поворота верхний аварийный"	(S2)
 *			DATA2 - значение датчиков "стол подъёмный" (концевики верхний и нижний соответственно в байте)
  *			DATA3 -	Bit's number:
 * 						0 - не исп.(по умолч. 0)
 *						1 - состояние "аварийный выключатель линейного мотора"
 *						2 - состояние "аварийный выключатель линейного мотора"
 *						3 - состояние "датчик положения стола ЛЕВЫЙ"
 *						4 - состояние "датчик положения стола ПРАВЫЙ"
 *						5 - не исп.(по умолч. 0)
 *						6 - не исп.(по умолч. 0)
 *						7 - не исп.(по умолч. 0)
 *
 * 	Состояние генератора
 *	===TX
 *	CMD = 11	N = 0
 *	===RX
 *	CMD = 11	N = 2	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 != 0 - все интерлоки сработали, DATA1 = 0 - не работают		// интерлоки генератора и двери
 *
 * 	Положение сервопривода (значение энкодера, базирование)
 *	===TX
 *	CMD = 12	N = 0
 *	===RX
 *	CMD = 12	N = 6	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1..DATA4 = {high_byte, .. , .. , low_byte} - значение энкодера, микрометры
 *			DATA5 = 0 - не сбазирована С-рама, DATA3 != 0 - сбазирована		// базирование
 *
 * 	Состояние готовности аппарата
 *	===TX
 *	CMD = 13	N = 0
 *	===RX
 *	CMD = 13	N = 2	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 != 0 - аппарат готов для работы, DATA1 = 0 - не готов
 *
 * 	Версия прошивки
 *	===TX
 *	CMD = 14	N = 0
 *	===RX
 *  	CMD = 14	N = x + 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *  				DATA1-DATA[x] - информация о прошивке	// строка следующего вида: "Fobos embedded software version X.Y", где X,Y - версия прошивки
 *
 * 	Выполнить базирование и повернуть С-раму в начальное положение
 *	===TX
 *	CMD = 20	N = 0
 *	===RX
 *	CMD = 20	N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Команды работы с фобосом
 *	===TX
 *	CMD = 21	N = 2	DATA0 = 1 - старт, DATA0 = 0 - отмена,
 				DATA1 = 1 - тип сканирования "фронтальное сканирование", DATA1 = 2 - тип сканирования "боковое сканирование", DATA1 = 3 - тип сканирования "фронтальное+боковое"
 *	===RX
 *	CMD = 21	N = 1	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	Проверка начальных установок для готовности аппарата фобос
 *	===TX
 *	CMD = 22	N = 0
 *	===RX
 *	CMD = 22	N = 11	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1..DATA4 - значение энкодера, мм ({High_byte, .. , .. , low_byte})
 *				DATA5 - датчик базирования (!0-акт.)
 *				DATA6 - аварийные концевики №1 и 2
 *                      	DATA7 - Bit's number:
 *							0 - состояние "датчик поворота рабочий нижний" 		(S3)
 *							1 - состояние "датчик поворота рабочий верхний"		(S4)
 *							2 - не исп.(по умолч. 0)
 *							3 - не исп.(по умолч. 0)
 *							4 - не исп.(по умолч. 0)
 *							5 - не исп.(по умолч. 0)
 *							6 - состояние "датчик поворота нижний аварийный"	(S1)
 *							7 - состояние "датчик поворота верхний аварийный"	(S2)
 *				DATA8..DATA9 - состояние мотора {High byte, low byte}
 *				DATA10 - Состояние интерлоков (!0 - акт.)
 *
 *	Препятствие на пути С-рамы
 *	===TX
 *	CMD = 30	N = 0
 *	===RX
 *	CMD = 30	N = 2   DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 != 0 - препятствие C-рамы, DATA1 = 0 - отсутствие препятствия
 */

// Команды:
#define FOBOS_ETH_ECHO				0
#define FOBOS_ETH_CHANGE_IP			1
#define FOBOS_ETH_CHANGE_MASK			2
#define FOBOS_ETH_CHANGE_PORT			3
#define FOBOS_ETH_DHCP				4
#define FOBOS_CHANGE_TIMEOUT			5
#define FOBOS_ETH_RST				6 	// сброс в значения по умолчанию
#define FOBOS_ETH_GET_MAC			7
#define FOBOS_ETH_GET_IP			255-FOBOS_ETH_CHANGE_IP
#define FOBOS_ETH_GET_MASK			255-FOBOS_ETH_CHANGE_MASK
#define FOBOS_ETH_GET_PORT			255-FOBOS_ETH_CHANGE_PORT
#define FOBOS_ETH_GET_DHCP_STATE		255-FOBOS_ETH_DHCP
#define FOBOS_ETH_GET_TIMEOUT			255-FOBOS_CHANGE_TIMEOUT

#define FOBOS_SENSORS_STATE			10	// данные со всех датчиков (концевых)
#define FOBOS_GENERATOR_STATE			11	// интерлоки
#define FOBOS_SERVOMOTOR_PLACEMENT		12	// положение сервомотора
#define FOBOS_STATEMENT				13 	// состояние готовности аппарата
#define FOBOS_EMB_SOFT_VER			14	// версия прошивки платы

#define FOBOS_CMD_BASING			20	// выполнить базирование и повернуть С-раму в начальное положение
#define FOBOS_CMD_WORK				21	// команды работы с фобосом start (DATA0 = 0xFF), cancel (DATA0 = 0),
							// выбор типа сканирования "фронтальное сканирование" (DATA1 = 1),
							// "боковое сканирование" (DATA1 = 2),
							// "фронтальное+боковое" (DATA1 = 3)
#define FOBOS_CMD_READY_STATEMENT		22	// Проверка начальных установок для готовности аппарата фобос

#define FOBOS_CMD_BARRIER			30

// Статус ответа:
#define FOBOS_ETH_ERR_NO			0 // нормальное завершение команды
#define FOBOS_ETH_ERR_TX			1 // ошибка обмена
#define FOBOS_ETH_ERR_BU			2 // устройство занято
#define FOBOS_ETH_ERR_RE			3 // устройство не готово
#define FOBOS_ETH_ERR_PA			4 // неправильные параметры
#define FOBOS_ETH_ERR_NR			5 // устройство не отвечает
#define FOBOS_ETH_ERR_CMD			6 // неизвестная команда

// Структура пакета
typedef union {
	struct {
		uint8_t CMD;
		uint8_t bytes_in_packet_N;
		uint8_t data[256];
	} fobos_protocol_buf_t;
	uint8_t data_to_transmit[258];
} fobos_protocol_buf_u;

#endif /* UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_ */
