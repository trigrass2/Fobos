// encoding=utf-8; tabsize=8

/*
 * Universal_controller_PC_protocol.h
 *
 *  Created on: 3rd may 2019
 *      Author: Zilkov
 *      Protocol version: 15
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

/*
 * Watchdog будет перезагружать плату и выставлять в настройки по умолчанию все элементы аппарата ФОБОС,
 * в т.ч. останавливать все движения если плата не примет никакого сообщения (любого) по Ethernet, посылать любую команду не реже чем 1000 мс!
 * TODO: планируется сбрасывать не все настройки, а только часть из них.
 */

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
 * 	CMD = 5		N = 2	DATA0..DATA1 = {high_byte, low_byte}		// (uint16_t)timeout в ms. Время ожидания ответа.  По умолчанию: {0x00, 0x64} = 100
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
 *				DATA1..DATA6 = {XX, XX, XX, XX, XX, XX}		// MAC адрес платы. Уникальный для каждой платы (по умолчанию {0x00,0x08,0xDC,0x0F,0x00,0x01})
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
 *				DATA1..DATA4 = {XX, XX, XX, XX}		// Mask подсети
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
 *	CMD = 251	N = 2	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
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
 *	CMD = 10	N = 3	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      	DATA1 - Bit's number (см. АДН175.03.50.000 Э4):
 * 						0 - состояние "датчик поворота рабочий нижний" Lateral view	(S3) 0 - act
 *                      			1 - состояние "датчик поворота рабочий верхний"	AP view		(S4) 0 - act
 *                  	    			2 - не исп.(по умолч. 0)
 *                  	    			3 - не исп.(по умолч. 0)
 *                  	    			4 - не исп.(по умолч. 0)
 *                  	    			5 - не исп.(по умолч. 0)
 *                      			6 - состояние "датчик поворота нижний аварийный"		(S1) 0 - act
 *                      			7 - состояние "датчик поворота верхний аварийный"		(S2) 0 - act
 *				DATA2 -	Bit's number:
 *						0 - не исп.(по умолч. 0)
 *						1 - состояние "аварийный выключатель линейного мотора"		1 - act
 *						2 - состояние "аварийный выключатель линейного мотора"		1 - act
 *						3 - состояние "датчик положения стола ЛЕВЫЙ"			1 - act
 *						4 - состояние "датчик положения стола ПРАВЫЙ"			1 - act
 *						5 - не исп.(по умолч. 0)
 *						6 - не исп.(по умолч. 0)
 *						7 - не исп.(по умолч. 0)
 *
 * 	Состояние интерлоков
 *	===TX
 *	CMD = 11	N = 0
 *	===RX
 *	CMD = 11	N = 2	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1 != 0 - все интерлоки сработали, DATA1 = 0 - не работают	// интерлоки генератора и двери
 *
 * 	Положение сервопривода (значение энкодера)
 *	===TX
 *	CMD = 12	N = 0
 *	===RX
 *	CMD = 12	N = 5	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1..DATA4 = {high_byte, .. , .. , low_byte} - значение энкодера, микрометры
 *
 * 	Состояние готовности аппарата
 *	===TX
 *	CMD = 13	N = 0
 *	===RX
 *	CMD = 13	N = 2	DATA0 = 0..6		// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1 != 0 - аппарат готов для работы, DATA1 = 0 - не готов
 *
 * 	Версия прошивки
 *	===TX
 *	CMD = 14	N = 0
 *	===RX
 *	CMD = 14	N = x + 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *					DATA1-DATA[x] - информация о прошивке	// строка следующего вида:
 *  							"Fobos embedded software version X.Y", где X,Y - версия прошивки
 *
 * 	Повернуть С-раму в AP view и сбазировать линейный мотор
 *	===TX
 *	CMD = 20	N = 0
 *	===RX
 *	CMD = 20	N = 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *
 * 	Команды работы с фобосом
 *	===TX
 *	CMD = 21	N = 1	DATA0 = 1..2	//Повернуть С-раму в положение 1 (Lateral view) либо 2 (AP view)
 *	===RX
 *	CMD = 21	N = 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	Управление лаймпой "Не входить"
 *	===TX
 *	CMD = 22	N = 1	DATA0 = 0 (лампа "Не входить" выкл.), !0 (лампа "Не входить" вкл.)
 *	===RX
 *	CMD = 22	N = 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	Проверка базирования С-рамы
 *	===TX
 *	CMD = 23 	N = 0
 *	===RX
 *	CMD = 23 	N = 2	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1 = 0 (если рама не сбазирована), !0 (если рама сбазирована)
 *
 *	===TX
 *	CMD = 24	N = 1	DATA0 - 1 - переместить линейный мотор влево на 9 см
 *					2 - переместить линейный мотор влево,
 *					3 - вправо (относ. лицевой стороны фобоса)
 *	===RX
 *	CMD = 24	N = 1	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	===TX
 *	CMD = 25	N = 0
 *	===RX
 *	CMD = 25	N = 2	DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *				DATA1 = 0 - лин. мотор не достиг заданной точки, !0 - достиг
 *				DATA2 = 2 - С-рама находится в положении AP view, 1 - С-рама находится в положении Lateral view,
 *					0 - С-рама находится в неизвестном положении
 *
 *	Остановка ВСЕХ движений, сканирования
 *	===TX
 *	CMD = 30	N = 0
 *	===RX
 *	CMD = 30	N = 1   DATA0 = 0..6	// Статус ответа. Значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
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
#define FOBOS_CMD_ROTATION			21	// команды работы с фобосом
#define FOBOS_CMD_LAMP				22	// управление лаймпой "Не входить"
#define FOBOS_CMD_BASING_STATEMENT		23	// проверка базирования С-рамы
#define FOBOS_CMD_MOTION			24	//старт передвижения линейного мотора
#define FOBOS_CMD_MOTION_STATUS			25	//Отслеживание завершения перемещения линейного мотора и С-рамы

#define FOBOS_CMD_STOP				30	// остановка ВСЕХ движений, сканирования

// Статус ответа:
#define FOBOS_ETH_ERR_NO			0	// нормальное завершение команды
#define FOBOS_ETH_ERR_TX			1	// ошибка обмена
#define FOBOS_ETH_ERR_BU			2	// устройство занято
#define FOBOS_ETH_ERR_RE			3	// устройство не готово
#define FOBOS_ETH_ERR_PA			4	// неправильные параметры
#define FOBOS_ETH_ERR_NR			5	// устройство не отвечает
#define FOBOS_ETH_ERR_CMD			6	// неизвестная команда

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
