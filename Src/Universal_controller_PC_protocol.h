﻿/*
 * Universal_controller_PC_protocol.h
 *
 *  Created on: 20 дек. 2018 г.
 *      Author: Zilkov
 */

#ifndef UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
#define UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
/*
 * Структура протокола обмена пакетами по TCP/IP между ПК и контроллером в Фобос:
 * 	CMD 	(команда, uint8_t)
 * 	N		(количество байт передачи данных uint8_t)
 * 	DATA[N]	(массив данных, выставленный для передачи элементов типа uint8_t)
 */

/*
 * 	*** Значение команд (CMD): ***
 * 	0 - команда Echo

 * 	Системные команды:
 * 	1 - Изменение IP адреса
 * 	2 - Изменение маски подсети
 * 	3 - Изменение порта
 * 	4 - Переключение на статический/динамический (DHCP клиент)
 * 	5 - Изменение Timeout
 * 	6 - Reset устройства Eth
 *
 *	Команды опроса:
 * 	10 - запрос состояния всех датчиков (Концевики, в каком положении С-рама)
 * 	11 - состояние генератора
 * 	12 - положение сервопривода (значение энкодера, датчик базирования)
 *  13 - TODO: Описание????
 *
 * 	Команды исполнения:
 * 	20 - выполнить базирование и повернуть С-раму в начальное положение
 * 	21 - команды работы с фобосом start (DATA0 = 1), cancel (DATA0 = 2),
 * 	выбор типа сканирования "фронтальное сканирование" (DATA1 = 1), "боковое сканирование" (DATA1 = 2), "фронтальное+боковое" (DATA1 = 3)
 *	22 - команда готовности аппарата фобос
 *
 *	Команды ошибок работы аппарата:
 *	30 - препятствие на пути С-рамы
 *
 *	Команда подтверждения:
 *	255(0xFF) - подтверждает получение посылки. Если DATA0 == 0, то команда принята успешно. Если DATA0 != 0 - смотреть FOBOS_ETH_ERR
 *	!!!!!!!!!!!!	В течение разработки протокол может дополняться при необходимости.	!!!!!!!!!!!!!!!!
 */

/*
 * ***Пакеты***
 * 	Подтверждение системных команд:
 * 	* 	===TX (от ПК к контроллеру):
 * 	CMD = 0		N = x	DATA0..DATA[x - 1] 	// где, x - переменное значение от 0 до 255, если x=0, то набор данных DATA отсутствует
 * 	===RX (от контроллера к ПК)
 * 	CMD = 0		N = x	DATA0..DATA[x - 1]  // в ответ всегда приходит, то что было в запросе
 *
 *  Примечание: команда Echo CMD = 0 в ответе не содержит байт статуса! Все остальные в обязательном порядке содержат байт статуса и принимают значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *
 * 	===TX (от ПК к контроллеру):
 * 	CMD = 1		N = 4	DATA0..DATA3 = {xxx,xxx,xxx,xxx}			//По умолчанию: IP 192.168.100.1
 * 	===RX (от контроллера к ПК)
 * 	CMD = 1		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 2		N = 4	DATA0..DATA3 = {xxx,xxx,xxx,xxx}			//По умолчанию: Mask 255.255.255.0
 * 	===RX
 * 	CMD = 2		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 3		N = 2	DATA0..DATA1 = {high_byte, low_byte}		//По умолчанию: Port 15000
 * 	===RX
 * 	CMD = 3		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 4		N = 1	DATA0 = 0 - static, 1 - dynamic (DHCP)		//По умолчанию:	Без DHCP (т.е. 0)
 * 	===RX
 * 	CMD = 4		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 5		N = 2	DATA0..DATA1 = {high_byte, low_byte} (uint16_t)timeout (ms)		//По умолчанию: 100 мс
 * 	===RX
 * 	CMD = 5		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 6		N = 1	DATA0 = 1
 * 	===RX
 * 	CMD = 6		N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	Отправка информации о датчиках (при запросе):
 * 	===TX (от ПК к контроллеру):
 * 	CMD = 10	N = 0
 * 	===RX (от контроллера к ПК)
 *	CMD = 10	N = 4	DATA0 = 0..6 // значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - значение датчиков С-рамы S2, S1 (по АДН175.03.50.000 Э4)
 *						DATA2 - значение датчиков "стол подъёмный" (концевики верхний и нижний соответственно в байте)
 *						DATA3 - "боковая стойка" (датчик установки стола в раб. положение левый, датчик препятствия (ИК-датчик),
 *								аварийный левый датчик С-рамы, датчик установки стола в раб. положение правый)
 *
 *	===TX (от ПК к контроллеру):
 *	CMD = 11	N = 0
 *	===RX (от контроллера к ПК)
 *	CMD = 11	N = 2	DATA0 = 0..6 // значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - интерлок генератора
 *
 *	===TX (от ПК к контроллеру):
 *	CMD = 12	N = 0
 *	===RX (от контроллера к ПК)
 *	CMD = 12	N = 4	DATA0 = 0..6 // значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1..DATA2 = {high_byte, low_byte} - значение энкодера
 *						DATA3 - датчик базирования (0 не нажат, !0 - нажат)
 *	===TX (от ПК к контроллеру):
 *	CMD = 13	N = 0
 *	===RX (от контроллера к ПК)
 *	CMD = 13	N = 2	DATA0 = 0..6 // значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - !0 аппарат готов для работы, 0 - не готов
 *
 *	Команды исполнения:
 *	===TX
 *	CMD = 20	N = 1	DATA0 = 1
 *	===RX
 *	CMD = 20	N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	===TX
 *	CMD = 21	N = 2		start (DATA0 = 1), cancel (DATA0 = 0),
 * 	тип сканирования "фронтальное сканирование" (DATA1 = 1), "боковое сканирование" (DATA1 = 2), "фронтальное+боковое" (DATA1 = 3)
 *	===RX
 *	CMD = 21	N = 1	DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *
 *	Команды ошибок аппарата:
 *	===TX
 *	CMD = 30	N = 0		препятствие C-рамы
 *	===RX
 *	CMD = 30	N = 2   DATA0 = 0..6 // DATA0 это значение из диапазона FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 = 1 - препятствие C-рамы или если DATA1 = 0 - отсутствие препятствия
 */
#define FOBOS_ETH_ECHO				0
#define FOBOS_ETH_CHANGE_IP			1
#define FOBOS_ETH_MASK				2
#define FOBOS_ETH_CHANGE_PORT		3
#define FOBOS_ETH_DHCP				4
#define FOBOS_CHANGE_TIMEOUT		5
#define FOBOS_ETH_RST				6 // сброс в значения по умолчанию (описаны ниже)

#define FOBOS_SENSORS_STATE			10
#define FOBOS_GENERATOR_STATE		11
#define FOBOS_SERVOMOTOR_PLACEMENT	12
#define FOBOS_STATEMENT				13 // состояние готовности аппарата

#define FOBOS_CMD_BASING_SERVO		20
#define FOBOS_CMD_WORK				21

#define FOBOS_CMD_BARRIER			30

#define FOBOS_ETH_ERR_NO			0 // нормальное завершение команды
#define FOBOS_ETH_ERR_TX			1 // ошибка обмена
#define FOBOS_ETH_ERR_BU			2 // устройство занято
#define FOBOS_ETH_ERR_RE			3 // устройство не готово
#define FOBOS_ETH_ERR_PA			4 // неправильные параметры
#define FOBOS_ETH_ERR_NR			5 // устройство не отвечает
#define FOBOS_ETH_ERR_CMD			6 // неизвестная команда
typedef union {
	struct{
		uint8_t CMD;
		uint8_t bytes_in_packet_N;
		uint8_t data[253];
	}fobos_protocol_buf_t;
	uint8_t data_to_transmit[255];
}fobos_protocol_buf_u;

#endif /* UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_ */
