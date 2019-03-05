/*
 * Universal_controller_PC_protocol.h
 *
 *  Created on: 5th march 2018
 *      Author: Zilkov
 */

#ifndef UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
#define UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_
#include <stdint.h>
/*
 * ��������� ��������� ������ �������� �� TCP/IP ����� �� � ������������ � �����:
 * 	CMD 	(�������, uint8_t)
 * 	N		(���������� ���� �������� ������ uint8_t)
 * 	DATA[N]	(������ ������, ������������ ��� �������� ��������� ���� uint8_t)
 */

/*
 * 	*** �������� ������ (CMD): ***
 * 	0 - ������� Echo //�������� ������� ����������� ����� ������ 250 ��!

 * 	��������� �������:
 * 	1 - ��������� IP ������
 * 	2 - ��������� ����� �������
 * 	3 - ��������� �����
 * 	4 - ������������ �� �����������/������������ (DHCP ������)
 * 	5 - ��������� Timeout
 * 	6 - Reset ���������� Eth
 *
 *	������� ������:
 * 	10 - ������ ��������� ���� �������� (���������, � ����� ��������� �-����)
 * 	11 - ��������� ����������
 * 	12 - ��������� ������������ (�������� ��������, ������ �����������)
 * 	13 - ��������� ��������
 * 	14 - ������ ��������
 *
 * 	������� ����������:
 * 	20 - ��������� ����������� � ��������� �-���� � ��������� ���������
 * 	21 - ������� ������ � ������� start (DATA0 = 1), cancel (DATA0 = 2),
 * 	����� ���� ������������ "����������� ������������" (DATA1 = 1), "������� ������������" (DATA1 = 2), "�����������+�������" (DATA1 = 3)
 *	22 - ������� ���������� �������� �����
 *
 *	������� ������ ������ ��������:
 *	30 - ����������� �� ���� �-����
 *
 *	������� �������������:
 *	255(0xFF) - ������������ ��������� �������. ���� DATA0 == 0, �� ������� ������� �������. ���� DATA0 != 0 - �������� FOBOS_ETH_ERR
 *	!!!!!!!!!!!!	� ������� ���������� �������� ����� ����������� ��� �������������.	!!!!!!!!!!!!!!!!
 */

/*
 * ***������***
 * 	������������� ��������� ������:
 * 	* 	===TX (�� �� � �����������):
 * 	CMD = 0		N = x	DATA0..DATA[x - 1] 	// ���, x - ���������� �������� �� 0 �� 255, ���� x=0, �� ����� ������ DATA �����������
 * 	===RX (�� ����������� � ��)
 * 	CMD = 0		N = x	DATA0..DATA[x - 1]  // � ����� ������ ��������, �� ��� ���� � �������
 *
 *  ����������: ������� Echo CMD = 0 � ������ �� �������� ���� �������! ��� ��������� � ������������ ������� �������� ���� ������� � ��������� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *
 * 	===TX (�� �� � �����������):
 * 	CMD = 1		N = 4	DATA0..DATA3 = {xxx,xxx,xxx,xxx}			//�� ���������: IP 192.168.100.1
 * 	===RX (�� ����������� � ��)
 * 	CMD = 1		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 2		N = 4	DATA0..DATA3 = {xxx,xxx,xxx,xxx}			//�� ���������: Mask 255.255.255.0
 * 	===RX
 * 	CMD = 2		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 3		N = 2	DATA0..DATA1 = {high_byte, low_byte}		//�� ���������: Port 15000
 * 	===RX
 * 	CMD = 3		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 4		N = 1	DATA0 = 0 - static, 1 - dynamic (DHCP)		//�� ���������:	��� DHCP (�.�. 0)
 * 	===RX
 * 	CMD = 4		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 5		N = 2	DATA0..DATA1 = {high_byte, low_byte} (uint16_t)timeout (ms)		//�� ���������: 100 ��
 * 	===RX
 * 	CMD = 5		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	===TX
 * 	CMD = 6		N = 1	DATA0 = 1
 * 	===RX
 * 	CMD = 6		N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 * 	�������� ���������� � �������� (��� �������):
 * 	===TX (�� �� � �����������):
 * 	CMD = 10	N = 0
 * 	===RX (�� ����������� � ��)
 *	CMD = 10	N = 4	DATA0 = 0..6 // �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - �������� �������� �-���� S2, S1 (�� ���175.03.50.000 �4)
 *						DATA2 - �������� �������� "���� ���������" (��������� ������� � ������ �������������� � �����)
 *						DATA3 - "������� ������" (������ ��������� ����� � ���. ��������� �����, ������ ����������� (��-������),
 *								��������� ����� ������ �-����, ������ ��������� ����� � ���. ��������� ������)
 *
 *	===TX (�� �� � �����������):
 *	CMD = 11	N = 0
 *	===RX (�� ����������� � ��)
 *	CMD = 11	N = 2	DATA0 = 0..6 // �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - �������� ����������
 *
 *	===TX (�� �� � �����������):
 *	CMD = 12	N = 0
 *	===RX (�� ����������� � ��)
 *	CMD = 12	N = 4	DATA0 = 0..6 // �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1..DATA2 = {high_byte, low_byte} - �������� ��������
 *						DATA3 - ������ ����������� (0 �� �����, !0 - �����)
 *	===TX (�� �� � �����������):
 *	CMD = 13	N = 0
 *	===RX (�� ����������� � ��)
 *	CMD = 13	N = 2	DATA0 = 0..6 // �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 - !0 ������� ����� ��� ������, 0 - �� �����
 *	===TX (�� �� � �����������):
 *	CMD = 14	N = 0
 *	===RX
 *  CMD = 14	N = x + 1	DATA0 = 0..6 // �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *  						DATA1-DATAx - ������ //"Fobos embedded software version X.Y" ��� X,Y - ������ ��������
 *
 *	������� ����������:
 *	===TX
 *	CMD = 20	N = 1	DATA0 = 1
 *	===RX
 *	CMD = 20	N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *	===TX
 *	CMD = 21	N = 2		start (DATA0 = 1), cancel (DATA0 = 0),
 * 	��� ������������ "����������� ������������" (DATA1 = 1), "������� ������������" (DATA1 = 2), "�����������+�������" (DATA1 = 3)
 *	===RX
 *	CMD = 21	N = 1	DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *
 *
 *	������� ������ ��������:
 *	===TX
 *	CMD = 30	N = 0		����������� C-����
 *	===RX
 *	CMD = 30	N = 2   DATA0 = 0..6 // DATA0 ��� �������� �� ��������� FOBOS_ETH_ERR_NO...FOBOS_ETH_ERR_CMD
 *                      DATA1 = 1 - ����������� C-���� ��� ���� DATA1 = 0 - ���������� �����������
 */
#define FOBOS_ETH_ECHO				0
#define FOBOS_ETH_CHANGE_IP			1
#define FOBOS_ETH_CHANGE_MASK		2
#define FOBOS_ETH_CHANGE_PORT		3
#define FOBOS_ETH_DHCP				4
#define FOBOS_CHANGE_TIMEOUT		5
#define FOBOS_ETH_RST				6 // ����� � �������� �� ��������� (������� ����)

#define FOBOS_SENSORS_STATE			10
#define FOBOS_GENERATOR_STATE		11
#define FOBOS_SERVOMOTOR_PLACEMENT	12
#define FOBOS_STATEMENT				13 	// ��������� ���������� ��������
#define FOBOS_EMB_SOFT_VER			14	//������ �������� �����

#define FOBOS_CMD_BASING_SERVO		20
#define FOBOS_CMD_WORK				21

#define FOBOS_CMD_BARRIER			30

#define FOBOS_ETH_ERR_NO			0 // ���������� ���������� �������
#define FOBOS_ETH_ERR_TX			1 // ������ ������
#define FOBOS_ETH_ERR_BU			2 // ���������� ������
#define FOBOS_ETH_ERR_RE			3 // ���������� �� ������
#define FOBOS_ETH_ERR_PA			4 // ������������ ���������
#define FOBOS_ETH_ERR_NR			5 // ���������� �� ��������
#define FOBOS_ETH_ERR_CMD			6 // ����������� �������
typedef union {
	struct{
		uint8_t CMD;
		uint8_t bytes_in_packet_N;
		uint8_t data[256];
	}fobos_protocol_buf_t;
	uint8_t data_to_transmit[258];
}fobos_protocol_buf_u;

#endif /* UNIVERSAL_CONTROLLER_PC_PROTOCOL_H_ */
