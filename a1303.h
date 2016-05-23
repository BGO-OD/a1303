//=============================================================================
//
//             --- CAEN SpA - Computing Systems Division ---
//
//	a1303.h
//
//	Header file for the CAEN A1303 CaeNet board driver.
//
//	February  2002 :   Created.
//
//=============================================================================

#ifndef _a1303_H
#define _a1303_H

// Defines for the a1303

#define PCI_DEVICE_ID_PLX_9030		0x9030
#define PCI_SUBDEVICE_ID_CAEN_A1303	0x2709

#define SEEK_SET			0
#define SEEK_CUR			1

#define PCI_SIZE_8			0x0001
#define PCI_SIZE_16			0x0002
#define PCI_SIZE_32			0x0003
/*
 --- OLD ---
#define IOCTL_TIMEOUT                 0
#define IOCTL_RESET                   1
#define IOCTL_LED                     2
*/

#define A1303_MAGIC			'3'

#define A1303_IOCTL_TIMEOUT		_IOW(A1303_MAGIC, 0, int)
#define A1303_IOCTL_RESET		_IO(A1303_MAGIC, 1)
#define A1303_IOCTL_LED			_IO(A1303_MAGIC, 2)

#define PCI_ID				0x0000
#define PCI_CSR				0x0004
#define PCI_CLASS			0x0008
#define PCI_MISC0			0x000C
#define PCI_BS				0x0010
#define PCI_MISC1			0x003C


//  A1303 Registers offsets
#define A1303_FIFO			(0)
#define A1303_REG			(1)
#define A1303_INTR			(2)
#define A1303_LED			(2)
#define A1303_RESET			(3)

//  Status Register Masks
#define NOINTR				0x26
#define RXFEM				1
#define IDLE		 (unsigned char)0xee
#define TXEFF				0x20
#define RXEFF				0x04

#define TIME_OUT			100000
#define TIME_OUT_TX			100000

#define TUTTOK				0
#define E_NO_A1303			-2

#define MAX_LENGTH_FIFO			4096
#endif
