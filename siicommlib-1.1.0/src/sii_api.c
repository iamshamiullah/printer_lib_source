/**
This software package supports the communications with SII thermal printers.
Copyright ( C ) 2009 - 2012 by Seiko Instruments Inc.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or ( at your option ) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/time.h>
#include <string.h>
#include <termios.h>
#include <linux/lp.h>
#include <sys/ioctl.h>
#include <aio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "sii_api.h"


#define TRUE								1
#define FALSE								0

// identifier of ASB data
#define AUTO_STATUS_HEAD_MASK			0xF0
#define AUTO_STATUS_HEAD_DATA			0xC0
#define AUTO_STATUS_OTHER_MASK			0xF0
#define AUTO_STATUS_OTHER_DATA			0xD0

// Default timeout
#define TIMEOUT_SEC						0
#define TIMEOUT_USEC_LONG				500*1000		// 500 ms
#define TIMEOUT_USEC_SHRT				50*1000		// 50 ms
#define TIMEOUT_USEC_LAN					0
#define TIMEOUT_SEC_LAN					2				// 2 s

#define WRITE_TIMEOUT_SEC				30				// 30 s
#define WRITE_TIMEOUT_NSEC				0
#define WRITE_TIMEOUT_USEC				0

// Max data size
#define MAX_WRITE_BYTES					1460

enum SII_INTERFACE_TYPE {	// device type
	DEV_USB = 0,				// 0 : USB
	DEV_SER,					// 1 : Serial
	DEV_LAN,					// 2 : LAN
	DEV_END					// Terminal
};

#define PORT_USB						"/dev/usb"			// device port name
#define PORT_SER						"/dev/ttyS"

#define PORT_LAN__IPV4_DELIMITER		'.'
#define TCP_PORT_NUMBER					9100
#define SPECIAL_PORT_NUMBER				26100

// Get Status relation
#define AUTO_STATUS_DATA_SIZE			8				// Size of ASB resp
#define GET_STATUS_RETRY_COUNT			10
#define GET_STATUS_WAIT_TIME			(10000)		// 10ms


// device info structure
typedef struct
{
	int					nDeviceFid;									// Device descriptor
	int					nDeviceType;									// Device type
	unsigned char		byAutoStatusBuf[AUTO_STATUS_DATA_SIZE]; 	// Auto status response data
	unsigned char		byAutoStatusIndex;							// Index of auto status data
	CALLBACK_FUNC		pfnUsrDefFunc;								// User defined function for call back status
	char				szIPAddress[INET_ADDRSTRLEN];				// IP address for LAN interface
} DEV_INFO, *PDEV_INFO;


// Macro
#define is_null( val )							( val == NULL )

#define OPEN_ERR()								\
{													\
	if( phDevInfo != NULL )						\
	{												\
		free( phDevInfo );						\
	}												\
	if( errno == EBUSY )							\
	{												\
		return ( ERR_SII_API_DEV_EBUSY );		\
	}												\
	else if( errno == EACCES )					\
	{												\
		return ( ERR_SII_API_DEV_EACCES );		\
	}												\
	else if( errno == ENXIO )					\
	{												\
		return ( ERR_SII_API_DEV_ENXIO );		\
	}												\
	else 											\
	{												\
		return ( ERR_SII_API_DIS_OPEN );		\
	}												\
}

// open device function
int open_device_usb( PDEV_INFO, char * );
int open_device_ser( PDEV_INFO, char * );
int open_device_lan( PDEV_INFO, char * );

// write device function
static int write_device(
			PDEV_INFO,
			unsigned char *,
			size_t,
			size_t *,
			struct timeval * );
static int write_device_lan(
			PDEV_INFO,
			unsigned char *,
			size_t,
			size_t *,
			struct timeval * );

// read device function
static int read_device(
			PDEV_INFO,
			unsigned char *,
			size_t,
			size_t *,
			struct timeval * );

// reset device function
static int reset_device_usb( PDEV_INFO );
static int reset_device_ser( PDEV_INFO );
static int reset_device_lan( PDEV_INFO );
static int ( *reset_device[] )( PDEV_INFO ) =
{
	reset_device_usb,
	reset_device_ser,
	reset_device_lan
};

// static function
static int _get_protocol_family( char * );
static int _open_device_lan(
			PDEV_INFO,
			int,
			int );






/*******************************************

Global function

********************************************/

/**
 * open device
 *
 * @param[ out ] hSiiDevice		handle to device object
 * @param[ in ] pszPath			device path or IP Address
 * @retval							return values of function result
 *
 */
int
sii_api_open_device(
	SIIAPIHANDLE		*hSiiDevice,
	char				*pszPath )
{
	PDEV_INFO			*phDevInfo = (PDEV_INFO*)hSiiDevice;

	if( is_null( hSiiDevice ) || is_null( pszPath ) )
	{
		return ( ERR_SII_API_NULL );
	}

	if( ( *phDevInfo = malloc( sizeof( DEV_INFO ) ) ) == NULL )
	{
		return ( ERR_SII_API_MALLOC );
	}

	memset( *phDevInfo, 0, sizeof( DEV_INFO ) );

	if( !strncmp( pszPath, PORT_USB, strlen( PORT_USB ) ) )
	{
		return open_device_usb( *phDevInfo, pszPath );
	}
	else if( !strncmp( pszPath, PORT_SER, strlen( PORT_SER ) ) )
	{
		return open_device_ser( *phDevInfo, pszPath );
	}
	else if( strchr( pszPath, PORT_LAN__IPV4_DELIMITER ) != NULL )
	{
		return open_device_lan( *phDevInfo, pszPath );
	}
	else
	{
		free( *phDevInfo );
		return ( ERR_SII_API_DIS_OPEN );
	}
}


/**
 * close device
 *
 * @param[ out ] hSiiDevice		handle to device object
 * @retval							return values of function result
 */
int
sii_api_close_device(
	SIIAPIHANDLE		hSiiDevice )
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;
	int					nBufSize=0;
	socklen_t			nSocLen = 1;
	char				*pbuffer;

	if( is_null( hSiiDevice ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	switch(phDevInfo->nDeviceType)
	{
		case DEV_SER:
			// Exclusive control OFF
			ioctl( phDevInfo->nDeviceFid, TIOCNXCL );
			break;
		case DEV_LAN:
			if( shutdown( phDevInfo->nDeviceFid, SHUT_RDWR ) !=0 )
			{
				return ( ERR_SII_API_DIS_CLOSE );
			}
			if( getsockopt(phDevInfo->nDeviceFid, SOL_SOCKET, SO_RCVBUF, &nBufSize, &nSocLen) !=0 )
			{
				return ( ERR_SII_API_DIS_CLOSE );
			}
			pbuffer = (char *)malloc(nBufSize);
			if(pbuffer == NULL)
			{
				return ( ERR_SII_API_DIS_CLOSE );
			}
			bzero( pbuffer, nBufSize );
			read(phDevInfo->nDeviceFid, pbuffer, nBufSize);
			if(pbuffer)
			{
				free(pbuffer);
				pbuffer = NULL;
			}
			break;
		default :
			break;
	}

	if( close( phDevInfo->nDeviceFid ) != 0 )
	{
		return ( ERR_SII_API_DIS_CLOSE );
	}

	phDevInfo->nDeviceType = DEV_END;
	free( phDevInfo );
	return ( SUCC_SII_API );
}

/**
 * write device
 *
 * @param[ in ] hSiiDevice		handle to device object
 * @param[ in ] pbyCmd  			array of printer data
 * @param[ in ] tagCmdSize		size of array
 * @param[ out ] pSize			bytes written to printer
 * @retval							return values of function result
 */
int
sii_api_write_device(
	SIIAPIHANDLE		hSiiDevice,
	unsigned char		*pbyCmd,
	size_t				tagCmdSize,
	size_t				*pSize)
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;
	unsigned char		byTmpBuf[512];
	int					nRetVal=0;

	if( is_null( hSiiDevice ) || is_null( pbyCmd ) || is_null( pSize ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	if( tagCmdSize <= 0 )
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	if( phDevInfo->pfnUsrDefFunc != NULL )
	{
		nRetVal = read_device(
				phDevInfo,
				byTmpBuf,
				sizeof(byTmpBuf),
				NULL,
				NULL );
	}

	if( nRetVal >= 0 )
	{
		switch(phDevInfo->nDeviceType)
		{
			case DEV_LAN:
				nRetVal = write_device_lan(
						phDevInfo,
						pbyCmd,
						tagCmdSize,
						pSize,
						NULL );
				break;
			default:
				nRetVal = write_device(
						phDevInfo,
						pbyCmd,
						tagCmdSize,
						pSize,
						NULL );
				break;
		}
	}

	return nRetVal;
}


/**
 * read device
 *
 * @param[ in ] hSiiDevice		handle to device object
 * @param[ in ] pbyResp			data buffer
 * @param[ in ] tagRespSize		size of data buffer
 * @param[ out ] pSize			bytes received
 * @retval							return values of function result
 */
int
sii_api_read_device(
	SIIAPIHANDLE		hSiiDevice,
	unsigned char		*pbyResp,
	size_t				tagRespSize,
	size_t				*pSize )
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;
	struct timeval	tagTimeout;

	if( is_null( hSiiDevice ) || is_null( pbyResp ) || is_null( pSize ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	if( tagRespSize <= 0 )
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	switch(phDevInfo->nDeviceType)
	{
		case DEV_LAN:
			tagTimeout.tv_sec = TIMEOUT_SEC_LAN;
			tagTimeout.tv_usec = TIMEOUT_USEC_LAN;
			break;
		default:
			tagTimeout.tv_sec = TIMEOUT_SEC;
			tagTimeout.tv_usec = TIMEOUT_USEC_LONG;
			break;
	}

	return read_device(
				phDevInfo,
				pbyResp,
				tagRespSize,
				pSize,
				&tagTimeout );
}

/**
 * get status
 *
 * @param[ in ] hSiiDevice		handle to device object
 * @param[ out ] pnResp			response data area
 * @retval							return values of function result
 *
 */
int
sii_api_get_status(
	SIIAPIHANDLE		hSiiDevice,
	unsigned int		*pnResp)
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;
	int					nRetVal;
	int					i;
	unsigned char		byStaCmd[] = { 0x1d, 0x61, 0x7F };
	unsigned char		byTmpBuf[512];
	struct timeval	tagTimeout;
	size_t				tagSize;

	if( is_null( hSiiDevice ) || is_null( pnResp ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	switch(phDevInfo->nDeviceType)
	{
		case DEV_LAN:
			tagTimeout.tv_sec = TIMEOUT_SEC_LAN;
			tagTimeout.tv_usec = TIMEOUT_USEC_LAN;

			nRetVal = write_device_lan(
					phDevInfo,
					(unsigned char*)byStaCmd,
					sizeof( byStaCmd ),
					NULL,
					&tagTimeout );
			break;
		default:
			tagTimeout.tv_sec = TIMEOUT_SEC;
			tagTimeout.tv_usec = TIMEOUT_USEC_LONG;

			nRetVal = write_device(
					phDevInfo,
					(unsigned char*)byStaCmd,
					sizeof( byStaCmd ),
					NULL,
					&tagTimeout );
			break;
	}
	if( nRetVal < 0 )
	{
		return nRetVal;
	}

	for( i = 0; i < GET_STATUS_RETRY_COUNT; i ++ )
	{

		nRetVal = read_device(
				phDevInfo,
				byTmpBuf,
				sizeof(byTmpBuf),
				&tagSize,
				&tagTimeout );

		if( nRetVal < 0 )
		{
			return nRetVal;
		}

		if( tagSize >= sizeof( unsigned int ) )
		{
			break;
		}
		usleep( GET_STATUS_WAIT_TIME );
	}

	if( ( i == GET_STATUS_RETRY_COUNT ) && ( tagSize == 0 ) )
	{
		return ( ERR_SII_API_TIMEOUT );
	}

	if( *(phDevInfo->byAutoStatusBuf) == 0x00 )
	{
		return ( ERR_SII_API_NO_DATA );
	}

	*pnResp = 0;
	for( i = 0; i < AUTO_STATUS_DATA_SIZE; i ++ )
	{
		*pnResp |= ( phDevInfo->byAutoStatusBuf[i] & 0x0F ) << (i * 4);
	}

	return ( SUCC_SII_API );
}



/**
 * reset device
 *
 * @param[ in ] hSiiDevice		handle to device object
 * @retval							return values of function result
 */
int
sii_api_reset_device(
	SIIAPIHANDLE		hSiiDevice )
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;

	if( is_null( hSiiDevice ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	return reset_device[ phDevInfo->nDeviceType ]( phDevInfo );
}


/**
 * set call back func for printer status
 *
 * @param[ in ] hSiiDevice		handle to device object
 * @param[ in ] pFunc			pointer of call back func on user application
 * @retval							return values of function result
 */
int
sii_api_set_callback_func(
	SIIAPIHANDLE		hSiiDevice,
	CALLBACK_FUNC		pFunc )
{
	PDEV_INFO			phDevInfo = (PDEV_INFO)hSiiDevice;
	unsigned int		nResp;
	int					nRetVal;

	if( is_null( hSiiDevice ) || is_null( pFunc ) )
	{
		return ( ERR_SII_API_NULL );
	}
	if(phDevInfo->nDeviceType >= DEV_END)
	{
		return ( ERR_SII_API_RANGE );
	}
	else if(phDevInfo->nDeviceType < 0)
	{
		return ( ERR_SII_API_RANGE );
	}
	if(phDevInfo->nDeviceFid == 0)
	{
		return ( ERR_SII_API_RANGE );
	}

	nRetVal = sii_api_get_status( hSiiDevice, &nResp );
	if( nRetVal >= 0 )
	{
		phDevInfo->pfnUsrDefFunc = pFunc;
	}

	return nRetVal;
}






/*******************************************

Internal function

********************************************/



/**
 * open USB device interface
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pszPath			device path
 * @retval							return values of function result
 *
*/
int
open_device_usb(
	PDEV_INFO			phDevInfo,
	char				*pszPath )
{
	int					nFd;
	char				pszDeviceId[1024];

	if( ( nFd = open( pszPath, O_RDWR | O_EXCL ) ) < 0 )
	{
		OPEN_ERR();
	}

	memset( pszDeviceId, 0, sizeof(pszDeviceId) );

	// get printer device id
	if( ioctl( nFd, _IOC(_IOC_READ, 'P', 1, sizeof(pszDeviceId) ), pszDeviceId ) )
	{
		OPEN_ERR();
	}

	// check Vendor
	if( strstr( pszDeviceId+2, "MFG:SII;" ) == 0 )
	{
		OPEN_ERR();
	}


	phDevInfo->nDeviceType	= DEV_USB;
	phDevInfo->nDeviceFid	= nFd;

	return ( SUCC_SII_API );
}

/**
 * open Serial device interface
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pszPath			device path
 * @retval							return values of function result
 *
*/
int
open_device_ser(
	PDEV_INFO			phDevInfo,
	char				*pszPath )
{
	int						nFd;
	struct termios		tagOpts;
	struct termios		tagOrigOpts;

	if( ( nFd = open( pszPath, O_RDWR | O_NOCTTY | O_EXCL | O_NONBLOCK ) ) < 0 )
	{
		OPEN_ERR();
	}

	phDevInfo->nDeviceType	= DEV_SER;
	phDevInfo->nDeviceFid	= nFd;
	tcgetattr( nFd, &tagOpts );
	tcgetattr( nFd, &tagOrigOpts );
	tagOpts.c_lflag &= ~( ICANON | ECHO | ISIG );
	tcsetattr( nFd, TCSANOW, &tagOpts );

	// Exclusive control ON
	if( ioctl( nFd, TIOCEXCL ) != 0 )
	{
		OPEN_ERR();
	}

	return ( SUCC_SII_API );
}


/**
 * open LAN device interface
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pszPath			IP Address
 * @retval							return values of function result
 *
 */
int
open_device_lan(
	PDEV_INFO			phDevInfo,
	char				*pszPath )
{
	int					nFd;
	int					nFamily;

	nFamily = _get_protocol_family( pszPath );
	if( nFamily < 0 )
	{
		OPEN_ERR();
	}

	strcpy( phDevInfo->szIPAddress, pszPath );

	nFd = _open_device_lan( phDevInfo, nFamily, TCP_PORT_NUMBER );
	if( nFd < 0 )
	{
		OPEN_ERR();
	}

	phDevInfo->nDeviceType = DEV_LAN;
	phDevInfo->nDeviceFid = nFd;

	return  ( SUCC_SII_API );
}

/**
 * write device
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pbyCmd  			array of printer data
 * @param[ in ] tagCmdSize		size of array
 * @param[ out ] pSize			bytes written to printer
 * @param[ in ] pTimeout			writing timeout value
 * @retval							return values of function result
 *
 */
static int
write_device(
	PDEV_INFO			phDevInfo,
	unsigned char		*pbyCmd,
	size_t				tagCmdSize,
	size_t				*pSize,
	struct timeval	*pTimeout )
{
	int						nDeviceFid;
	int						nRetVal = 0;
	size_t					tagWriteBytes = 0;
	fd_set					tagOutput;
	struct timeval		tagTimeout;
	int						nRetBytes = 0;
	struct timespec		tagTimespec;
	struct aiocb			tagAioWrite;
	const struct aiocb 	*plistAiocb[1];

	if( pSize != NULL )
	{
		*pSize = 0;
	}

	if( pTimeout != NULL )
	{
		memcpy( &tagTimeout, pTimeout, sizeof( struct timeval ) );
	}
	else
	{
		tagTimeout.tv_sec = WRITE_TIMEOUT_SEC;
		tagTimeout.tv_usec = WRITE_TIMEOUT_USEC;
	}

	if( tagCmdSize < MAX_WRITE_BYTES )
	{
		tagWriteBytes = tagCmdSize;
	}
	else
	{
		tagWriteBytes = MAX_WRITE_BYTES;
	}

	nDeviceFid = phDevInfo->nDeviceFid;
	FD_ZERO( &tagOutput );
	FD_SET( nDeviceFid, &tagOutput );
	nRetVal = select(
				nDeviceFid + 1,
				NULL,
				&tagOutput,
				NULL,
				&tagTimeout );
	if( nRetVal < 0 )
	{
		return ( ERR_SII_API_SELECT );
	}
	else if( nRetVal == 0 )
	{
		return ( ERR_SII_API_TIMEOUT );
	}

	if( FD_ISSET( nDeviceFid, &tagOutput ) )
	{
		bzero( &tagAioWrite, sizeof( struct aiocb ) );
		tagAioWrite.aio_fildes = nDeviceFid;
		tagAioWrite.aio_buf    = pbyCmd;
		tagAioWrite.aio_nbytes = tagWriteBytes;
		tagAioWrite.aio_sigevent.sigev_notify = SIGEV_NONE;
		plistAiocb[0] = &tagAioWrite;

		tagTimespec.tv_sec = tagTimeout.tv_sec;
		tagTimespec.tv_nsec = tagTimeout.tv_usec;

		nRetVal = aio_write( &tagAioWrite );
		if( nRetVal != 0 )
		{
			return ( ERR_SII_API_DEV_ACCESS );
		}

		do
		{
			nRetVal = aio_suspend( plistAiocb, 1, &tagTimespec );
			if( nRetVal != 0 )
			{
				if( errno == EAGAIN )
				{
					nRetBytes = aio_return( &tagAioWrite );
					if( nRetBytes < 0 )
					{
						return ( ERR_SII_API_DEV_ACCESS );
					}
					if( pSize != NULL )
					{
						*pSize = nRetBytes;
					}
					return ( ERR_SII_API_TIMEOUT );
				}
			}
			nRetVal = aio_error( &tagAioWrite );
		}while( nRetVal == EINPROGRESS );
		nRetBytes = aio_return( &tagAioWrite );
		if( nRetBytes < 0 )
		{
			return ( ERR_SII_API_DEV_ACCESS );
		}
		if( pSize != NULL )
		{
			*pSize = nRetBytes;
		}
	}
	else
	{
		return ( ERR_SII_API_DEV_ACCESS );
	}

	return ( SUCC_SII_API );
}


/**
 * write device_lan
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pbyCmd  			array of printer data
 * @param[ in ] tagCmdSize		size of array
 * @param[ out ] pSize			bytes written to printer
 * @param[ in ] pTimeout			writing timeout value
 * @retval							return values of function result
 *
 */
static int
write_device_lan(
	PDEV_INFO			phDevInfo,
	unsigned char		*pbyCmd,
	size_t				tagCmdSize,
	size_t				*pSize,
	struct timeval	*pTimeout )
{
	int						nDeviceFid;
	int						nRetVal = 0;
	size_t					tagWriteBytes = 0;
	fd_set					tagOutput;
	struct timeval		tagTimeout;
	int						nRetBytes = 0;

	if( pSize != NULL )
	{
		*pSize = 0;
	}

	if( pTimeout != NULL )
	{
		memcpy( &tagTimeout, pTimeout, sizeof( struct timeval ) );
	}
	else
	{
		tagTimeout.tv_sec = WRITE_TIMEOUT_SEC;
		tagTimeout.tv_usec = WRITE_TIMEOUT_USEC;
	}

	if( tagCmdSize < MAX_WRITE_BYTES )
	{
		tagWriteBytes = tagCmdSize;
	}
	else
	{
		tagWriteBytes = MAX_WRITE_BYTES;
	}

	nDeviceFid = phDevInfo->nDeviceFid;
	FD_ZERO( &tagOutput );
	FD_SET( nDeviceFid, &tagOutput );
	nRetVal = select(
				nDeviceFid + 1,
				NULL,
				&tagOutput,
				NULL,
				&tagTimeout );
	if( nRetVal < 0 )
	{
		return ( ERR_SII_API_SELECT );
	}
	else if( nRetVal == 0 )
	{
		return ( ERR_SII_API_TIMEOUT );
	}

	if( FD_ISSET( nDeviceFid, &tagOutput ) )
	{
		if( setsockopt( nDeviceFid, SOL_SOCKET, SO_SNDTIMEO, &tagTimeout, sizeof( struct timeval ) ) != 0 )
		{
			return ( ERR_SII_API_DEV_ACCESS );
		}
		nRetBytes = send( nDeviceFid, pbyCmd, tagWriteBytes, 0 );
		if( nRetBytes < 0 )
		{
			if( errno == EAGAIN || errno == EWOULDBLOCK )
			{
				return ( ERR_SII_API_TIMEOUT );
			}
			else
			{
				return ( ERR_SII_API_DEV_ACCESS );
			}
		}
		if( pSize != NULL )
		{
			*pSize = nRetBytes;
		}
	}
	else
	{
		return ( ERR_SII_API_DEV_ACCESS );
	}
	return ( SUCC_SII_API );
}


/**
 * read device and analyze rec data
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] pbyResp			data buffer
 * @param[ in ] tagRespSize		size of data buffer
 * @param[ out ] pSize			bytes received
 * @retval							return values of function result
 *
 */
static int
read_device(
	PDEV_INFO			phDevInfo,
	unsigned char		*pbyResp,
	size_t				tagRespSize,
	size_t				*pSize,
	struct timeval	*pTimeout )
{
	unsigned char		byLastAutoStatus[ AUTO_STATUS_DATA_SIZE ];
	unsigned char		pbyTmpBuf[ 512 ];
	int					nSelRet = 0;
	int					nDeviceFid = 0;
	int					nEndFlag = FALSE;
	int					nIndex;
	int					i;
	fd_set				tagInput;
	ssize_t			tagBytes = 0;
	size_t				tagReadBytes;
	size_t				tagSize = 0;
	struct timeval	tagTimeout;
	struct timeval	tagCurrentTime;
	struct timeval	tagStartTime;
	struct timeval	tagPassTime;

	if( tagRespSize > sizeof( pbyTmpBuf ) )
	{
		tagReadBytes = sizeof( pbyTmpBuf );
	}
	else if( tagRespSize > 0 )
	{
		tagReadBytes = tagRespSize;
	}
	else
	{
		return ( ERR_SII_API_RANGE );
	}

	memset( pbyResp, 0, tagRespSize );

	if( pSize != NULL )
	{
		*pSize = 0;
	}

	memcpy( byLastAutoStatus, phDevInfo->byAutoStatusBuf, AUTO_STATUS_DATA_SIZE );

	if( pTimeout != NULL )
	{
		memcpy( &tagTimeout, pTimeout, sizeof( struct timeval ) );
	}
	else
	{
		switch(phDevInfo->nDeviceType)
		{
			case DEV_LAN:
				tagTimeout.tv_sec = TIMEOUT_SEC_LAN;
				tagTimeout.tv_usec = TIMEOUT_USEC_LAN;
				break;
			default:
				tagTimeout.tv_sec = TIMEOUT_SEC;
				tagTimeout.tv_usec = TIMEOUT_USEC_SHRT;
				break;
		}
	}
	nDeviceFid = phDevInfo->nDeviceFid;

	// get start time ( for timeout )
	gettimeofday( &tagStartTime, NULL );

	// read data
	for( ;; )
	{
		FD_ZERO( &tagInput );
		FD_SET( nDeviceFid, &tagInput );
		nSelRet = select(
					nDeviceFid + 1,
					&tagInput,
					NULL,
					NULL,
					&tagTimeout );
		if( nSelRet < 0 )
		{
			return ( ERR_SII_API_SELECT );
		}
		else if( nSelRet == 0 )
		{
			break;
		}

		if( FD_ISSET( nDeviceFid, &tagInput ) )
		{
			memset( pbyTmpBuf, 0, sizeof( pbyTmpBuf ) );
			tagBytes = read(
						nDeviceFid,
						pbyTmpBuf,
						tagReadBytes );
			if( tagBytes < 0 )
			{
				if( errno == EIO )
				{
					return ( ERR_SII_API_DEV_ACCESS );
				}
				tagBytes = 0;
			}

			for( nIndex = 0; nIndex < tagBytes; nIndex++ )
			{
				if( ( pbyTmpBuf[ nIndex ] & AUTO_STATUS_HEAD_MASK ) == AUTO_STATUS_HEAD_DATA )
				{
					phDevInfo->byAutoStatusIndex = 0;
					phDevInfo->byAutoStatusBuf[ phDevInfo->byAutoStatusIndex ] = pbyTmpBuf[ nIndex ];
				}
				else if(
					( phDevInfo->byAutoStatusIndex < AUTO_STATUS_DATA_SIZE -1 )	&&
					( ( pbyTmpBuf[ nIndex ] & AUTO_STATUS_OTHER_MASK ) == AUTO_STATUS_OTHER_DATA ) )
				{
					phDevInfo->byAutoStatusIndex++;
					phDevInfo->byAutoStatusBuf[ phDevInfo->byAutoStatusIndex ] = pbyTmpBuf[ nIndex ];

					if( phDevInfo->byAutoStatusIndex == ( AUTO_STATUS_DATA_SIZE -1 ) )
					{
						// check auto status response
						phDevInfo->byAutoStatusIndex++;
						if(
							( phDevInfo->pfnUsrDefFunc != NULL)	&&
							( memcmp(
								phDevInfo->byAutoStatusBuf,
								byLastAutoStatus,
								AUTO_STATUS_DATA_SIZE ) != 0 ) )
						{
							unsigned int nResp = 0;
							memcpy( byLastAutoStatus, phDevInfo->byAutoStatusBuf, AUTO_STATUS_DATA_SIZE );
							for( i = 0; i < AUTO_STATUS_DATA_SIZE; i ++ )
							{
								nResp |= ( phDevInfo->byAutoStatusBuf[i] & 0x0F ) << (i * 4); 
							}

							phDevInfo->pfnUsrDefFunc( (unsigned int)( nResp ) );	//call back status
						}
					}
				}

				nEndFlag = TRUE;
				if( ( pbyTmpBuf[ nIndex ] != 0x11 )		&&
					( pbyTmpBuf[ nIndex ] != 0x13 ) )
				{
					if( tagReadBytes != 0 )
					{
						pbyResp[ tagSize++ ] = pbyTmpBuf[ nIndex ];
						if( tagReadBytes-- > 0 )
						{
							nEndFlag = FALSE;
						}
					}
				}
			}
		}

		// check timeout
		gettimeofday( &tagCurrentTime, NULL );
		timersub( &tagCurrentTime, &tagStartTime, &tagPassTime );
		if( timercmp( &tagTimeout, &tagPassTime, < ) )
		{
			break;					// Timeout
		}

		if( ( tagBytes == 0 ) && ( tagSize != 0 ) )
		{
			break;					// End of data
		}

		if( nEndFlag == TRUE )
		{
			break;					// End flag was set
		}
	}

	if( pSize != NULL )
	{
		*pSize = tagSize;
	}

	return ( SUCC_SII_API );
}




// reset device
/**
 * reset USB device
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @retval							return values of function result
 *
 */
static int
reset_device_usb(
	PDEV_INFO			phDevInfo )
{
	if( ioctl( phDevInfo->nDeviceFid, _IOC( _IOC_NONE, 'P', 7, 0 ), 0 ) < 0 )
	{
		return ( ERR_SII_API_RESET );
	}

	return( SUCC_SII_API );
}

/**
 * reset Serial device
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @retval							return values of function result
 *
 */
static int
reset_device_ser(
	PDEV_INFO			phDevInfo )
{
	if( tcsendbreak( phDevInfo->nDeviceFid, 0 ) < 0 )
	{
		return ( ERR_SII_API_RESET );
	}

	return ( SUCC_SII_API );
}

/**
 * reset LAN device
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @retval							return values of function result
 *
 */
static int
reset_device_lan(
	PDEV_INFO			phDevInfo )
{
	int					nRetVal;
	int					nFd;
	int					nFdtmp;
	int					nFamily;
	int					nBufSize=0;
	socklen_t			nSocLen = 1;
	char				*pbuffer;

#define RESET_COMMAND_LENGTH	13
	unsigned char		pCmdBuf[] = { 'S', 'I', 'I', 0, 0, 0xC1, 0x40, 0, 0, 0, 0, 8, 0 };
	size_t				tagReadBytes;

	nFamily = _get_protocol_family( phDevInfo->szIPAddress );
	if( nFamily < 0 )
	{
		return ( ERR_SII_API_RESET );
	}

	nFd = _open_device_lan( phDevInfo, nFamily, SPECIAL_PORT_NUMBER );
	if( nFd < 0 )
	{
		return ( ERR_SII_API_RESET );
	}
	nFdtmp = phDevInfo->nDeviceFid;
	phDevInfo->nDeviceFid = nFd;

	// send command
	nRetVal = write_device(
		phDevInfo,
		pCmdBuf,
		RESET_COMMAND_LENGTH,
		&tagReadBytes,
		NULL );
	shutdown( nFd, SHUT_RDWR );
	close( nFd );
	if( nRetVal < 0 )
	{
		return ( ERR_SII_API_RESET );
	}
	phDevInfo->nDeviceFid = nFdtmp;


	if( shutdown( phDevInfo->nDeviceFid, SHUT_RD ) !=0 )
	{
		return ( ERR_SII_API_RESET );
	}
	if( getsockopt(phDevInfo->nDeviceFid, SOL_SOCKET, SO_RCVBUF, &nBufSize, &nSocLen) !=0 )
	{
		return ( ERR_SII_API_RESET );
	}
	pbuffer = (char *)malloc(nBufSize);
	if(pbuffer == NULL)
	{
		return ( ERR_SII_API_MALLOC );
	}
	bzero( pbuffer, nBufSize );
	read(phDevInfo->nDeviceFid, pbuffer, nBufSize);
	if(pbuffer)
	{
		free(pbuffer);
		pbuffer = 0;
	}
	if( close( phDevInfo->nDeviceFid ) < 0 )
	{
		return ( ERR_SII_API_RESET );
	}
	sleep(3);

	nFamily = _get_protocol_family( phDevInfo->szIPAddress );
	if( nFamily < 0 )
	{
		return ( ERR_SII_API_RESET );
	}

	nFd = _open_device_lan( phDevInfo, nFamily, TCP_PORT_NUMBER );
	if( nFd < 0 )
	{
		return ( ERR_SII_API_RESET );
	}

	phDevInfo->nDeviceFid = nFd;

	return ( SUCC_SII_API );
}

/**
 *
 *
 * @param[ in ] pszIPAddress	pointer of IP Address (String)
 * @retval							protocol family (domain)
 *									Fail : -1
 *									Success : 	AF_INET
 *
 */
static int
_get_protocol_family(
	char *pszIPAddress )
{
	unsigned char buf[sizeof(struct in_addr)];
	int		nRetVal;

	nRetVal = inet_pton( AF_INET, pszIPAddress, buf );
	if( nRetVal == 1 )
	{
		nRetVal = AF_INET;
	}
	else
	{
		nRetVal = -1;
	}

	return nRetVal;
}

/**
 *
 *
 * @param[ in ] phDevInfo		pointer of device info
 * @param[ in ] nFamily			LAN of family
 * @param[ in ] portNumber		LAN of port number
 * @retval							Fail : -1
 *									Success : file descriptor
 *
 */
static int _open_device_lan(
	PDEV_INFO			phDevInfo,
	int					nFamily,
	int 				nPortNumber )
{
	int					nSocketType, nRetVal, soc=0;
	char				szPortString[8];
	struct addrinfo	hints;
	struct addrinfo	*pres;

	fd_set				tagInput;
	struct timeval	tagTimeout;

	socklen_t			nGetErrorSize = sizeof( nRetVal );

	if( nPortNumber == TCP_PORT_NUMBER || nPortNumber == SPECIAL_PORT_NUMBER )
	{
		nSocketType = SOCK_STREAM;
	}
	else
	{
		return -1;		// port number Error
	}

	memset( &hints, 0, sizeof( struct addrinfo ) );
	hints.ai_family = nFamily;
	hints.ai_socktype = nSocketType;
	hints.ai_flags = AI_NUMERICHOST;

	sprintf( szPortString, "%d", nPortNumber );

	// network information
	nRetVal = getaddrinfo( phDevInfo->szIPAddress,
						   szPortString, &hints, &pres );
	if( nRetVal != 0 )
	{
		return -1;			// getaddrinfo Error
	}

	// socket
	soc = socket( pres->ai_family, pres->ai_socktype, pres->ai_protocol );
	if( soc < 0 )
	{
		freeaddrinfo( pres );
		return -1;			// Socket Error
	}

	nRetVal = connect( soc, pres->ai_addr, pres->ai_addrlen );
	if( nRetVal != 0 )
	{
		if( errno == EINPROGRESS )
		{
			FD_ZERO( &tagInput );
			FD_SET( soc, &tagInput );

			tagTimeout.tv_sec = TIMEOUT_SEC_LAN;
			tagTimeout.tv_usec = TIMEOUT_USEC_LAN;

			nRetVal = select( soc+1, NULL, &tagInput, NULL, &tagTimeout );
			if( nRetVal <= 0 )
			{
				if( nRetVal == 0 )
				{
					errno = EBUSY;
				}
				nRetVal = -1;
			}
			if( getsockopt( soc, SOL_SOCKET, SO_ERROR, &nRetVal, &nGetErrorSize ) != 0 )
			{
				nRetVal = -1;
			}
			if( nRetVal != 0 )
			{
				if( nRetVal == EISCONN )
				{
					nRetVal = 0;
				}
				else
				{
					nRetVal = -1;
				}
			}
		}
		else
		{
			nRetVal = -1;
		}
	}

	freeaddrinfo( pres );

	if( nRetVal < 0 )
	{
		close( soc );
		return nRetVal;
	}

	return soc;
}

