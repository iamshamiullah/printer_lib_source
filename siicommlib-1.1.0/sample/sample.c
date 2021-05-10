#include <stdio.h>
#include <string.h>
#include <sii_api.h>
#include <unistd.h>

// Call back function for checking status.
void callback_samp_func( unsigned int sta )
{
	printf( "Call back status : %08x\n",sta );
}


int
main( int argc,char **argv )
{
	SIIAPIHANDLE	hSiiApiHandle = NULL;
	int				nIndex;
	int				nSize;
	unsigned int	nStaData = 0;
	unsigned char	pbyRecBuf[512];
	unsigned char	pbyCmdBuf[256];
	int				nRetVal;
	size_t			tagRetSize;

	if( argc != 3 )
	{	//	argument error
		fprintf( stderr,"Argument error\n\n" );
		fprintf( stderr,"Sample [mode] [device path]\n\n" );
		fprintf( stderr,"mode 0 : Get status test\n" );
		fprintf( stderr,"mode 1 : Print & read response test\n" );
		fprintf( stderr,"mode 2 : Reset device test\n" );
		fprintf( stderr,"mode 3 : Call back func test\n" );
		fprintf( stderr,"device path : ex. /dev/ttyS0( COM ) /dev/usb/lp0( USB ) /dev/lp0( LTP )\n" );
		exit ( -1 );
	}

	if( ( nRetVal = sii_api_open_device( &hSiiApiHandle, argv[2] ) ) < 0 )
	{	// open error
		printf( "ERROR : device path: %s\n", argv[2] );
		exit ( -1 );
	}

	switch( atoi( argv[1] ) )
	{
	case 0:
		printf( "Get status test\n" );
		nRetVal = sii_api_get_status( hSiiApiHandle, &nStaData );
		if( nRetVal < 0 )
			printf( "ERROR : sii_api_get_status %d\n",nRetVal );
		else
			printf( "status : [0x%08x]\n", nStaData );
		break;

	case 1:
		printf( "Print & read response test\n");
		for( nSize = 0, nIndex = 0x20; nIndex < 0xFF; nIndex++ )
		{
			pbyCmdBuf[nSize++] = nIndex;
		}

		pbyCmdBuf[nSize++] = 0x0a;
		pbyCmdBuf[nSize++] = 0x12;
		pbyCmdBuf[nSize++] = 'q';
		pbyCmdBuf[nSize++] = 0x00;

		nRetVal = sii_api_write_device(
								hSiiApiHandle,
								pbyCmdBuf,
								nSize,
								&tagRetSize );
		if( nRetVal < 0 )
		{
			printf( "ERROR : sii_api_write_device %d\n",nRetVal );
		}
		else
		{
			sleep( 1 );
			nRetVal = sii_api_read_device(
									hSiiApiHandle,
									pbyRecBuf,
									sizeof( pbyRecBuf ),
									&tagRetSize );
			if( nRetVal < 0 )
			{
				printf( "ERROR : sii_api_read_device %d\n",nRetVal );
			}
			else
			{
				printf( "Received data : [%zd byte]\n",tagRetSize );
				for( nIndex = 0; nIndex < tagRetSize; nIndex++ )
				{
					printf( "[0x%02x] ", pbyRecBuf[nIndex] );
				}
			}
			printf( "\n" );
		}
		break;

	case 2:
		printf( "Reset device test\n");
		nRetVal = sii_api_reset_device( hSiiApiHandle );
		if( nRetVal < 0 )
			printf( "ERROR : sii_api_reset_device %d\n",nRetVal );
		break;

	case 3:
		printf( "Call back func test\n" );
		nRetVal = sii_api_set_callback_func( hSiiApiHandle, callback_samp_func );
		if( nRetVal < 0 )
		{
			printf( "ERROR : sii_api_set_callback_func %d\n",nRetVal );
		}
		else
		{
			printf( "Watching status for 15 seconds.\nPlease push feed-button to change status !!\n\n" );
			memset( pbyRecBuf, 0, sizeof( pbyRecBuf ) );
			for( nIndex = 0; nIndex < 15; nIndex++ )
			{
				sleep( 1 );
				nRetVal = sii_api_read_device(
										hSiiApiHandle,
										pbyRecBuf,
										sizeof( pbyRecBuf ),
										&tagRetSize );
			}
			printf( "Watching was done.\n" );
		}
		break;

	default:
		printf( "This mode number is wrong.\n" );
		break;
	}

	sii_api_close_device( hSiiApiHandle );
}
