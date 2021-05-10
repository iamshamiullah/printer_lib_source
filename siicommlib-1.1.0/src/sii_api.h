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

#include <stdlib.h>


// function return value 
//	[ function return value ]
/*	
 *	List of function returned values
 *	The following describes function returned values: 
 */
#define SUCC_SII_API						0		/* Function succeeded		*/
#define ERR_SII_API_NULL					-1		/* Null argument				*/
#define ERR_SII_API_RANGE				-2		/* Argument is out of range	*/
#define ERR_SII_API_DEV_ACCESS			-3		/* Device access error		*/
#define ERR_SII_API_TIMEOUT				-4		/* Timeout error				*/
#define ERR_SII_API_NO_DATA				-5		/* Read no data				*/
#define ERR_SII_API_MALLOC				-6		/* Memory allocation error	*/
#define ERR_SII_API_SELECT				-7		/* Device select error		*/
#define	ERR_SII_API_DIS_OPEN			-11		/* Device open error			*/
#define ERR_SII_API_DEV_EBUSY			-12		/* Device busy				*/
#define ERR_SII_API_DEV_EACCES			-13		/* Unauthorized access mode	*/	
#define ERR_SII_API_DEV_ENXIO			-14		/* Invalid device				*/
#define	ERR_SII_API_DIS_CLOSE		-15		/* Device close error		*/
#define	ERR_SII_API_RESET				-16		/* Device reset error		*/

#define	SIIAPIHANDLE				void *


//	[ open device ]
int sii_api_open_device(
	SIIAPIHANDLE*,			/* [out] pointer of handle to device object */
	char *);					/* [out] pointer of device path name */

//	[ close device ]
int sii_api_close_device(
	SIIAPIHANDLE);			/* [in] handle to device object */

//	[ output to device ]
int sii_api_write_device(
	SIIAPIHANDLE,				/* [in] handle to device object */
	unsigned char *,			/* [out] array of printer data */
	size_t,					/* [in] size of array */
	size_t *);					/* [out] pointer to the variable that receives the number of bytes written */

//	[ input from device ]
int sii_api_read_device(
	SIIAPIHANDLE,				/* [in] handle to device object */
	unsigned char *,			/* [out] data buffer */
	size_t,					/* [in] size of data buffer */
	size_t *);					/* [out] pointer to the variable that receives the number of bytes received */
 
//	[ input status from device ]
int sii_api_get_status(
	SIIAPIHANDLE,				/* [in] handle to device object */
	unsigned int *);			/* [out] status data buffer */

//	[reset to device ]
int sii_api_reset_device(
	SIIAPIHANDLE);			/* [in] handle to device object */

//	[ set call back function ]
typedef void (*CALLBACK_FUNC)(unsigned int);
int sii_api_set_callback_func(
	SIIAPIHANDLE,				/* [in] handle to device object */
	CALLBACK_FUNC);			/* [out] pointer of callback function */

