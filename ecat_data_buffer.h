/*******************************************************************
 *
 * ecat_data_buffer.h  -  EtherCAT data buffer header file (ring buffer)
 *
 *
 * DESCRIPTION:
 *
 *  The module provide access write much more group reference and feedback data.
 *
 * HISTORY:
 *
 *  When			Who		What
 *   2024.06.08		Wei	Initial version
 *
 */
#ifndef __ECAT_DATA_BUFFER_H__
#define __ECAT_DATA_BUFFER_H__

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#define MAX_MOTORS_NUM 30		  /*Max motors*/
#define MAX_REFERNECE_GROUPS 2000 /*Max group motor references*/

typedef struct
{
	int ctrl_word;		/*CiA402 control word*/
	int operation_mode; /*CiA402 operation mode*/
	int target_postion; /*motor target position*/
	int target_speed;	/*motor target speed*/
	int target_torque;	/*motor target torque*/
	int speed_offset;	/*motor speed offset*/
	int torque_offset;	/*motor torque offset*/
} MOTOR_REFERENCE;

typedef struct
{
	int status_word;	/*CiA402 status word*/
	int feedbk_postion; /*Feed back position*/
	int feedbk_speed;	/*Feed back speed*/
	int feedbk_torque;	/*Feed back torque*/
	int target_position;
} MOTOR_FEEDBACK;

typedef struct
{
	MOTOR_REFERENCE motor_ref[MAX_MOTORS_NUM]; /*Reference data for all motor*/
} GROUP_REFERENCE;

typedef struct
{
	int read_pointer;		/*RING BUFFER: read pointer*/
	int write_pointer;		/*RING BUFFER: Write pointer*/
	int max_reference_grps; /*RING BUFFER: Size*/

	GROUP_REFERENCE grp_ref[MAX_REFERNECE_GROUPS]; /*All group reference*/

} ALL_REFERENCE;

typedef struct
{
	MOTOR_FEEDBACK motor_fdbk[MAX_MOTORS_NUM]; /*Feedback data for all motor*/
} GROUP_FEEDBACK;

typedef struct
{
	int struct_size; /*Total size for struct ECAT_BUF_SHM_STRU*/
	int magic_no;	 /*Magic number*/
	int version;	 /*Protocol version*/
	int inited_done; /*Inited done*/

	ALL_REFERENCE reference; /*All motors reference in ring buffer*/
	GROUP_FEEDBACK feedback; /*All motors feedback*/

} ECAT_BUF_SHM_STRU;

/*Initialize share memory*/
bool edb_init(void *buf_pointer, int buf_size, bool init_buf);

/*Push/write refernce*/
bool edb_push_ref(GROUP_REFERENCE *p_grp_ref);

/*Pull/read refernce*/
bool edb_pull_ref(GROUP_REFERENCE *p_grp_ref);

/*Push/write feedback*/
bool edb_push_fdbk(GROUP_FEEDBACK *p_grp_fdbk);

/*Pull/read feedback*/
bool edb_pull_fdbk(GROUP_FEEDBACK *p_grp_fdbk);

#endif