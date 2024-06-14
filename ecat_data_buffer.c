/*******************************************************************
 *
 * ecat_data_buffer.c  -  EtherCAT data buffer implementation (ring buffer)
 *
 *
 * DESCRIPTION:
 *
 *  The module provide access write much more group reference and feedback data.
 *
 * HISTORY:
 *
 *  When			Who		What
 *   2024.06.08		Wei 	Initial version
 * 
 */

 #include "ecat_data_buffer.h"
 
 /*Define ecat buffer shared memory pointer*/
 static ECAT_BUF_SHM_STRU* ecat_shm_pointer=0;
 
 /*Initialize share memory*/
 bool edb_init(void* buf_pointer, int buf_size, bool init_buf)
 {
	 if(0==buf_pointer)
	 {
		 /*NULL pointer*/
		 printf("edb_init: null pointer!\n");
		 return false;
	 }
	 if(buf_size< sizeof(ECAT_BUF_SHM_STRU))
	 {
		 /*Buffer size is not enough*/
		 printf("edb_init: buf_size=%d not enough, expect %ld !\n", buf_size, sizeof(ECAT_BUF_SHM_STRU));
		 return false;
	 }
	 ecat_shm_pointer = (ECAT_BUF_SHM_STRU*)buf_pointer;	

	/*Need initialize buffer*/
	if(true==init_buf)
	{
		 memset((void*)ecat_shm_pointer, 0, sizeof(ECAT_BUF_SHM_STRU));	 
		 
		 ecat_shm_pointer->reference.max_reference_grps=MAX_REFERNECE_GROUPS;
		 
		 ecat_shm_pointer->struct_size=sizeof(ECAT_BUF_SHM_STRU);
		 ecat_shm_pointer->magic_no=0x12345678;
		 ecat_shm_pointer->version=0x0100;
		 ecat_shm_pointer->inited_done=1;	 
	}
	
	return true;
 }
 
 /*Push/write refernce*/
 bool edb_push_ref(GROUP_REFERENCE* p_grp_ref)
 {
	 int rd_ptr=0, wr_ptr=0, total_grp=0;
	 if(0==ecat_shm_pointer || 0==p_grp_ref)
	 {
		 /*Null ecat pointer */
		 return false;
	 }
	 if(1!=ecat_shm_pointer->inited_done)
	 {
		 /*Ecat buffer not inited*/
		 return false;
	 }
	 total_grp=ecat_shm_pointer->reference.max_reference_grps;
	 wr_ptr=(ecat_shm_pointer->reference.write_pointer)%total_grp;
	 rd_ptr=ecat_shm_pointer->reference.read_pointer%total_grp;
	 
	 if(((wr_ptr+1)%total_grp)==rd_ptr)
	 {
		 /*Buffer is full*/
		 return false;
	 }
	 
	 ecat_shm_pointer->reference.grp_ref[wr_ptr]=*p_grp_ref;
	 ecat_shm_pointer->reference.write_pointer=(wr_ptr+1)%total_grp;
	 return true;	 
 }
 
  /*Pull/read refernce*/
 bool edb_pull_ref(GROUP_REFERENCE* p_grp_ref)
 {
	int rd_ptr=0, wr_ptr=0, total_grp=0;
	 if(0==ecat_shm_pointer || 0==p_grp_ref)
	 {
		 /*Null ecat pointer */
		 return false;
	 }
	 if(1!=ecat_shm_pointer->inited_done)
	 {
		 /*Ecat buffer not inited*/
		 return false;
	 }
	 
	 total_grp=ecat_shm_pointer->reference.max_reference_grps;
	 wr_ptr=(ecat_shm_pointer->reference.write_pointer)%total_grp;
	 rd_ptr=ecat_shm_pointer->reference.read_pointer%total_grp;
	 
	 if(wr_ptr==rd_ptr)
	 {
		 /*Buffer is empty*/
		 return false;
	 }
	 
	 *p_grp_ref=ecat_shm_pointer->reference.grp_ref[rd_ptr];
	 ecat_shm_pointer->reference.read_pointer=(rd_ptr+1)%total_grp;
	 return true;
	 
 }
 
 /*Push/write feedback*/
 bool edb_push_fdbk(GROUP_FEEDBACK* p_grp_fdbk)
 {
	
	 if(0==ecat_shm_pointer|| 0==p_grp_fdbk)
	 {
		 /*Null ecat pointer */
		 return false;
	 }
	 if(1!=ecat_shm_pointer->inited_done)
	 {
		 /*Ecat buffer not inited*/
		 return false;
	 }
	 
	 ecat_shm_pointer->feedback=*p_grp_fdbk;
	 
	 return true;
	 
 }
  
 /*Pull/read feedback*/
 bool edb_pull_fdbk(GROUP_FEEDBACK* p_grp_fdbk)
 {

	 if(0==ecat_shm_pointer)
	 {
		 /*Null ecat pointer */
		 return false;
	 }
	 if(1!=ecat_shm_pointer->inited_done)
	 {
		 /*Ecat buffer not inited*/
		 return false;
	 }
	 
	 *p_grp_fdbk=ecat_shm_pointer->feedback;
	 
	 return true;
 }
 
 