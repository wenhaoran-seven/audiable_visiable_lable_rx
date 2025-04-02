#include <string.h>
#include "fs.h"
#include "error.h"
#include "fsr_raid.h"
#include "log.h"
#include "flash.h"

extern fs_t* pg_fs;
fs_raid_t fs_raid; 

int fsr_repair(void);
static void fsr_ctx_m(void)
{ 
		pg_fs = &fs_raid.fs_m;	
}

static void fsr_ctx_b(void)
{ 
		pg_fs = &fs_raid.fs_b;	
}

int fsr_format(uint32_t fs_start_address,uint32_t backup_start_address,uint8_t sector_num)
{
	fsr_ctx_m();
  hal_fs_format_pre(fs_start_address,sector_num);
	
	fsr_ctx_b();
	hal_fs_format_pre(backup_start_address,sector_num);
	
	return 0;
}

int fsr_cmp(uint32_t fs_src_addr, uint32_t fs_backup_addr, uint16_t len)//fs_dst_addr = backup
{
	uint8_t src_data[FS_ITEM_LEN];
	uint8_t bak_data[FS_ITEM_LEN];
	uint16_t src_last_file_addr = 0,src_now_file_addr = 0;
	uint16_t bak_last_file_addr = 0,bak_now_file_addr = 0;
	uint32_t src_file_start_addr = 0,bak_file_start_addr = 0;
	
	for(int i=0;i<len/fs_raid.sector_size;i++)
	{		
			for(int j=0;j<(fs_raid.sector_size-sizeof(fs_cfg_t))/FS_ITEM_LEN;j++)
			{
				uint32_t offset_addr = sizeof(fs_cfg_t) +(fs_raid.sector_size*i)+(j*FS_ITEM_LEN);
				
				fs_spif_read(fs_backup_addr + offset_addr, bak_data, FS_ITEM_LEN);
				fs_spif_read(fs_src_addr + offset_addr, src_data, FS_ITEM_LEN);
       
				bak_now_file_addr = bak_data[0]|(bak_data[1]<<8);
				src_now_file_addr = src_data[0]|(src_data[1]<<8);

				if(((src_now_file_addr != src_last_file_addr)||(bak_now_file_addr != bak_last_file_addr))&& \
					((src_now_file_addr != 0xFFFF)||(bak_now_file_addr != 0xFFFF)))
				{
					src_file_start_addr = fs_src_addr + offset_addr;
					bak_file_start_addr = fs_backup_addr + offset_addr;
					//LOG("DIFF=%#x  \n", src_file_start_addr);
				}					
				
				src_last_file_addr = src_now_file_addr;
				bak_last_file_addr = bak_now_file_addr;
				
				uint32_t ret = memcmp(bak_data,src_data,FS_ITEM_LEN);//len = byte				
				
				if(ret != 0)
				{
						fs_item_t *bak_file_attr;
					  fs_item_t *src_file_attr;
						uint32_t src_file_line_end,bak_file_line_end;
						uint32_t offset_line,line_0;
					  uint32_t sector_data,data_in_next_sector;
					
						bak_file_attr = (fs_item_t *)(&bak_data[0]); //file attribute
					  src_file_attr = (fs_item_t *)(&src_data[0]);

					 /* data is illegally changed */
					  if(bak_file_attr == src_file_attr)
					  {
						  /* the data is diff */
						  fs_raid.erro_sta.fail_b = 1;
							fs_raid.erro_sta.fail_m = 1;
					  }
						
					  sector_data = 0x1000 - sizeof(fs_cfg_t) - 255*4; //one sector include data numbers
				
					  #if(FS_CRC_EN == 1)
					    src_file_attr->b.len += 4;//add crc16
					  #endif
	
					  data_in_next_sector = ((src_file_start_addr%0x1000 + (src_file_attr->b.len))>sector_data)?1:0;	
					
					  offset_line = (src_file_attr->b.len)/FS_ITEM_DATA_LEN + data_in_next_sector;				
						line_0 = ((src_file_attr->b.len)%FS_ITEM_DATA_LEN)? 1 : 0;
					
						src_file_line_end = src_file_start_addr + (offset_line + line_0 - 1)*FS_ITEM_LEN;	
					  bak_file_line_end = bak_file_start_addr + (offset_line + line_0 - 1)*FS_ITEM_LEN;
            //LOG("START=%#x  %#x, %#x \n", src_file_start_addr,offset_line,line_0);
						
						 /* power off when deleting/writing file,
						 if mask doing but not done,fail_m = 1 
					   if mask have done but backup not done,fail_b = 1 */
						 fs_spif_read(src_file_line_end, src_data, FS_ITEM_LEN);
						 fs_spif_read(bak_file_line_end, bak_data, FS_ITEM_LEN);							
						 //LOG("END=%#x  src=%#x ,bak=%#x \n", src_file_line_end,src_data[2],bak_data[2]);
						 
						 if(((src_file_attr->b.frame != ITEM_MF_E)&&(bak_file_attr->b.frame == ITEM_SF))||\
							  ((src_file_attr->b.pro == ITEM_USED)&&(bak_file_attr->b.pro == ITEM_USED))||\
						    ((src_file_attr->b.pro == ITEM_DEL)&&(bak_file_attr->b.pro == ITEM_DEL)))
						 {
							 fs_raid.erro_sta.fail_m = 1;//mask error
						 }
						 else
						 {
							 fs_raid.erro_sta.fail_b = 1;
						 }
						
					//LOG("a=%#x ,b=%#x \n", fs_raid.erro_sta.fail_m,fs_raid.erro_sta.fail_b);
					return ret;		
				}								
			}	
	}
	return 0;	
}

int fsr_cpy(uint32_t fs_dst_addr, uint32_t fs_src_addr)
{
	fs_cfg_t src_cfg;
	uint32_t offset = 0;
	
	for(int i=0;i<fs_raid.sec_num;i++)
	{
		fs_spif_read(fs_src_addr+fs_raid.sector_size*i,(uint8_t *)&src_cfg,sizeof(fs_cfg_t));
	
		if(src_cfg.sector_addr != 0xFFFFFFFF)
		{			
			src_cfg.sector_addr = fs_dst_addr;
		}
		else
		{
			fs_erase_ucds_one_sector(fs_dst_addr + fs_raid.sector_size*i);
			continue;
		}
		src_cfg.cpy_flag = SECTOR_DOING;//0x01 cpy start
		
		fs_erase_ucds_one_sector(fs_dst_addr + fs_raid.sector_size*i);		
		fs_spif_write(fs_dst_addr + fs_raid.sector_size*i,(uint8_t*)&src_cfg,sizeof(fs_cfg_t));
		
		offset = fs_raid.sector_size*i + sizeof(fs_cfg_t);	
		if(PPlus_SUCCESS != fs_spif_write(fs_dst_addr + offset,(void *)(fs_src_addr + offset),fs_raid.sector_size-sizeof(fs_cfg_t)))
		{
			fsr_repair();
			return PPlus_ERR_FS_WRITE_FAILED;
		}			
		else
		{					
			src_cfg.cpy_flag = SECTOR_DONE;//0x0 cpy finish
			fs_spif_write(fs_dst_addr + fs_raid.sector_size*i,(uint8_t*)&src_cfg, sizeof(fs_cfg_t));
			memset((void *)&src_cfg,0,sizeof(fs_cfg_t));			
		}		
	}
	 return PPlus_SUCCESS; 
}
void cpy_from_backup(void)
{
	fsr_cpy(fs_raid.start_addr,fs_raid.backup_addr);
}

void cpy_from_mask(void)
{
	fsr_cpy(fs_raid.backup_addr,fs_raid.start_addr);
}

int fsr_repair(void)
{
	fs_cfg_t sta_cfg;
	fs_cfg_t bak_cfg;
	
	LOG("repair \n");
	/* if they are illegal changed, format it */	
	if((fs_raid.erro_sta.fail_m)&&(fs_raid.erro_sta.fail_b)) //both M and B are illegal changed,
	{
		fsr_format(fs_raid.start_addr,fs_raid.backup_addr,fs_raid.sec_num);
		fs_raid.erro_sta.fail_m = 0;
		fs_raid.erro_sta.fail_b = 0;

		return 0;	
	}
	
	/* for break*/
	for(int i=0;i<fs_raid.sec_num;i++)
	{		
		fs_spif_read(fs_raid.start_addr + fs_raid.sector_size*i,(uint8_t *)&sta_cfg,sizeof(fs_cfg_t));
		fs_spif_read(fs_raid.backup_addr + fs_raid.sector_size*i,(uint8_t *)&bak_cfg,sizeof(fs_cfg_t));
		
		if((bak_cfg.gc_flag == SECTOR_DOING) || (bak_cfg.cpy_flag == SECTOR_DOING) || fs_raid.erro_sta.fail_b) //if gc_flag_b=1	cpy_flag_b=1, cpy mask
		{
			cpy_from_mask();
			break;
		}
		else if((sta_cfg.gc_flag == SECTOR_DOING) || (sta_cfg.cpy_flag == SECTOR_DOING) || fs_raid.erro_sta.fail_m ||(i == fs_raid.sec_num-1))
				{
					cpy_from_backup();
					break;	
				}
	}
		
		fs_raid.erro_sta.fail_m = 0;
		fs_raid.erro_sta.fail_b = 0;
		fsr_init(fs_raid.start_addr,fs_raid.backup_addr,fs_raid.sec_num);	
		return 0;	
}

int fsr_init(uint32_t fs_start_address,uint32_t backup_start_address,uint8_t sector_num)
{
	fs_raid.start_addr = fs_start_address;
	fs_raid.backup_addr = backup_start_address;
	fs_raid.sector_size = 4*1024;
	fs_raid.sec_num = sector_num;
	
	uint32_t ret = fsr_cmp(fs_start_address,backup_start_address,(fs_raid.sector_size)*sector_num);

	if(PPlus_SUCCESS != ret)
	{
		fs_raid.erro_sta.init_count ++;
		if(fs_raid.erro_sta.init_count > 2)	
		{	
			LOG("repair fail!! \n");
			return 1;
		}		
     return fsr_repair();		
	}
		
	fsr_ctx_m();
	if(PPlus_SUCCESS == hal_fs_init_pre(fs_start_address,sector_num))	
	{
		fsr_ctx_b();
		return hal_fs_init_pre(backup_start_address,sector_num);		
	}		
	else 
		return 1;		
}

int fsr_item_write(uint16_t id,uint8_t* buf,uint16_t len)
{
	fs_raid.erro_sta.fail_m = 0;
	fs_raid.erro_sta.fail_b = 0;
	
	fsr_ctx_m();
	if(PPlus_SUCCESS != hal_fs_item_write_pre(id,buf,len))
     fs_raid.erro_sta.fail_m = 1;

	fsr_ctx_b();
	if(PPlus_SUCCESS != hal_fs_item_write_pre(id,buf,len))
		fs_raid.erro_sta.fail_b = 1;

	if(fs_raid.erro_sta.fail_m || fs_raid.erro_sta.fail_b)
		return fsr_repair();
	
	return PPlus_SUCCESS;
}

int fsr_item_read(uint16_t id,uint8_t* buf,uint16_t buf_len,uint16_t* len)
{
	uint32_t ret,free_size;	

	fsr_ctx_m();	
	ret = hal_fs_item_read_pre(id,buf,buf_len,len);

	if(ret != PPlus_SUCCESS)
	{			
		fsr_ctx_b();
		ret = hal_fs_item_read_pre(id,buf,buf_len,len);

		if (ret == PPlus_SUCCESS)
		{
			//repair mask fs
			fsr_ctx_m();
			hal_fs_get_free_size_pre(&free_size);

			if(free_size >= buf_len)
			{
			  fsr_item_write(id,buf,buf_len);
			}
			else
			{
			  hal_fs_item_del_pre(id); //del id in mask
	      hal_fs_garbage_collect_pre(); //gc mask					
			  hal_fs_item_write_pre(id,buf,buf_len); //write mask

			  fsr_ctx_b();
			  hal_fs_item_del_pre(id); //del id in backup
	    	hal_fs_garbage_collect_pre(); //gc backeup
	    	hal_fs_item_write_pre(id,buf,buf_len); //write again 			  
			}
			
			fsr_ctx_m();	
			ret = hal_fs_item_read_pre(id,buf,buf_len,len);		
		}	
	}		

	return ret;
}


int fsr_item_del(uint16_t id)
{
	fs_raid.erro_sta.fail_m = 0;
	fs_raid.erro_sta.fail_b = 0;
	
	fsr_ctx_m();
	uint32_t ret = hal_fs_item_del_pre(id);
	if((PPlus_SUCCESS != ret)&&(ret != PPlus_ERR_FS_NOT_FIND_ID))
		fs_raid.erro_sta.fail_m = 1;
	
	fsr_ctx_b();
	ret = hal_fs_item_del_pre(id);
	if((PPlus_SUCCESS != ret)&&(ret != PPlus_ERR_FS_NOT_FIND_ID))
		fs_raid.erro_sta.fail_b = 1;
	
	if(fs_raid.erro_sta.fail_m || fs_raid.erro_sta.fail_b)
		return fsr_repair();
	else
		return ret;
}

int fsr_get_free_size(uint32_t* free_size)
{
	fsr_ctx_m();
	if(PPlus_SUCCESS == hal_fs_get_free_size_pre(free_size))
	{
		fsr_ctx_b(); 
		return hal_fs_get_free_size_pre(free_size);		
	}
	return 1;
}

int fsr_get_garbage_size(uint32_t* garbage_file_num)
{
	uint32_t ret_m,ret_b;
	fsr_ctx_m();
	ret_m = hal_fs_get_garbage_size_pre(garbage_file_num);
		
	fsr_ctx_b(); 
	ret_b = hal_fs_get_garbage_size_pre(garbage_file_num);
	
  if(ret_m == ret_b)		return ret_m;
	else  return 1;
}

int fsr_garbage_collect(void)
{		
	fs_raid.erro_sta.fail_m = 0;
	fs_raid.erro_sta.fail_b = 0;
	
	fsr_ctx_m(); 	
	if(PPlus_SUCCESS != hal_fs_garbage_collect_pre())
		fs_raid.erro_sta.fail_m = 1;
	
	fsr_ctx_b(); 		
	if(PPlus_SUCCESS != hal_fs_garbage_collect_pre())
		fs_raid.erro_sta.fail_b = 1;
	
	if(fs_raid.erro_sta.fail_m || fs_raid.erro_sta.fail_b)
		return fsr_repair();
	else
		return PPlus_SUCCESS;		
}	
