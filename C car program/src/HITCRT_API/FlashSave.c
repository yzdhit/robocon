#include "stm32f4xx.h"
#include "FlashSave.h"
#include "HITCRT_Types.h"
#include "PrivateFunction.h"


UINT32 i=0;



SSHORT16 GetOneShort16(UINT32 uiReadAddr)
{
	SSHORT16  ssTemp;
	ssTemp=(SSHORT16)(*(__IO SINT32*)(ADDR_FLASH_SECTOR_11+(uiReadAddr<<2)));
	return ssTemp;
}

SINT32	GetOneInt32(UINT32 uiReadAddr)
{
	SINT32	ssTemp;
	ssTemp=*(__IO SINT32*)(ADDR_FLASH_SECTOR_11+(uiReadAddr<<2));
	return ssTemp;
}



UCHAR8  SaveOneWord32(SINT32 siData,UINT32 uiWriteAddr)
{
   UCHAR8 ucRet = 0; 
//   UINT32 uiStartTime = 0;
   //UINT32 uiEndTime = 0;
   
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();
 // uiStartTime = GetCurTime();
  for(i=0;i<MAX_FLASH_LEN;i++)
  	{
  		g_siFlashValue[i]=*(__IO SINT32*)(ADDR_FLASH_SECTOR_11+(i<<2));
  	}
  
  /* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 


	    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	       be done by word */ 
   if (FLASH_EraseSector( FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	    { 
	      /* Error occurred while sector erase. 
	         User can add here some code to deal with this error  */
	      	return 0;
	    }
   // uiEndTime = GetCurTime();
	g_siFlashValue[uiWriteAddr]  	=siData;

	for(i=0;i<MAX_FLASH_LEN;i++)
		{
		  if (FLASH_ProgramWord(ADDR_FLASH_SECTOR_11+(i<<2), g_siFlashValue[i]) == FLASH_COMPLETE)
		    {
		      	
				ucRet =  0x01;
		    }  
		    else
		    {
		      		ucRet =  0x00;
				 g_ucFlashWrongNum++;
		    }
		}

   /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
      FLASH_Lock(); 
	g_ucFlashFlag=ucRet;
	return ucRet;
}



UCHAR8  SaveAllWord32(void)
{
   UCHAR8 ucRet = 0; 
  // UINT32 uiStartTime = 0;
  // UINT32 uiEndTime = 0;
   
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();
 // uiStartTime = GetCurTime();
 
  /* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 


	    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	       be done by word */ 
   if (FLASH_EraseSector( FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	    { 
	      /* Error occurred while sector erase. 
	         User can add here some code to deal with this error  */
	      	return 0;
	    }
   // uiEndTime = GetCurTime();

	for(i=0;i<MAX_FLASH_LEN;i++)
		{
		  if (FLASH_ProgramWord(ADDR_FLASH_SECTOR_11+(i<<2), g_siFlashValue[i]) == FLASH_COMPLETE)
		    {
		      	
				ucRet =  0x01;
		    }  
		    else
		    {
		      		ucRet =  0x00;
				 g_ucFlashWrongNum++;
		    }
		}

   /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
      FLASH_Lock(); 
	g_ucFlashFlag=ucRet;
	return ucRet;
}




	



